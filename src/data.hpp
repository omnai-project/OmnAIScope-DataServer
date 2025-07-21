// #define BOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "../ai_omniscope-v2-communication_sw/src/OmniscopeSampler.hpp"
#include "CLI/CLI.hpp"
#include <tuple>
#include <functional>
#include <nlohmann/json.hpp>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <mutex>
#include <csignal>
#include "crow.h"
#include "crow/middlewares/cors.h"
#include "sample.pb.h"
#include <optional>

// CLASSES/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Writer;
class Measurement;
class ControlWriter; 
enum class DataDestination;
enum class FormatType;

// GLOBAL VARIABLES//////////////////////////////////////////////////////////////////////////////////////////////////

using val_T = double;
using ts_T = double; // timestamp

using sample_T = std::tuple<ts_T, val_T, std::optional<std::vector<val_T>>>;

inline OmniscopeDeviceManager deviceManager{};
inline std::vector<std::shared_ptr<OmniscopeDevice>> devices;
inline std::optional<OmniscopeSampler> sampler{};
inline std::map<Omniscope::Id, std::vector<std::pair<double, double>>> captureData;
std::atomic<bool> running{true};
bool verbose{false};
std::queue<sample_T> sampleQueue;
std::queue<sample_T> saveDataQueue; 
std::mutex sampleQueueMutex;
std::mutex wsDataQueueMutex;
nlohmann::json HeaderJSON;
std::queue<nlohmann::json> wsPackagesQueue;
std::queue<std::string> wsCSVPackagesQueue;
std::queue<std::string> wsBinaryPackagesQueue;
std::thread wsDataQueueThread;
std::thread sendDataviaWSThread;
std::atomic<bool> WEBSOCKET_ACTIVE{false};
std::atomic<bool> websocketConnectionActive{false};
crow::App<crow::CORSHandler> crowApp;
std::thread websocket;
static std::atomic<int> dataPointsInSampleQue(0);
static std::atomic<int> dataPointsInSaveQue(0); 
static int Datenanzahl(0);
static bool startWriter = true;
struct WSContext {
    std::shared_ptr<Measurement> currentMeasurement;
    std::jthread msmntThread;  
    std::jthread sendThread; 
}; 
WSContext wsCtx;

// FUNCTION HEADER/////////////////////////////////////////////////////////////////////////////////////////////////////

void initDevices();
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void selectDevices(std::vector<std::string> &UUID);
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string &);
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t> &);
double round_to(double, int);
void sendDataStreamToWS(std::vector<std::string> &, std::string &, bool &, bool &);
void resetDevices();
void clearAllQueues();
void stopAndJoinWSThread();
void ExitProgramm();
std::string colorToString(const std::tuple<uint8_t, uint8_t, uint8_t> &rgb);
nlohmann::json getDevicesAsJson(bool);
Measurement parseWSDataToMeasurement(const std::string &data);
void processDeque(std::stop_token, crow::websocket::connection& , std::shared_ptr<Measurement>);
template <typename T, typename Container = std::deque<T>>
void clearQueue(std::queue<T, Container> &q)
{
    std::queue<T, Container> empty;
    std::swap(q, empty);
};

// CLASSES/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Transform data map std::map<Omniscope::Id, std::vector<std::pair<double,double>>> to std::queue<sample_T>
 * This class starts transformData function on construction with params from the constructor
 * @param captureData map of OmnAIScope data
 * @param handle Handle to data queue with samples
 * @param dataPointsInSampleQue count amount of samples in Que, check against SegFault
 * @param samplingRate wanted samples per second, between 10 and 100000 per second
 */
class QueueFormatter
{
private:
    std::queue<sample_T> &handle;
    int samplingRate;
    /**
     * @brief Transform data map std::map<Omniscope::Id, std::vector<std::pair<double,double>>> to std::queue<sample_T>
     * @param captureData map of OmnAIScope data
     * @param handle Handle to data queue with samples
     * @param dataPointsInSampleQue count amount of samples in Que, check against SegFault
     * @details
     * CaptureData contains a different amount of data points each time, samples at the edge are sometimes ignored at specific sampling rates
     * Values are rounded to 5th decimal place
     */
    void transformData(std::map<Omniscope::Id, std::vector<std::pair<double, double>>> &captureData, std::queue<sample_T> &handle, std::queue<sample_T> &saveHandle, std::atomic<int> &dataPointsInSampleQue)
    {
        // TODO: sampling should be improved
        int currentPosition = 0;
        ts_T timeStamp = 0;
        val_T firstX = 0;
        std::optional<std::vector<val_T>> otherX;
        int sampleQuotient = 10000;

        if (samplingRate > 10 && samplingRate < 100000)
        {
            sampleQuotient = 100000 / samplingRate;
        }
        if (verbose)
        {
            std::cout << "sampleQuotient: " << sampleQuotient << std::endl;
        }
        if (captureData.empty())
        {
            return;
        }

        // Access to first device
        const auto &[firstId, firstDeviceData] = *captureData.begin();
        size_t vectorSize = firstDeviceData.size();

        // Save all data in the saveQueue
        for(int i = 0; i < vectorSize-1; ++i){
            if(!running) return; 

            timeStamp = round_to(firstDeviceData[i].first, 5);
            firstX = round_to(firstDeviceData[i].second, 5);

            // values from other devices
            otherX = std::vector<val_T>();
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it)
            {
                const auto &deviceData = it->second;
                if (i < deviceData.size())
                {
                    otherX->push_back(round_to(deviceData[i].second, 5));
                }
            }

            sample_T sample = std::make_tuple(timeStamp, firstX, otherX);

            // thread save access to handle and counter
            saveHandle.push(sample);
            dataPointsInSaveQue++; 
        }

        for (currentPosition = 0; currentPosition < vectorSize; ++currentPosition)
        {
            if ((currentPosition + sampleQuotient) < vectorSize)
            {
                currentPosition = currentPosition + sampleQuotient - 1;
            }
            else
                return;

            if (!running)
            {
                return;
            }

            // values from first device
            timeStamp = round_to(firstDeviceData[currentPosition].first, 5);
            firstX = round_to(firstDeviceData[currentPosition].second, 5);

            // values from other devices
            otherX = std::vector<val_T>();
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it)
            {
                const auto &deviceData = it->second;
                if (currentPosition < deviceData.size())
                {
                    otherX->push_back(round_to(deviceData[currentPosition].second, 5));
                }
            }

            sample_T sample = std::make_tuple(timeStamp, firstX, otherX);

            // thread save access to handle and counter
            std::lock_guard<std::mutex> lock(sampleQueueMutex);
            handle.push(sample);
            dataPointsInSampleQue++;
        }
    }

public:
    QueueFormatter(std::map<Omniscope::Id, std::vector<std::pair<double, double>>> &captureData, std::queue<sample_T> &handle, std::queue<sample_T> &saveHandle, std::atomic<int> &dataPointsInSampleQue, int &samplingRate)
        : handle(handle), samplingRate(samplingRate)
    {
        transformData(captureData, handle, saveHandle, dataPointsInSampleQue);
    }
    ~QueueFormatter()
    {
    }
};

// MEASUREMENT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief defines possible datadestinations
 */
enum class DataDestination
{
    LOCALFILE,
    WS
};

/**
 * defines possible format types
 */
enum class FormatType
{
    CSV,
    JSON,
    BINARY,
    UNKNOWN
};

/**
 * @brief create a Measurement on construction
 *  Provides functionality to start a measurement
 * @param uuids of devices
 * @param samplingRate of data
 * @param format of data
 * @param dataDestination of data
 * @param filePath optional filePath for output
 */
class Measurement : public std::enable_shared_from_this<Measurement>
{
public:
    std::vector<std::string> uuids;
    int samplingRate;
    FormatType format = FormatType::UNKNOWN;
    DataDestination dataDestination;
    std::string filePath; // optional

    Measurement(std::vector<std::string> uuids, std::string filePath, int samplingRate, FormatType fmt, DataDestination destination)
        : uuids(uuids), samplingRate(samplingRate), format(fmt), dataDestination(destination), filePath(filePath) {}
    Measurement() = default;
    ~Measurement() {}
    void start(std::stop_token stopToken, ControlWriter &controlWriter);
    void startLocal(ControlWriter &controlWriter); 
};

// WRITER//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief On construction: Starts to write a measurement in a object or file depending on the given presets
 * Stops to write on destruction
 * @param measurement object of type Measurement
 * @param handle to a data queue
 * @param dataPointsInSampleQue amount of datapoints in the data queue
 * @param jsonHandle and other handles / currently not sure why they are needed
 */
class Writer
{   // write data into various formats, including a json object for the Websocket
private:
    std::ofstream outFile;
    std::queue<sample_T> &handle;
    std::queue<nlohmann::json> &jsonHandle;
    std::queue<std::string> &csvHandle;
    std::queue<std::string> &binaryHandle;
    std::thread writerThread;
    std::shared_ptr<Measurement> measurement;

    nlohmann::json createJsonObject(const std::vector<std::string> &UUIDs)
    {
        nlohmann::json jsonObject;

        // Metadata-Objekt erstellen
        nlohmann::json metadata;
        nlohmann::json devices = nlohmann::json::array();

        // Devices-Daten hinzufügen
        for (const auto &uuid : UUIDs)
        {
            nlohmann::json device;
            device["UUID"] = uuid;
            devices.push_back(device);
        }

        metadata["devices"] = devices;

        // Metadaten in das Hauptobjekt einfügen
        jsonObject["metadata"] = metadata;

        return jsonObject;
    }

    void write_csv(std::atomic<int> &dataPointsInSampleQue)
    {  
        while (running && !saveDataQueue.empty())
        {   
            if (dataPointsInSampleQue > 0)
            {
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                outFile << std::get<0>(sample) << " , " << std::get<1>(sample) << " ";
                const auto &optionalValues = std::get<2>(sample);
                if (optionalValues)
                {
                    for (size_t i = 0; i < optionalValues->size(); i++)
                    {
                        outFile << (*optionalValues)[i];
                        if (i < optionalValues->size() - 1)
                        {
                            outFile << " , ";
                        }
                    }
                }
                outFile << "\n";
                dataPointsInSampleQue--;
            }
        }
    }

    void write_json(std::atomic<int> &dataPointsInSampleQue)
    {

        outFile << "\"data\": " << "[";
        while (running && !saveDataQueue.empty())
        {  
            if (dataPointsInSampleQue > 0)
            {
                int i = 0;
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                outFile << "{\"timestamp\" : " << std::get<0>(sample) << "," << "\"value\": [" << std::get<1>(sample);
                if (i < measurement->uuids.size() - 1)
                {
                    outFile << ",";
                    i++;
                }
                const auto &optionalValues = std::get<2>(sample);
                if (optionalValues)
                {
                    for (const auto &value : optionalValues.value())
                    {
                        outFile << value;
                        if (i < measurement->uuids.size() - 1)
                        {
                            outFile << ",";
                            i++;
                        }
                    }
                }
                outFile << "]" << "}" << ",";
                dataPointsInSampleQue--;
                i = 0;
            }
        }
        outFile << "]";
    }

    void write_console(std::atomic<int> &dataPointsInSampleQue)
    {
        while (running)
        {
            if (dataPointsInSampleQue > 0)
            {
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                std::cout << "\r[";

                std::cout << std::get<0>(sample) << " , " << std::get<1>(sample) << " ";
                const auto &optionalValues = std::get<2>(sample);
                if (optionalValues)
                {
                    if (optionalValues)
                    {
                        for (size_t i = 0; i < optionalValues->size(); i++)
                        {
                            std::cout << (*optionalValues)[i];
                            if (i < optionalValues->size() - 1)
                            {
                                std::cout << " , ";
                            }
                        }
                    }
                }

                std::cout << "]" << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                dataPointsInSampleQue--;
            }
        }
    }

    void write_JsonObject(std::atomic<int> &dataPointsInSampleQue, std::mutex &jsonMutex)
    {
        constexpr int batchSize = 1;
        nlohmann::json currentBatch;
        currentBatch["data"] = nlohmann::json::array();

        int batchCounter = 0;

        while (websocketConnectionActive)
        {
            if (dataPointsInSampleQue > 0)
            {
                sample_T sample;

                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                // add sample to Json object
                nlohmann::json sampleObject;
                sampleObject["timestamp"] = std::get<0>(sample);
                sampleObject["value"] = nlohmann::json::array();
                sampleObject["value"].push_back(std::get<1>(sample));

                const auto &optionalValues = std::get<2>(sample);
                if (optionalValues)
                {
                    for (size_t i = 0; i < optionalValues->size(); ++i)
                    {
                        sampleObject["value"].push_back((*optionalValues)[i]);
                    }
                }

                // push in JSON queue:

                currentBatch["data"].push_back(sampleObject);
                batchCounter++;
                dataPointsInSampleQue--;

                if (batchCounter >= batchSize)
                {
                    std::lock_guard<std::mutex> lock(jsonMutex);
                    jsonHandle.push(currentBatch);

                    currentBatch["data"] = nlohmann::json::array();
                    batchCounter = 0;
                }
                Datenanzahl++;
            }
        }
    }
    void write_CsvBatch(std::atomic<int> &dataPointsInSampleQue, std::mutex &jsonMutex)
    {
        std::string currentBatch;
        int batchSize = 1;
        int batchCounter = 0;

        while (websocketConnectionActive)
        {
            if (dataPointsInSampleQue > 0)
            {
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3);

                oss << std::get<0>(sample) << ", " << std::get<1>(sample);

                const auto &optionalValues = std::get<2>(sample);
                if (optionalValues.has_value())
                {
                    for (const auto &val : optionalValues.value())
                    {
                        oss << ", " << val;
                    }
                }

                oss << "\n";
                std::string sampleLine = oss.str();
                currentBatch += sampleLine;
                ++batchCounter;
                --dataPointsInSampleQue;

                if (batchCounter >= batchSize)
                {
                    std::lock_guard<std::mutex> lock(jsonMutex);
                    csvHandle.push(currentBatch);
                    currentBatch.clear();
                    batchCounter = 0;
                }
                ++Datenanzahl;
            }
        }

        if (!currentBatch.empty())
        {
            std::lock_guard<std::mutex> lock(jsonMutex);
            csvHandle.push(currentBatch);
        }
    }

    void write_ProtobufSamples(std::atomic<int> &dataPointsInSampleQue, std::mutex &jsonMutex)
    {
        while (websocketConnectionActive)
        {
            if (dataPointsInSampleQue > 0)
            {
                sample_T sample;
                {
                    std::lock_guard<std::mutex> lock(sampleQueueMutex);
                    sample = handle.front();
                    handle.pop();
                }

                Sample sampleMsg;
                sampleMsg.set_timestamp(std::get<0>(sample));

                sampleMsg.add_values(std::get<1>(sample));

                const auto &optValues = std::get<2>(sample);
                if (optValues)
                {
                    for (const auto &val : *optValues)
                    {
                        sampleMsg.add_values(val);
                    }
                }

                std::string serializedSample;
                if (!sampleMsg.SerializeToString(&serializedSample))
                {
                    std::cerr << "Protobuf-Serialisierung fehlgeschlagen!" << std::endl;
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(jsonMutex);
                    binaryHandle.push(serializedSample);
                }

                --dataPointsInSampleQue;
                ++Datenanzahl;
            }
        }
    }

    void write(std::atomic<int> &dataPointsInSampleQue, std::mutex &jsonMutex)
    {

        if (verbose)
        {
            std::cout << "Writer startet" << std::endl;
        }
        if (measurement->dataDestination == DataDestination::WS)
        {
            if (measurement->format == FormatType::JSON)
            {
                write_JsonObject(dataPointsInSampleQue, jsonMutex);
            }
            else if (measurement->format == FormatType::CSV)
            {
                write_CsvBatch(dataPointsInSampleQue, jsonMutex);
            }
            else if (measurement->format == FormatType::BINARY)
            {
                write_ProtobufSamples(dataPointsInSampleQue, jsonMutex);
            }
        }
        else if (measurement->dataDestination == DataDestination::LOCALFILE)
        {
            if (!measurement->filePath.empty())
            {
                if (measurement->format == FormatType::CSV)
                {
                    write_csv(dataPointsInSampleQue);
                }
                else if (measurement->format == FormatType::JSON)
                {
                    write_json(dataPointsInSampleQue);
                }
            }
            else
            {
                write_console(dataPointsInSampleQue);
            }
        }

        if (verbose)
        {
            std::cout << "Schreiben beendet" << std::endl;
        }
    }

public:
    Writer(std::shared_ptr<Measurement> measurement, std::queue<sample_T> &handle, std::atomic<int> &dataPointsInSampleQue, std::queue<nlohmann::json> &jsonHandle, std::mutex &jsonMutex, std::queue<std::string> &csvHandle, std::queue<std::string> &binaryHandle)
        : handle(handle), measurement(measurement), jsonHandle(jsonHandle), csvHandle(csvHandle), binaryHandle(binaryHandle)
    {
        std::cout << std::fixed << std::setprecision(3); 
        // Seperation by data destination
        if (measurement->dataDestination == DataDestination::WS)
        {
            if (measurement->format == FormatType::JSON)
            {
                HeaderJSON = createJsonObject(measurement->uuids);
            }
        }
        else if (measurement->dataDestination == DataDestination::LOCALFILE)
        {
            if (!measurement->filePath.empty())
            {
                outFile.open(measurement->filePath, std::ios::app);
                if (!outFile)
                {
                    throw std::ios_base::failure("Error opening file: " + measurement->filePath);
                } // Write header
                else if (measurement->format == FormatType::CSV)
                {
                    outFile << "Timestamp [s]" << " , ";
                    for (size_t i = 0; i < measurement->uuids.size(); ++i)
                    {
                        outFile << measurement->uuids[i];
                        if (i < measurement->uuids.size() - 1)
                        {
                            outFile << " , ";
                        }
                    }
                    outFile << "\n";
                }
                else if (measurement->format == FormatType::JSON)
                { 
                    outFile << "{\"metadata\": {";
                    outFile << "\"" << "devices" << "\"" << ":" << "[" << "{";
                    for (size_t i = 0; i < measurement->uuids.size(); ++i)
                    {
                        outFile << "\"" << "UUID" << "\"" << ": " << "\"" << measurement->uuids[i] << "\"";
                        if (i < measurement->uuids.size() - 1)
                        {
                            outFile << ",";
                        }
                    }
                    outFile << "}" << "]"; // optional metadata: measurement, host
                    outFile << "},";
                }
            }
            else
            {
                std::cout << "Timestamp [s]" << " , ";
                for (size_t i = 0; i < measurement->uuids.size(); ++i)
                {
                    std::cout << measurement->uuids[i] << " [V]";
                    if (i < measurement->uuids.size() - 1)
                    {
                        std::cout << " , ";
                    }
                }
                std::cout << "\n";
            }
        }

        // Starte den Thread erst nach dem Öffnen der Datei
        writerThread = std::thread(&Writer::write, this, std::ref(dataPointsInSampleQue), std::ref(jsonMutex));
    }

    ~Writer()
    {
        if (!measurement->filePath.empty())
        {
            if (measurement->format == FormatType::JSON)
            {
                outFile << "}";
            }
            // write Trailer
            outFile.flush();
            outFile.close();
        }
        else
            std::cout << "Schreiben beendet" << std::endl;

        if (verbose)
        {
            std::cout << "Writer destroyed" << std::endl;
        }
        if (writerThread.joinable())
        {
            writerThread.join();
        }
    }
};

class ControlWriter {
    public: 
        void printOrWriteData(std::shared_ptr<Measurement> measurement, std::optional<std::stop_token> stopToken = std::nullopt);
        void stopWriter();   
        void saveData(WSContext& wsCtx);

    private: 
        Writer* writer_{nullptr}; 
};
// FUNCTIONS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief Start a measurement on Websocket with the Measurement object presets
     */
    void Measurement::start(std::stop_token stopToken, ControlWriter &controlWriter)
    {
        auto self = shared_from_this();
        searchDevices(); // Init Scopes
        selectDevices(self->uuids); // select only chosen devices

        while (!stopToken.stop_requested())
        {
            controlWriter.printOrWriteData(self, stopToken);  
        }
    }

    void Measurement::startLocal(ControlWriter &controlWriter){
        auto self = shared_from_this();
        searchDevices(); // Init Scopes
        selectDevices(self->uuids); // select only chosen devices
        while (running)
            {
                controlWriter.printOrWriteData(self); 
            }
        resetDevices();
    }
    /**
    * @brief Starts the QueueFormatter, waits till formatter stops, starts writer with given measurement presets
    */
    void ControlWriter::printOrWriteData(std::shared_ptr<Measurement> measurement, std::optional<std::stop_token> stopToken)
    {
        if (sampler.has_value())
        {
            captureData.clear();
            int vectorSize = 0;

            while (vectorSize < 100000 &&
               !(stopToken && stopToken->stop_requested()))
            {
                if (sampler.has_value())
                {
                    sampler->copyOut(captureData); // all copy out data is cleared from sampler by copyOut function
                }
                if (captureData.empty())
                {
                    return;
                }
                auto it = captureData.begin();
                auto &[updatedId, updatedDeviceData] = *it;
                vectorSize = static_cast<int>(updatedDeviceData.size());
            }
            if (stopToken && stopToken->stop_requested())
            return;     

            QueueFormatter *queueFormatter = new QueueFormatter(captureData, sampleQueue, saveDataQueue, dataPointsInSampleQue, measurement->samplingRate);
            delete queueFormatter;

            if (startWriter)
            {
                writer_= new Writer(measurement, sampleQueue, dataPointsInSampleQue, wsPackagesQueue, wsDataQueueMutex, wsCSVPackagesQueue, wsBinaryPackagesQueue);
                startWriter = false;
            }
        }
    }

    void ControlWriter::saveData(WSContext& wsCtx){
        if (startWriter)
        {
            writer_= new Writer(wsCtx.currentMeasurement, saveDataQueue, dataPointsInSaveQue, wsPackagesQueue, wsDataQueueMutex, wsCSVPackagesQueue, wsBinaryPackagesQueue);
            startWriter = false;
        }
    }
    void ControlWriter::stopWriter() {
    // this could lead to race condition but in rare cases 
        if(!websocketConnectionActive){
            if (writer_) {
                delete writer_;
                writer_ = nullptr;
            }
        }
    }
// Closing the Programm and WS Connections savely:

// TODO :  Make this cleaner, this is a mess
/**
 * @brief stop programm with SIGINT
 */
void customSignalHandler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\ngot SIGINT. Stopping the programm..." << std::endl;
        running = false;
    }
}

/**
 * @brief Close all WS threads, WS and resetDevices
 */
void ExitProgramm()
{
    if (WEBSOCKET_ACTIVE)
    {
        crowApp.stop();
    }
    resetDevices();
    stopAndJoinWSThread();
    std::cout << "Programm was closed correctly, all threads closed" << std::endl;
}

/**
 * @brief Stops all devices, reset sampler, clear devices and device Manager, Runs ws and sample queues
 */
void resetDevices()
{
    if (sampler.has_value())
    {
        for (auto &device : sampler->sampleDevices)
        {
            device.first->send(Omniscope::Stop{});
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sampler.reset();
    devices.clear();
    deviceManager.clearDevices();
    dataPointsInSampleQue = 0;
    clearAllQueues();
}
/**
 * @brief clear sampleQueue, wsPackagesQueue, wsBinaryPackagesQueue, wsCSVPackagesQueue
 */
void clearAllQueues()
{

    if (!sampleQueue.empty())
    {
        std::lock_guard<std::mutex> lock(sampleQueueMutex);
        clearQueue(sampleQueue);
    }
    if (!wsPackagesQueue.empty())
    {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsPackagesQueue);
    }
    if (!wsBinaryPackagesQueue.empty())
    {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsBinaryPackagesQueue);
    }
    if (!wsCSVPackagesQueue.empty())
    {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsCSVPackagesQueue);
    }
}
/** @brief stops and joins websocket thread */
void stopAndJoinWSThread()
{
    if (WEBSOCKET_ACTIVE)
    {
        WEBSOCKET_ACTIVE = false;
        if (websocket.joinable())
        {
            websocket.join();
            if (verbose)
            {
                std::cout << "Websocket thread was joined" << std::endl;
            }
        }
    }
}

// CONNECT_DEVICES//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initalize OmnAIScope devices connected via USB
 * gets available devices
 * sets LED color
 * gets information about calibration
 *
 * Devices is a global variable
 */
void initDevices()
{
    constexpr int VID = 0x2e8au;
    constexpr int PID = 0x000au;
    if (verbose)
    {
        std::cout << "Search for devices" << std::endl;
    }

    devices = deviceManager.getDevices(VID, PID);
    for (auto &device : devices)
    {
        auto metaDataCb = [&](auto const &msg)
        {
            if (std::holds_alternative<Omniscope::MetaData>(msg))
            {
                parseDeviceMetaData(std::get<Omniscope::MetaData>(msg), device);
            }
        };
        auto id = device->getId().value();
        auto sampleRate = static_cast<double>(id.sampleRate);
        device->setTimeScale(static_cast<double>(1 / sampleRate));

        auto [r, g, b] = uuidToColor(device->getId()->serial);
        device->send(Omniscope::SetRgb{static_cast<std::uint8_t>(static_cast<int>(r)),
                                       static_cast<std::uint8_t>(static_cast<int>(g)),
                                       static_cast<std::uint8_t>(static_cast<int>(b))});
        // set Callback for MetaData
        device->setMessageCallback(metaDataCb);
        device->send(Omniscope::GetMetaData{});
    }
}
/**
 * @brief cleans up old data, init new devices, pause between checks
 */
void searchDevices()
{
    if (!sampler.has_value())
    {
        devices.clear();
        deviceManager.clearDevices();
        initDevices();
    }
}
/**
 * @brief Erase not selected devices per UUIDs from deviceManager
 * @param UUID vector of selected UUIDs
 */
void selectDevices(std::vector<std::string> &UUID)
{
    if (!devices.empty() && !sampler.has_value())
    {   // move the device in the sampler
        std::cout << "Devices where found and are emplaced" << std::endl;
        for (const auto &device : devices)
        {
            devices.erase(
                std::remove_if(
                    devices.begin(),
                    devices.end(),
                    [&UUID](const std::shared_ptr<OmniscopeDevice> &device)
            {
                return std::find(UUID.begin(), UUID.end(), device->getId()->serial) == UUID.end();
            }),
            devices.end());
        }
        if (!devices.empty())
        {
            sampler.emplace(deviceManager, std::move(devices));
            if (sampler.has_value())
            {
                for (auto &device : sampler->sampleDevices)
                {
                    device.first->send(Omniscope::Start{});
                }
            }
        }
    }
}
/**
 * @brief check if scope is calibrated via given metaData string
 */
void parseDeviceMetaData(Omniscope::MetaData metaData,
                         std::shared_ptr<OmniscopeDevice> &device)
{
    try
    {
        nlohmann::json metaJson = nlohmann::json::parse(metaData.data);
        if (verbose)
        {
            fmt::println("{}", metaJson.dump());
        }
        device->setScale(std::stod(metaJson["scale"].dump()));
        device->setOffset(std::stod(metaJson["offset"].dump()));
        device->setEgu(metaJson["egu"]);
    }
    catch (...)
    {
        if (verbose)
        {
            fmt::print("This Scope is not calibrated: {}", metaData.data);
        }
    }
}

// DATAHANDELING///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Prints devices from global device variable, sets running = false after that --> should stop application
 */
void printDevices()
{

    // get IDs
    if (devices.empty())
    {
        std::cout << "No devices are connected. Please connect a device and start again" << std::endl;
    }
    else
    {
        std::cout << "The following devices are connected:" << std::endl;
        for (const auto &device : devices)
        {
            std::string deviceId = device->getId()->serial;
            fmt::print("{}Device: {}\033[0m\n", rgbToAnsi(uuidToColor(deviceId)), deviceId);
        }
        devices.clear();
        deviceManager.clearDevices();
        running = false;
    }
}
/**
 * @brief returns device info as JSON object from global devices variable
 */
nlohmann::json getDevicesAsJson(bool allInfo)
{
    nlohmann::json responseJson;

    if (devices.empty())
    {
        return {
            {"error", "No devices are connected. Please connect a device and start again."}};
    }

    auto versionToJson = [](auto v)
    {
        return nlohmann::json{
            {"major", v.major},
            {"minor", v.minor},
            {"patch", v.patch}};
    };

    nlohmann::json response{
        {"devices", nlohmann::json::array()},
        {"colors", nlohmann::json::array()}};

    if (allInfo)
    {
        response["hardwareInfo"] = nlohmann::json::array();
        response["firmwareInfo"] = nlohmann::json::array();
        response["offset"] = nlohmann::json::array();
        response["scale"] = nlohmann::json::array();
    }

    /**
     * Iteration over the devices to extract all infos
     */
    for (const auto &dev : devices)
    {
        const auto &id = dev->getId().value();
        const std::string uuid = dev->getId()->serial;
        const auto [r, g, b] = uuidToColor(uuid);

        response["devices"].push_back({{"UUID", uuid}});
        response["colors"].push_back({{"color", {{"r", r}, {"g", g}, {"b", b}}}});

        if (!allInfo)
            continue;

        response["hardwareInfo"].push_back(versionToJson(id.hwVersion));
        response["firmwareInfo"].push_back(versionToJson(id.swVersion));

        if (auto o = dev->getOffset(); o)
            response["offset"].push_back(*o);
        if (auto s = dev->getScale(); s)
            response["scale"].push_back(*s);
    }

    devices.clear();
    deviceManager.clearDevices();
    return response;
}
/**
 * @brief Transform given uuids via hashing into rgb colors
 */
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string &uuid)
{
    // std::hash anwenden
    size_t hashValue = std::hash<std::string> {}(uuid);

    // Die ersten 3 Bytes extrahieren
    uint8_t r = (hashValue & 0xFF);       // Erste 8 Bits
    uint8_t g = (hashValue >> 8) & 0xFF;  // Zweite 8 Bits
    uint8_t b = (hashValue >> 16) & 0xFF; // Dritte 8 Bits

    return {r, g, b};
}
/**
 * Transform rgb colors to ANSI color code
 */
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t> &rgb)
{
    auto [r, g, b] = rgb;
    return fmt::format("\033[38;2;{};{};{}m", r, g, b);
}
/**
 * @brief Transform rgb colors to string format
 */
std::string colorToString(const std::tuple<uint8_t, uint8_t, uint8_t> &rgb)
{
    auto [r, g, b] = rgb;
    return fmt::format("{};{};{}", r, g, b);
}
/**
 * @brief Round value to specific decimals place
 * @param value to round
 * @param decimal places rounded
 */
double round_to(double value, int decimals)
{
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
}

/// WEBSOCKET HANDLING ///////////////////////////////////////////////////////////////////////////////////////////////////
/** 
* @brief Thread Save queue, pop waits till push is finished, only removes from non empty queue 
*/
template<class T>
class TSQueue {
    std::queue<T> q_;
    std::mutex    m_;
    std::condition_variable cv_;
public:
    void push(T v) {
        { std::lock_guard lg(m_); q_.push(std::move(v)); }
        cv_.notify_one();                 
    }
    T pop() {                            
        std::unique_lock ul(m_);
        cv_.wait(ul,[&]{ return !q_.empty(); });
        T v = std::move(q_.front());
        q_.pop();
        return v;
    }
};

enum class CmdType {
    START, STOP, SAVE, UNKNOWN
};

/**
* @brief Metadataobject containing uuids, samplingRate and a format 
 */
struct StartMeta {
    std::vector<std::string> uuids; 
    int samplingRate = 60; 
    FormatType format = FormatType::JSON; 
    std::string filepath = " "; 
};
/**
* @brief Commandobject containing commandType, metadataobject, websocket connection ptr
*/
struct Command {
    CmdType type; 
    StartMeta startMetaData; 
    StartMeta saveMetaData; 
    crow::websocket::connection* conn = nullptr; 
};

TSQueue<Command> cmdQueue; 
Command parseCommand(const std::string&, crow::websocket::connection&); 

/**
* @brief Parse a Datastring to a Commandobject 
* @param rawData data to be parsed 
* @param conn websocket connection handle for Commandobject 
*/
Command parseCommand(const std::string& rawData, crow::websocket::connection* conn){
    Command cmd{CmdType::UNKNOWN, {}, {}, conn}; 

    try{
        auto rawDataJson = nlohmann::json::parse(rawData); 
        std::string type = rawDataJson.at("type").get<std::string>(); 

        if(type=="start"){
            cmd.type = CmdType::START; 

            if(rawDataJson.contains("uuids")){
                cmd.startMetaData.uuids = rawDataJson["uuids"].get<std::vector<std::string>>(); 
            }
            if(rawDataJson.contains("rate")){
                cmd.startMetaData.samplingRate = std::max(10, std::min(100000, rawDataJson["rate"].get<int>()));
            }
            if(rawDataJson.contains("format")){
                std::string fmt = rawDataJson["format"].get<std::string>();
                if(fmt=="csv") cmd.startMetaData.format = FormatType::CSV; 
                else if (fmt=="binary") cmd.startMetaData.format = FormatType::BINARY; 
                else cmd.startMetaData.format = FormatType::JSON;  
            }
        }
        else if (type == "stop") {
            cmd.type = CmdType::STOP;
        }
        else if(type =="save"){
            cmd.type = CmdType::SAVE;  
            if(rawDataJson.contains("uuids")){
                cmd.saveMetaData.uuids = rawDataJson["uuids"].get<std::vector<std::string>>(); 
            }
            if(rawDataJson.contains("rate")){
                cmd.saveMetaData.samplingRate = std::max(10, std::min(100000, rawDataJson["rate"].get<int>()));
            }
            if(rawDataJson.contains("format")){
                std::string fmt = rawDataJson["format"].get<std::string>();
                if(fmt=="csv") cmd.saveMetaData.format = FormatType::CSV; 
                else if (fmt=="binary") cmd.saveMetaData.format = FormatType::BINARY; 
                else cmd.saveMetaData.format = FormatType::JSON;  
            }    
            if(rawDataJson.contains("path")){
                std::string path = rawDataJson["path"].get<std::string>(); 
                cmd.saveMetaData.filepath = path; 
            }   
        }
    }
    catch (...) {
        cmd.type = CmdType::UNKNOWN;
    }
    return cmd;

}

/** 
* @brief Starts new measurement with given presets 
* Sends data via WS connection 
*/
void startMeasurement(Command& cmd, ControlWriter& ctrl, WSContext& wsCtx){
    wsCtx.currentMeasurement = std::make_shared<Measurement>(); 
    wsCtx.currentMeasurement->dataDestination = DataDestination::WS; 
    wsCtx.currentMeasurement->format = cmd.startMetaData.format; 
    wsCtx.currentMeasurement->samplingRate = cmd.startMetaData.samplingRate; 
    wsCtx.currentMeasurement->uuids = cmd.startMetaData.uuids; 
    wsCtx.currentMeasurement->filePath = " "; 

    clearAllQueues();

    wsCtx.sendThread = std::jthread(processDeque, std::ref(*cmd.conn), wsCtx.currentMeasurement); 
    if(verbose) {
        cmd.conn->send_text("Sending data was started via the client.");
    }

    wsCtx.msmntThread = std::jthread(std::bind_front(&Measurement::start, wsCtx.currentMeasurement.get()), std::ref(ctrl));
    std::cout << "Measurement was set" << std::endl;

}

/**
* @brief Stops wsCtx threads, Writer; resetDevices and measurement
*/
void stopMeasurement(ControlWriter& ctrl, WSContext& wsCtx){
    websocketConnectionActive = false;
    if (wsCtx.sendThread.joinable()) {        
        wsCtx.sendThread.request_stop();      
        wsCtx.sendThread.join();             
    }
    if (wsCtx.msmntThread.joinable()) {        
        wsCtx.msmntThread.request_stop();              
        wsCtx.msmntThread.join();             
    }
    ctrl.stopWriter(); 
    resetDevices(); // do not switch order 
    startWriter = true;
    wsCtx.currentMeasurement = nullptr; 
}

void saveMeasurement(Command &cmd, ControlWriter& ctrl, WSContext& wsCtx){
    wsCtx.currentMeasurement = std::make_shared<Measurement>(); 
    wsCtx.currentMeasurement->dataDestination = DataDestination::LOCALFILE; 
    wsCtx.currentMeasurement->format = cmd.saveMetaData.format; 
    wsCtx.currentMeasurement->samplingRate = cmd.saveMetaData.samplingRate; 
    wsCtx.currentMeasurement->uuids = cmd.saveMetaData.uuids; 
    wsCtx.currentMeasurement->filePath = cmd.saveMetaData.filepath; 

    ctrl.saveData(wsCtx); 

    while(!saveDataQueue.empty()){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Process data from wsdataqueue from writer into messages and send messages via websocket to client
 */
// TODO: Make this better
void processDeque(std::stop_token stopToken, crow::websocket::connection &conn, std::shared_ptr<Measurement> measurement)
{
    while (!stopToken.stop_requested())
    {
        if (measurement->format == FormatType::JSON)
        {
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsPackagesQueue.empty())
            {
                nlohmann::ordered_json message;
                message.update(wsPackagesQueue.front());
                wsPackagesQueue.pop();
                message["type"].push_back("data"); 
                if (!measurement->uuids.empty())
                {
                    message["devices"] = nlohmann::json::array();
                    for (const auto &uuid : measurement->uuids)
                    {
                        message["devices"].push_back(uuid);
                    }
                }
                conn.send_text(message.dump());
            }
        }
        else if (measurement->format == FormatType::CSV)
        {
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsCSVPackagesQueue.empty())
            {
                std::string csvBatch = wsCSVPackagesQueue.front();
                wsCSVPackagesQueue.pop();
                conn.send_text(csvBatch);
            }
        }
        else if (measurement->format == FormatType::BINARY)
        {
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsBinaryPackagesQueue.empty())
            {
                std::string binaryBatch = wsBinaryPackagesQueue.front();
                wsBinaryPackagesQueue.pop();
                conn.send_binary(binaryBatch);
            }
        }
    }

    if (verbose)
    {
        std::cout << "Processdeque stopped" << std::endl;
    }
}

/**
* @brief CommandWorker process commands from the commandQueue 
* @param ControlWriter controls the Writer started for the measurement
* @param stop_token stops when stop_requested 
*/
void workOnCommands(std::stop_token stop_token, ControlWriter &controlWriter ){
     while(!stop_token.stop_requested()){
        Command cmd = cmdQueue.pop(); 

        switch (cmd.type){
            case CmdType::START: {
                websocketConnectionActive = true; 
                if(wsCtx.currentMeasurement){
                    cmd.conn->send_text(R"({"type":"error","msg":"already running"})");
                    break;
                }
                else startMeasurement(cmd, controlWriter, wsCtx);
                break;
            }
            case CmdType::STOP:{
                if (!wsCtx.currentMeasurement) {
                    cmd.conn->send_text(R"({"type":"error","msg":"not running"})");
                    break;
                }
                else stopMeasurement(controlWriter, wsCtx);
                break;  
            }
            case CmdType::SAVE:{
                if(wsCtx.currentMeasurement){
                    cmd.conn->send_text(R"({"type":"error","msg":"Stop measurement before saving"})");
                    break;
                }
                else saveMeasurement(cmd, controlWriter, wsCtx); 
                cmd.conn->send_text(nlohmann::json{ {"type","file-ready"},{"url","/download/" + cmd.saveMetaData.filepath}}.dump());
                break; 
            }
        }
     }
}

/**
 * @brief Start Crow WS on given port
 * Provides API endpoints for information and WS communication to start a measurement
 * @param port to start WS on
 * @param controlWriter to stop the Writer object 
 */
void StartWS(int &port, ControlWriter &controlWriter)
{
    std::mutex mtx;
    std::unordered_set<crow::websocket::connection *> users;
    std::unordered_map<crow::websocket::connection *, std::atomic<bool>> thread_control_flags;
    bool json_temp = false;
    bool ws_temp = true;
    std::string file_temp = " ";
    std::jthread cmdWorker;      

    auto &cors = crowApp.get_middleware<crow::CORSHandler>();

    cors
    .global()
    .origin("*")
    .headers("Origin", "Content-Type", "Accept", "X-Custom-Header", "Authorization")
    .methods("POST"_method, "GET"_method, "OPTIONS"_method)
    .max_age(600);

    WEBSOCKET_ACTIVE = true;

    /**
     * @brief endpoint to receive UUIDs and colors of connected devices
     */
    CROW_ROUTE(crowApp, "/UUID")
    ([]()
    {
        searchDevices();
        nlohmann::json devicesJson = getDevicesAsJson(false);
        return crow::response(devicesJson.dump());
    });

    /**
     * endpoint to receive further information from the backend
     */
    CROW_ROUTE(crowApp, "/v1/get_info")
    ([]()
    {
        searchDevices();
        nlohmann::json devicesJson = getDevicesAsJson(true);
        return crow::response(devicesJson.dump());
    });
    /**
     * @brief Endpoint to receive help for the websocket
     */
    CROW_ROUTE(crowApp, "/help")
    ([]()
    {
        return std::string("Starting the websocket under ip/ws. Set one or multiply UUIDs by writing them after the hello message. \n") + "The last input can be a sampling rate. The default sampling Rate is 60 Sa/s.\n" + "The sampling Rate cant be higher than 100.000 Sa/s. Press enter to start the measurement.";
    });

    CROW_ROUTE(crowApp, "/cors").methods("OPTIONS"_method)([]()
    {
        return crow::response(204);
    });

    CROW_ROUTE(crowApp, "/download/<string>")
    ([](const std::string& fname) {                    
        std::ifstream f(fname, std::ios::binary);
        if (fname.find_first_of("\\/") != std::string::npos) return crow::response(400);
        if (!f) return crow::response(404);

        std::string body{ std::istreambuf_iterator<char>(f),
                        std::istreambuf_iterator<char>() };

        crow::response res;
        res.code = 200;
        res.set_header("Content-Type", "text/plain");
        res.set_header("Content-Disposition",
                    "attachment; filename=\"" + fname + "\"");
        res.body = std::move(body);
        return res;                                   
    });

    /**
     * @brief websocket endpoint to receive measurement data from devices
     * While this is open the API endpoints cannot be used
     */
    // TODO : Endpoint usage while WS connection is open
    CROW_WEBSOCKET_ROUTE(crowApp, "/ws")
    .onopen([&](crow::websocket::connection &conn)
    {
        websocketConnectionActive = true;
        CROW_LOG_INFO << "new websocket connection from " << conn.get_remote_ip();
        std::lock_guard<std::mutex> _(mtx);
        users.insert(&conn);
        conn.send_text("Hello, connection to websocket established. To start a measurement send the wished UUID, optional send a sampling rate between 10 and 100000");
        cmdWorker = std::jthread(workOnCommands, std::ref(controlWriter));
    })
    .onclose([&](crow::websocket::connection &conn, const std::string &reason)
    {
        websocketConnectionActive = false;
        if (cmdWorker.joinable()) {
            cmdQueue.push(Command{CmdType::UNKNOWN,{},{},{nullptr}});
            cmdWorker.request_stop();
        }
        CROW_LOG_INFO << "websocket connection closed. Your measurement was stopped. " << reason;
        stopMeasurement(controlWriter, wsCtx);
        std::lock_guard<std::mutex> _(mtx);
        users.erase(&conn);
    })
    .onmessage([&](crow::websocket::connection &conn, const std::string &data, bool is_binary)
    {
        CROW_LOG_INFO << "Received message: " << data;
        websocketConnectionActive = true; 
        // TODO: Implement saving functionality as cmd 
        cmdQueue.push(parseCommand(data, &conn)); 
    }); 

    crowApp.signal_clear();
    std::signal(SIGINT, customSignalHandler);

    crowApp.port(port).multithreaded().run();
}

