//#define BOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT
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

//CLASSES/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Writer;
class Measurement;
enum class DataDestination;
enum class FormatType;

//GLOBAL VARIABLES//////////////////////////////////////////////////////////////////////////////////////////////////

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
std::mutex sampleQueueMutex;
std::mutex wsDataQueueMutex;
nlohmann::json HeaderJSON;
std::queue<nlohmann::json> wsPackagesQueue;
std::queue<std::string> wsCSVPackagesQueue;
std::queue<std::string> wsBinaryPackagesQueue;
std::thread wsDataQueueThread;
std::thread sendDataviaWSThread;
std::atomic<bool> WEBSOCKET_ACTIVE{false};
std::atomic<bool> wsDataQueueThreadActive{false};
std::atomic<bool> websocketConnectionActive{false};
crow::App<crow::CORSHandler> crowApp;
std::thread websocket;
bool sendDataviaWSThreadActive = false;
static std::atomic<int> dataPointsInSampleQue(0);
static int Datenanzahl(0);
static bool startWriter = true;

//FUNCTION HEADER/////////////////////////////////////////////////////////////////////////////////////////////////////

void initDevices();
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void selectDevices(std::vector<std::string> &UUID);
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& );
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t>& );
double round_to(double, int);
void sendDataStreamToWS(std::vector<std::string> &, std::string &, bool &, bool &);
void resetDevices();
void clearAllDeques();
void stopAndJoinWSConnectionThreads();
void stopAndJoinWSThread();
void ExitProgramm();
void CloseWSConnection();
std::string colorToString(const std::tuple<uint8_t, uint8_t, uint8_t>& rgb);
nlohmann::json getDevicesAsJson();
Measurement parseWSDataToMeasurement(const std::string& data);
void processDeque(crow::websocket::connection&, std::shared_ptr<Measurement>);
template<typename T, typename Container = std::deque<T>>
void clearQueue(std::queue<T, Container>& q) {
    std::queue<T, Container> empty;
    std::swap(q, empty);
};

//CLASSES/////////////////////////////////////////////////////////////////////////////////////////////////////////////

class DequeFormatter{ // Formatting data from captureData into a deque<sample_T> format
private:
    std::queue<sample_T>& handle;
    int samplingRate;

    void transformData(std::map<Omniscope::Id, std::vector<std::pair<double,double>>>& captureData, std::queue<sample_T>&handle, std::atomic<int>& dataPointsInSampleQue) {
        // transform Data into the sample format
        int currentPosition = 0;
        ts_T timeStamp = 0;
        val_T firstX = 0;
        std::optional<std::vector<val_T>> otherX;
        int sampleQuotient = 10000;

        if(samplingRate > 10 && samplingRate < 100000) {
            sampleQuotient = 100000/samplingRate;
        }
        if(verbose) {
            std::cout << "sampling quo: " << sampleQuotient << std::endl;
        }

        if (captureData.empty()) {
            return;
        }

        // Access to first device
        const auto& [firstId, firstDeviceData] = *captureData.begin();
        size_t vectorSize = firstDeviceData.size();

        for (currentPosition = 0; currentPosition < vectorSize; ++currentPosition) {
            if((currentPosition + sampleQuotient) < vectorSize) {
                currentPosition = currentPosition + sampleQuotient-1;
            }
            else return;

            if (!running) {
                return;
            }

            // values from first device
            timeStamp = round_to(firstDeviceData[currentPosition].first, 5);
            firstX = round_to(firstDeviceData[currentPosition].second, 5);

            // values from other devices
            otherX = std::vector<val_T>();
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it) {
                const auto& deviceData = it->second;
                if (currentPosition < deviceData.size()) {
                    otherX->push_back(round_to(deviceData[currentPosition].second,5));
                }
            }

            sample_T sample = std::make_tuple(timeStamp, firstX, otherX);

            //thread save acces to handle and counter
            std::lock_guard<std::mutex> lock(sampleQueueMutex);
            handle.push(sample);
            dataPointsInSampleQue++;
        }
    }

public:
    DequeFormatter(std::map<Omniscope::Id, std::vector<std::pair<double,double>>>& captureData, std::queue<sample_T>&handle, std::atomic<int>& dataPointsInSampleQue, int &samplingRate)
        : handle(handle), samplingRate(samplingRate) {
        transformData(captureData, handle, dataPointsInSampleQue);
    }
    ~DequeFormatter() {
    }
};

//MEASUREMENT///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class DataDestination {
    LOCALFILE,
    WS
};

enum class FormatType {
    CSV,
    JSON,
    BINARY,
    UNKNOWN
};

void printOrWriteData(std::shared_ptr<Measurement> measurement);

class Measurement:  public std::enable_shared_from_this<Measurement> {
public:
    std::vector<std::string> uuids;
    int samplingRate;
    FormatType format = FormatType::UNKNOWN;
    DataDestination dataDestination;
    std::string filePath; // optional

    Measurement(std::vector<std::string> uuids,std::string filePath, int samplingRate, FormatType fmt, DataDestination destination)
        :uuids(uuids), samplingRate(samplingRate), format(fmt), dataDestination(destination), filePath(filePath) {}
    Measurement() = default;
    ~Measurement() {}

    void start() {
        auto self = shared_from_this();

        if(self -> dataDestination == DataDestination::WS) {
            while(sendDataviaWSThreadActive) {
                searchDevices();   // Init Scopes

                selectDevices(self->uuids);  // select only chosen devices
                printOrWriteData(self); // print the data in the console or save it in the given filepath
            }
        }
        else {
            while(running) {
                searchDevices();   // Init Scopes

                selectDevices(self->uuids);  // select only chosen devices
                printOrWriteData(self); // print the data in the console or save it in the given filepath
            }
            resetDevices();
        }
    }
};


//WRITER//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Writer{ // write data into various formats, including a json object for the Websocket
private:
    std::ofstream outFile;
    std::queue<sample_T>& handle;
    std::queue<nlohmann::json>& jsonHandle;
    std::queue<std::string> & csvHandle;
    std::queue<std::string> & binaryHandle;
    std::thread writerThread;
    std::shared_ptr<Measurement> measurement;

    nlohmann::json createJsonObject(const std::vector<std::string>& UUIDs) {
        nlohmann::json jsonObject;

        // Metadata-Objekt erstellen
        nlohmann::json metadata;
        nlohmann::json devices = nlohmann::json::array();

        // Devices-Daten hinzufügen
        for (const auto& uuid : UUIDs) {
            nlohmann::json device;
            device["UUID"] = uuid;
            devices.push_back(device);
        }

        metadata["devices"] = devices;

        // Metadaten in das Hauptobjekt einfügen
        jsonObject["metadata"] = metadata;

        return jsonObject;
    }


    void write_csv(std::atomic<int> &dataPointsInSampleQue) {
        while(running) {
            if(dataPointsInSampleQue > 0) {
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                outFile << std::get<0>(sample) << " , " << std::get<1>(sample) << " ";
                const auto& optionalValues = std::get<2>(sample);
                if(optionalValues) {
                    for(size_t i = 0; i < optionalValues->size(); i++ ) {
                        outFile << (*optionalValues)[i];
                        if(i < optionalValues->size()-1) {
                            outFile << " , " ;
                        }
                    }
                }
                outFile << "\n";
                dataPointsInSampleQue --;
            }
        }
    }

    void write_json(std::atomic<int> &dataPointsInSampleQue) {

        outFile << "\"data\": " << "[";
        while(running) {
            if(dataPointsInSampleQue > 0) {
                int i = 0;
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                outFile << "{\"timestamp\" : " << std::get<0>(sample) << ","<< "\"value\": [" << std::get<1>(sample);
                if (i < measurement->uuids.size() -1) {
                    outFile << ",";
                    i++;
                }
                const auto& optionalValues = std::get<2>(sample);
                if(optionalValues) {
                    for(const auto& value : optionalValues.value()) {
                        outFile << value;
                        if (i < measurement->uuids.size()-1) {
                            outFile << ",";
                            i++;
                        }
                    }
                }
                outFile << "]" << "}" << ",";
                dataPointsInSampleQue --;
                i = 0;
            }
        }
        outFile << "]";
    }

    void write_console(std::atomic<int> &dataPointsInSampleQue) {
        while(running) {
            if(dataPointsInSampleQue > 0) {
                sample_T sample;
                std::lock_guard<std::mutex> lock(sampleQueueMutex);
                sample = handle.front();
                handle.pop();

                std::cout << "\r[";

                std::cout << std::get<0>(sample) << " , " << std::get<1>(sample) << " ";
                const auto& optionalValues = std::get<2>(sample);
                if(optionalValues) {
                    if(optionalValues) {
                        for(size_t i = 0; i < optionalValues->size(); i++ ) {
                            std::cout << (*optionalValues)[i];
                            if(i < optionalValues->size()-1) {
                                std::cout << " , " ;
                            }
                        }
                    }
                }

                std::cout << "]" << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                dataPointsInSampleQue --;
            }
        }
    }

    void write_JsonObject(std::atomic<int> &dataPointsInSampleQue, std::mutex& jsonMutex) {
        constexpr int batchSize = 1;
        nlohmann::json currentBatch;
        currentBatch["data"] = nlohmann::json::array();

        int batchCounter = 0;

        while (websocketConnectionActive) {
                if (dataPointsInSampleQue > 0) {
                    sample_T sample;

                    std::lock_guard<std::mutex> lock(sampleQueueMutex);
                    sample = handle.front();
                    handle.pop();

                    // add sample to Json object
                    nlohmann::json sampleObject;
                    sampleObject["timestamp"] = std::get<0>(sample);
                    sampleObject["value"] = nlohmann::json::array();
                    sampleObject["value"].push_back(std::get<1>(sample));

                    const auto& optionalValues = std::get<2>(sample);
                    if(optionalValues) {
                        for (size_t i = 0; i < optionalValues->size(); ++i) {
                            sampleObject["value"].push_back((*optionalValues)[i]);
                        }
                    }

                    // push in JSON queue:

                    currentBatch["data"].push_back(sampleObject);
                    batchCounter ++;
                    dataPointsInSampleQue --;

                    if(batchCounter >= batchSize) {
                        std::lock_guard<std::mutex> lock(jsonMutex);
                        jsonHandle.push(currentBatch);

                        currentBatch["data"] = nlohmann::json::array();
                        batchCounter = 0;
                    }
                    Datenanzahl++;
                }
        }
    }
    void write_CsvBatch(std::atomic<int>& dataPointsInSampleQue, std::mutex& jsonMutex)
    {
        std::string currentBatch;
        int batchSize = 1;
        int batchCounter = 0;

        while (websocketConnectionActive) {
                if (dataPointsInSampleQue > 0) {
                    sample_T sample;
                    std::lock_guard<std::mutex> lock(sampleQueueMutex);
                    sample = handle.front();
                    handle.pop();

                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(3);

                    oss << std::get<0>(sample) << ", " << std::get<1>(sample);

                    const auto& optionalValues = std::get<2>(sample);
                    if (optionalValues.has_value()) {
                        for (const auto& val : optionalValues.value()) {
                            oss << ", " << val;
                        }
                    }

                    oss << "\n";
                    std::string sampleLine = oss.str();
                    currentBatch += sampleLine;
                    ++batchCounter;
                    --dataPointsInSampleQue;

                    if (batchCounter >= batchSize) {
                        std::lock_guard<std::mutex> lock(jsonMutex);
                        csvHandle.push(currentBatch);
                        currentBatch.clear();
                        batchCounter = 0;
                    }
                    ++Datenanzahl;
                }
        }

        if (!currentBatch.empty()) {
            std::lock_guard<std::mutex> lock(jsonMutex);
            csvHandle.push(currentBatch);
        }
    }

    void write_ProtobufSamples(std::atomic<int>& dataPointsInSampleQue, std::mutex& jsonMutex) {
        while (websocketConnectionActive) {
                if (dataPointsInSampleQue > 0) {
                    sample_T sample;
                    {
                        std::lock_guard<std::mutex> lock(sampleQueueMutex);
                        sample = handle.front();
                        handle.pop();
                    }

                    Sample sampleMsg;
                    sampleMsg.set_timestamp(std::get<0>(sample));

                    sampleMsg.add_values(std::get<1>(sample));

                    const auto& optValues = std::get<2>(sample);
                    if (optValues) {
                        for (const auto& val : *optValues) {
                            sampleMsg.add_values(val);
                        }
                    }

                    std::string serializedSample;
                    if (!sampleMsg.SerializeToString(&serializedSample)) {
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


    void write(std::atomic<int> &dataPointsInSampleQue, std::mutex& jsonMutex) {

        if(verbose) {
            std::cout << "Writer startet" << std::endl;
        }
        if(measurement->dataDestination == DataDestination::WS) {
            if(measurement->format == FormatType::JSON) {
                write_JsonObject(dataPointsInSampleQue, jsonMutex);
            }
            else if (measurement->format == FormatType::CSV) {
                write_CsvBatch(dataPointsInSampleQue, jsonMutex);
            }
            else if(measurement->format == FormatType::BINARY) {
                write_ProtobufSamples(dataPointsInSampleQue, jsonMutex);
            }
        }
        else if (measurement->dataDestination == DataDestination::LOCALFILE) {
            if(!measurement->filePath.empty()) {
                if(measurement->format == FormatType::CSV) {
                    write_csv(dataPointsInSampleQue);
                }
                else if(measurement->format == FormatType::JSON) {
                    write_json(dataPointsInSampleQue);
                }
            }
            else {
                write_console(dataPointsInSampleQue);
            }
        }

        if(verbose) {
            std::cout << "Schreiben beendet" << std::endl;
        }
    }

public:
    Writer(std::shared_ptr<Measurement> measurement, std::queue<sample_T>& handle, std::atomic<int>& dataPointsInSampleQue, std::queue<nlohmann::json>& jsonHandle, std::mutex &jsonMutex, std::queue<std::string> &csvHandle, std::queue<std::string> &binaryHandle)
        :handle(handle), measurement(measurement), jsonHandle(jsonHandle), csvHandle(csvHandle), binaryHandle(binaryHandle) {
        std::cout << std::fixed << std::setprecision(3);

        // Seperation by data destination
        if(measurement->dataDestination == DataDestination::WS) {
            if(measurement->format == FormatType::JSON) {
                HeaderJSON = createJsonObject(measurement->uuids);
            }
        }
        else if(measurement->dataDestination == DataDestination::LOCALFILE) {
            if(!measurement->filePath.empty()) {
                outFile.open(measurement->filePath, std::ios::app);
                if (!outFile) {
                    throw std::ios_base::failure("Error opening file: " + measurement->filePath);
                }// Write header
                else if(measurement->format == FormatType::CSV) {
                    outFile << "Timestamp [s]" << " , ";
                    for(size_t i = 0; i < measurement->uuids.size(); ++i) {
                        outFile << measurement->uuids[i];
                        if(i < measurement->uuids.size()-1) {
                            outFile << " , ";
                        }
                    }
                    outFile << "\n";
                }
                else if(measurement->format == FormatType::JSON) {
                    outFile << "{\"metadata\": {";
                    outFile << "\"" << "devices" << "\"" << ":" << "[" << "{";
                    for (size_t i = 0; i < measurement->uuids.size(); ++i) {
                        outFile << "\"" << "UUID" << "\"" << ": " << "\"" << measurement->uuids[i] << "\"";
                        if (i < measurement->uuids.size() - 1) {
                            outFile << ",";
                        }
                    }
                    outFile << "}" << "]";  // optional metadata: measurement, host
                    outFile << "},";
                }
            }
            else {
                std::cout << "Timestamp [s]" << " , ";
                for (size_t i = 0; i < measurement->uuids.size(); ++i) {
                    std::cout << measurement->uuids[i] << " [V]";
                    if (i < measurement->uuids.size() - 1) {
                        std::cout << " , ";
                    }
                }
                std::cout << "\n";
            }
        }

        // Starte den Thread erst nach dem Öffnen der Datei
        writerThread = std::thread(&Writer::write, this, std::ref(dataPointsInSampleQue), std::ref(jsonMutex));
    }

    ~Writer() {
        if(!measurement->filePath.empty()) {
            if(measurement->format == FormatType::JSON) {
                outFile << "}";
            }
            // write Trailer
            outFile.flush();
            outFile.close();
        }
        else std::cout << "Schreiben beendet" << std::endl;

        if(verbose) {
            std::cout << "Writer destroyed" << std::endl;
        }
        if(writerThread.joinable()) {
            writerThread.join();
        }
    }

};

//FUNCTIONS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Closing the Programm and WS Connections savely:

void customSignalHandler(int signal) {
    if(signal == SIGINT) {
        std::cout << "\ngot SIGINT. Stopping the programm..." << std::endl;
        running = false;
    }
}

void ExitProgramm() {
    if(WEBSOCKET_ACTIVE) {
        crowApp.stop();
    }
    resetDevices();
    stopAndJoinWSConnectionThreads();
    stopAndJoinWSThread();
    std::cout << "Programm was closed correctly, all threads closed" << std::endl;
}

void CloseWSConnection() {
    // std::cout << "CloseWSConnectionStarted" << std::endl;
    stopAndJoinWSConnectionThreads();
    resetDevices();
    startWriter = true;
}

void resetDevices() {
    if(sampler.has_value()) {
        for (auto &device : sampler->sampleDevices) {
            device.first->send(Omniscope::Stop{});
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    sampler.reset();
    devices.clear();
    deviceManager.clearDevices();
    //captureData.clear();
    dataPointsInSampleQue = 0;
    clearAllDeques();
}

void clearAllDeques() {

    if(!sampleQueue.empty()) {
        std::lock_guard<std::mutex> lock(sampleQueueMutex);
        clearQueue(sampleQueue);
    }
    if(!wsPackagesQueue.empty()) {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsPackagesQueue);
    }
    if(!wsBinaryPackagesQueue.empty()) {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsBinaryPackagesQueue);
    }
    if(!wsCSVPackagesQueue.empty()) {
        std::lock_guard<std::mutex> lock(wsDataQueueMutex);
        clearQueue(wsCSVPackagesQueue);
    }
}

void stopAndJoinWSConnectionThreads() {
    if(wsDataQueueThreadActive) {
        wsDataQueueThreadActive = false;
        if(wsDataQueueThread.joinable()) {
            wsDataQueueThread.join();
            if(verbose) {
                std::cout << "dataThread joined" << std::endl;
            }
        }
    }
    if(sendDataviaWSThreadActive) {
        sendDataviaWSThreadActive = false;
        if(sendDataviaWSThread.joinable()) {
            sendDataviaWSThread.join();
            if(verbose) {
                std::cout << "sendDataThread joined" << std::endl;
            }
        }
    }
}

void stopAndJoinWSThread() {
    if(WEBSOCKET_ACTIVE) {
        WEBSOCKET_ACTIVE = false;
        if(websocket.joinable()) {
            websocket.join();
            if(verbose) {
                std::cout << "Websocket thread was joined" << std::endl;
            }
        }
    }
}

//CONNECT_DEVICES//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void initDevices() { // Initalize the connected devices
    constexpr int VID = 0x2e8au;
    constexpr int PID = 0x000au;
    if(verbose) {
        std::cout << "Geraete werden gesucht" << std::endl;
    }

    devices = deviceManager.getDevices(VID, PID);
    for(auto& device : devices) {
        auto metaDataCb = [&](auto const &msg) {
            if (std::holds_alternative<Omniscope::MetaData>(msg)) {
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

void searchDevices() {
    if(!sampler.has_value()) {
        devices.clear();
        deviceManager.clearDevices();
        initDevices(); // check if devices are connected
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Pause between checks
    }
}

void selectDevices(std::vector<std::string> &UUID) {
    if(!devices.empty() && !sampler.has_value()) { // move the device in the sampler
        std::cout <<"Devices where found and are emplaced"<< std::endl;
        for(const auto &device : devices) {
            devices.erase(
                std::remove_if(
                    devices.begin(),
                    devices.end(),
            [&UUID](const std::shared_ptr<OmniscopeDevice>& device) {
                return std::find(UUID.begin(), UUID.end(), device->getId()->serial) == UUID.end();
            }),
            devices.end()
            );
        }
        if(!devices.empty()) {
            sampler.emplace(deviceManager, std::move(devices));
            if(sampler.has_value()) {
                for (auto &device : sampler->sampleDevices) {
                    device.first->send(Omniscope::Start{});
                }
            }
        }
    }
}

void parseDeviceMetaData(Omniscope::MetaData metaData,
                         std::shared_ptr<OmniscopeDevice> &device) {
    try {
        nlohmann::json metaJson = nlohmann::json::parse(metaData.data);
        if(verbose) {
            fmt::println("{}", metaJson.dump());
        }
        device->setScale(std::stod(metaJson["scale"].dump()));
        device->setOffset(std::stod(metaJson["offset"].dump()));
        device->setEgu(metaJson["egu"]);
    } catch (...) {
        if(verbose) {
            fmt::print("This Scope is not calibrated: {}", metaData.data);
        }
    }
}

//DATAHANDELING///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printOrWriteData(std::shared_ptr<Measurement> measurement) {

    if (sampler.has_value()) {
        captureData.clear();
        int vectorSize = 0;

        while (vectorSize < 100000) {
            if (sampler.has_value()) {
                sampler->copyOut(captureData);
            }
            if (captureData.empty()) {
                return;
            }

            auto it = captureData.begin();
            auto& [updatedId, updatedDeviceData] = *it;
            vectorSize = static_cast<int>(updatedDeviceData.size());
        }

        DequeFormatter* dequeFormatter = new DequeFormatter(captureData, sampleQueue, dataPointsInSampleQue, measurement->samplingRate);
        delete dequeFormatter;

        if (startWriter) {
            Writer* writi = new Writer(measurement, sampleQueue, dataPointsInSampleQue, wsPackagesQueue, wsDataQueueMutex, wsCSVPackagesQueue, wsBinaryPackagesQueue);
            startWriter = false;
        }
    }
}

void printDevices() {

    // get IDs
    if(devices.empty()) {
        std::cout << "No devices are connected. Please connect a device and start again" << std::endl;
    }
    else {
        std::cout << "The following devices are connected:" << std::endl;
        for(const auto& device : devices) {
            std::string deviceId = device -> getId()->serial;
            fmt::print("{}Device: {}\033[0m\n", rgbToAnsi(uuidToColor(deviceId)), deviceId);
        }
        devices.clear();
        deviceManager.clearDevices();
        running = false;
    }
}

nlohmann::json getDevicesAsJson() {
    nlohmann::json responseJson;

    if (devices.empty()) {
        // Kein Gerät verbunden
        responseJson["error"] = "No devices are connected. Please connect a device and start again.";
    } else {
        // JSON-Arrays für Geräte und Farben erstellen
        nlohmann::json devicesArray = nlohmann::json::array();
        nlohmann::json colorsArray = nlohmann::json::array();

        for (const auto& device : devices) {
            // Gerätedaten extrahieren
            std::string deviceId = device->getId()->serial;
            auto color = uuidToColor(deviceId);

            // Gerät zur JSON-Liste hinzufügen
            devicesArray.push_back({{"UUID", deviceId}});

            // Farbe zur JSON-Liste hinzufügen
            nlohmann::json colorJson;
            colorJson["color"]["r"] = std::get<0>(color);
            colorJson["color"]["g"] = std::get<1>(color);
            colorJson["color"]["b"] = std::get<2>(color);

            colorsArray.push_back(colorJson);
        }

        // JSON-Daten zusammenstellen
        responseJson["devices"] = devicesArray;
        responseJson["colors"] = colorsArray;

        // Geräte und Manager zurücksetzen
        devices.clear();
        deviceManager.clearDevices();
    }

    return responseJson;
}

std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& uuid) {
    // std::hash anwenden
    size_t hashValue = std::hash<std::string> {}(uuid);

    // Die ersten 3 Bytes extrahieren
    uint8_t r = (hashValue & 0xFF);         // Erste 8 Bits
    uint8_t g = (hashValue >> 8) & 0xFF;   // Zweite 8 Bits
    uint8_t b = (hashValue >> 16) & 0xFF;  // Dritte 8 Bits

    return {r, g, b};
}

std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t>& rgb) {
    auto [r, g, b] = rgb; // Tupel entpacken
    return fmt::format("\033[38;2;{};{};{}m", r, g, b); // ANSI-Farbcode generieren
}

std::string colorToString(const std::tuple<uint8_t, uint8_t, uint8_t>& rgb) {
    auto [r, g, b] = rgb; // Tupel entpacken
    return fmt::format("{};{};{}", r, g, b); // ANSI-Farbcode generieren
}

double round_to(double value, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
}

/// WEBSOCKET HANDLING ///////////////////////////////////////////////////////////////////////////////////////////////////

void StartWS(int &port) {
    std::mutex mtx;
    std::unordered_set<crow::websocket::connection*> users;
    std::unordered_map<crow::websocket::connection*, std::atomic<bool>> thread_control_flags;
    bool json_temp = false;
    bool ws_temp = true;
    std::string file_temp = " ";

    auto& cors = crowApp.get_middleware<crow::CORSHandler>();

    cors
    .global()
    .origin("*") // Erlaube Anfragen von allen Domains (Browser-freundlich)
    .headers("Origin", "Content-Type", "Accept", "X-Custom-Header", "Authorization") // Alle relevanten Header
    .methods("POST"_method, "GET"_method, "OPTIONS"_method) // Erlaube POST, GET und OPTIONS (für Preflight)
    .max_age(600); // Cache Preflight-Antworten für 10 Minuten
    // clang-format on

    WEBSOCKET_ACTIVE = true;

    // API
    CROW_ROUTE(crowApp, "/UUID") // HTTP GET auf "/hello"
    ([]() {
        searchDevices();
        nlohmann::json devicesJson = getDevicesAsJson();
        return crow::response(devicesJson.dump());
    });

    CROW_ROUTE(crowApp, "/help") // HTTP GET auf "/hello"
    ([]() {
        return std::string("Starting the websocket under ip/ws. Set one or multiply UUIDs by writing them after the hello message. \n")
               + "The last input can be a sampling rate. The default sampling Rate is 60 Sa/s.\n"
               + "The sampling Rate cant be higher than 100.000 Sa/s. Press enter to start the measurement.";
    });

    // OPTIONS-Endpunkt, um Preflight-Anfragen zu behandeln
    CROW_ROUTE(crowApp, "/cors").methods("OPTIONS"_method)([]() {
        return crow::response(204); // Antwort ohne Inhalt
    });

    // Websocket
    CROW_WEBSOCKET_ROUTE(crowApp, "/ws")
    .onopen([&](crow::websocket::connection& conn) {
        websocketConnectionActive = true;
        CROW_LOG_INFO << "new websocket connection from " << conn.get_remote_ip();
        std::lock_guard<std::mutex> _(mtx);
        users.insert(&conn);
        conn.send_text("Hello, connection to websocket established. To start a measurement send the wished UUID, optional send a sampling rate between 10 and 100000");
    })
    .onclose([&](crow::websocket::connection& conn, const std::string& reason) {
        websocketConnectionActive = false;
        CROW_LOG_INFO << "websocket connection closed. Your measurement was stopped. " << reason;
        CloseWSConnection();
        std::lock_guard<std::mutex> _(mtx);
        users.erase(&conn);
    })
    .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
        CROW_LOG_INFO << "Received message: " << data;
        std::lock_guard<std::mutex> _(mtx);
        auto measurement = std::make_shared<Measurement>(parseWSDataToMeasurement(data));
        if(!measurement->uuids.empty()) {
            clearAllDeques();

            wsDataQueueThread = std::thread(processDeque, std::ref(conn), measurement);
            wsDataQueueThreadActive = true;
            if(verbose) {
                conn.send_text("Sending data was started via the client.");
            }

            sendDataviaWSThread = std::thread(&Measurement::start, measurement);
            sendDataviaWSThreadActive = true;
            std::cout << "Measurement was set" << std::endl;
        }
    });

    crowApp.signal_clear();
    std::signal(SIGINT, customSignalHandler);

    crowApp.port(port).multithreaded().run();
}

Measurement parseWSDataToMeasurement(const std::string& data) {
    std::istringstream iss(data);
    std::string token;
    Measurement measurement;
    measurement.dataDestination = DataDestination::WS;
    measurement.format = FormatType::JSON;

    bool foundSamplingRate = false;

    while (iss >> token) {
        // Check if token is uuid
        if (!token.empty() && token[0] == 'E') {
            measurement.uuids.push_back(token);
        }
        else if (std::all_of(token.begin(), token.end(), ::isdigit)) { //check if token is number
            measurement.samplingRate = std::stoi(token);
            foundSamplingRate = true;
        }
        else if (token == "csv") { // check if token is a format
            measurement.format = FormatType::CSV;
        }
        else if (token == "json") {
            measurement.format = FormatType::JSON;
        }
        else if (token == "binary") {
            measurement.format = FormatType::BINARY;
        }
    }
    measurement.filePath = " ";
    return measurement;
}

void processDeque(crow::websocket::connection& conn, std::shared_ptr<Measurement> measurement) {
    while (running && websocketConnectionActive) {
        if (measurement->format == FormatType::JSON) {
            std::cout << "Format ist json" << std::endl;
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsPackagesQueue.empty()) {
                nlohmann::ordered_json message;
                message.update(wsPackagesQueue.front());
                wsPackagesQueue.pop();

                if (!measurement->uuids.empty()) {
                    message["devices"] = nlohmann::json::array();
                    for (const auto& uuid : measurement->uuids) {
                        message["devices"].push_back(uuid);
                    }
                }
                conn.send_text(message.dump());
            }
        }
        else if (measurement->format == FormatType::CSV) {
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsCSVPackagesQueue.empty()) {
                std::string csvBatch = wsCSVPackagesQueue.front();
                wsCSVPackagesQueue.pop();
                conn.send_text(csvBatch);
            }
        }
        else if (measurement->format == FormatType::BINARY) {
            std::lock_guard<std::mutex> lock(wsDataQueueMutex);
            if (!wsBinaryPackagesQueue.empty()) {
                std::string binaryBatch = wsBinaryPackagesQueue.front();
                wsBinaryPackagesQueue.pop();
                conn.send_binary(binaryBatch);
            }
        }
    }

    if (verbose) {
        std::cout << "Processdeque stopped" << std::endl;
    }
}

