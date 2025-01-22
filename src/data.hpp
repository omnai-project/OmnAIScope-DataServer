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
#include <deque>
#include <cmath>
#include <unordered_set>
#include <mutex>
#include "crow.h"

// Declaration

using val_T = double;
using ts_T = double; // timestamp

using sample_T = std::tuple<ts_T, val_T, std::optional<std::vector<val_T>>>;

inline OmniscopeDeviceManager deviceManager{};
inline std::vector<std::shared_ptr<OmniscopeDevice>> devices;
inline std::optional<OmniscopeSampler> sampler{};
inline std::map<Omniscope::Id, std::vector<std::pair<double, double>>> captureData;
std::atomic<bool> running{true};
bool verbose{false};
std::deque<sample_T> dataDeque;
std::mutex handleMutex;
std::mutex jsonMutex; 
nlohmann::json HeaderJSON; 
std::deque<nlohmann::json> packageDeque; 
std::atomic<bool> websocketActive{false}; 


void waitForExit();
void initDevices();
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void startMeasurementAndWrite(std::vector<std::string> &, std::string &, bool &, bool &);
void selectDevices();
void printOrWriteData(std::string &, std::vector<std::string> &, bool &, bool & );
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& );
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t>& );
double round_to(double , int);
void processDeque(crow::websocket::connection&); 



// Initialization

class Transformater{
private:
    std::deque<sample_T>& handle;

    void transformData(std::map<Omniscope::Id, std::vector<std::pair<double,double>>>& captureData, std::deque<sample_T>&handle,std::vector<std::string>& UUID, std::atomic<int>& counter) {
        // transform Data into the sample format
        int currentPosition = 0;
        ts_T timeStamp = 0;
        val_T firstX = 0;
        std::optional<std::vector<val_T>> otherX;

        if (captureData.empty()) {
            return;
        }

        // Access to first device
        const auto& [firstId, firstDeviceData] = *captureData.begin();
        size_t vectorSize = firstDeviceData.size();

        for (currentPosition = 0; currentPosition < vectorSize; ++currentPosition) {
            if (!running) {
                return;
            }

            // values from first device
            timeStamp = round_to(firstDeviceData[currentPosition].first, 3);
            firstX = round_to(firstDeviceData[currentPosition].second, 3);

            // values from other devices
            otherX = std::vector<val_T>();
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it) {
                const auto& deviceData = it->second;
                if (currentPosition < deviceData.size()) {
                    otherX->push_back(round_to(deviceData[currentPosition].second,3));
                }
            }

            sample_T sample = std::make_tuple(timeStamp, firstX, otherX);

            //thread save acces to handle and counter
            std::lock_guard<std::mutex> lock(handleMutex);
            handle.push_back(sample);
            counter++;


            if (verbose) {
                std::cout << "Sample " << counter << " geschrieben." << std::endl;
            }
        }
    }

public:
    Transformater(std::map<Omniscope::Id, std::vector<std::pair<double,double>>>& captureData, std::deque<sample_T>&handle,std::vector<std::string> &UUID, std::atomic<int>& counter)
        : handle(handle) {
        if(verbose) {
            std::cout << "Transformation wird gestartet" << std::endl;
        }

        transformData(captureData, handle, UUID, counter);

        if(verbose) {
            std::cout << "Daten erfolgreich in Deque" << std::endl;
        }
    }
    ~Transformater() {
        if(verbose) {
            std::cout << "Transformater deleted" << std::endl;
        }
    }
};

class Writer{
private:
    std::string format;
    std::string filePath;
    std::ofstream outFile;
    std::deque<sample_T>& handle;
    std::deque<nlohmann::json>& jsonHandle; 
    std::thread writerThread;
    std::vector<std::string> UUID;
    bool WS; 

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


    void write_csv(std::atomic<int> &counter) {
        while(running) {
            if(counter > 0) {
                sample_T sample;
                std::lock_guard<std::mutex> lock(handleMutex);
                sample = handle.front();
                handle.pop_front();

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
                counter --;
            }
        }
    }

    void write_json(std::atomic<int> &counter) {

        outFile << "\"data\": " << "[";
        while(running) {
            if(counter > 0) {
                int i = 0;
                sample_T sample;
                std::lock_guard<std::mutex> lock(handleMutex);
                sample = handle.front();
                handle.pop_front();

                outFile << "{\"timestamp\" : " << std::get<0>(sample) << ","<< "\"value\": [" << std::get<1>(sample);
                if (i < UUID.size() -1) {
                    outFile << ",";
                    i++;
                }
                const auto& optionalValues = std::get<2>(sample);
                if(optionalValues) {
                    for(const auto& value : optionalValues.value()) {
                        outFile << value;
                        if (i < UUID.size()-1) {
                            outFile << ",";
                            i++;
                        }
                    }
                }
                outFile << "]" << "}" << ",";
                counter --;
                i = 0;
            }
        }
        outFile << "]";
    }

    void write_console(std::atomic<int> &counter) {
        while(running) {
            if(counter > 0) {
                sample_T sample;
                std::lock_guard<std::mutex> lock(handleMutex);
                sample = handle.front();
                handle.pop_front();

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
                counter --;
            }
        }
    }

    void write_JsonObject(std::atomic<int> &counter, std::mutex& jsonMutex){
        constexpr int batchSize = 10; 
        nlohmann::json currentBatch; 
        currentBatch["data"] = nlohmann::json::array(); 

        int batchCounter = 0; 

        while(running){
            if (counter > 0) {
                sample_T sample; 

                std::lock_guard<std::mutex> lock(handleMutex); 
                sample = handle.front(); 
                handle.pop_front(); 

                // add sample to Json object 
                nlohmann::json sampleObject; 
                sampleObject["timestamp"] = std::get<0>(sample); 
                sampleObject["value"] = std::get<1>(sample); 

                const auto& optionalValues = std::get<2>(sample); 
                if(optionalValues){
                    for(const auto& value : optionalValues.value()){
                        sampleObject["value"].push_back(value); 
                    }
                }

                // push in JSON deque: 

                currentBatch["data"].push_back(sampleObject); 
                batchCounter ++; 
                counter --; 

                if(batchCounter >= batchSize){
                    std::lock_guard<std::mutex> lock(jsonMutex); 
                    jsonHandle.push_back(currentBatch); 

                    currentBatch["data"] = nlohmann::json::array(); 
                    batchCounter = 0; 
                }
            }
        }
    }

    void write(std::atomic<int> &counter, std::mutex& jsonMutex) {

        if(verbose) {
            std::cout << "Writer startet" << std::endl;
        }
        if(WS){
            write_JsonObject(counter, jsonMutex); 
        }
        else {
            // Write into a file
            if(!filePath.empty()) {
                // choose format
                if(format == "csv") {
                    write_csv(counter);
                }
                else if(format == "json") {
                    write_json(counter);
                }
            }
            else {
                write_console(counter);
            }
        }

        if(verbose) {
            std::cout << "Schreiben beendet" << std::endl;
        }
    }

public:
    Writer(std::string& format, std::string &filePath, std::vector<std::string> &UUID, std::deque<sample_T>& handle, std::atomic<int>& counter, bool &WS, std::deque<nlohmann::json>& jsonHandle, std::mutex &jsonMutex)
        :handle(handle), format(format), UUID(UUID), filePath(filePath), WS(WS), jsonHandle(jsonHandle){
        std::cout << std::fixed << std::setprecision(3);
        if(WS){
            HeaderJSON = createJsonObject(UUID); 
        }
        else{
            if(!filePath.empty()) {
                outFile.open(filePath, std::ios::app);
                if (!outFile) {
                    throw std::ios_base::failure("Error opening file: " + filePath);
                }// Write header
                else if(format == "csv") {
                    outFile << "Timestamp [s]" << " , ";
                    for(size_t i = 0; i < UUID.size(); ++i) {
                        outFile << UUID[i];
                        if(i < UUID.size()-1) {
                            outFile << " , ";
                        }
                    }
                    outFile << "\n";
                }
                else if(format == "json") {
                    outFile << "{\"metadata\": {";
                    outFile << "\"" << "devices" << "\"" << ":" << "[" << "{";
                    for (size_t i = 0; i < UUID.size(); ++i) {
                        outFile << "\"" << "UUID" << "\"" << ": " << "\"" << UUID[i] << "\"";
                        if (i < UUID.size() - 1) {
                            outFile << ",";
                        }
                    }
                    outFile << "}" << "]";  // optional metadata: measurement, host
                    outFile << "},";
                }
            }
            else {
                std::cout << "Timestamp [s]" << " , ";
                for (size_t i = 0; i < UUID.size(); ++i) {
                    std::cout << UUID[i] << " [V]";
                    if (i < UUID.size() - 1) {
                        std::cout << " , ";
                    }
                }
                std::cout << "\n";
            }
        }

        // Starte den Thread erst nach dem Öffnen der Datei
        writerThread = std::thread(&Writer::write, this, std::ref(counter), std::ref(jsonMutex));
    }

    ~Writer() {
        if(!filePath.empty()) {
            if(format == "json") {
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

void waitForExit() { // wait until the user closes the programm by pressing ENTER
    std::cout << "OmnAIView is starting. Press enter to stop the programm." << std::endl;
    std::cin.sync();
    std::cin.get();
    running = false;
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

void printOrWriteData(std::string &filePath, std::vector<std::string> &UUID, bool &isJson, bool &WS) {
    static bool startWriter = true;
    std::string format;
    if(isJson) {
        format = "json";
    }
    else format = "csv";

    static std::atomic<int> counter(0);

    if(sampler.has_value()) { // write Data into file
        captureData.clear();
        sampler->copyOut(captureData);

        Transformater* transformi = new Transformater(captureData, dataDeque, UUID, counter); // transform data into sample format
        delete transformi;

        if(startWriter) {
            Writer* writi = new Writer(format, filePath, UUID, dataDeque, counter, WS, packageDeque, jsonMutex); // write data into a file or the console
            startWriter = false;
        }
    }
}

void startMeasurementAndWrite(std::vector<std::string> &UUID, std::string &filePath, bool &isJson, bool &WS) {
    while(running) {
        searchDevices();   // Init Scopes

        selectDevices(UUID);  // select only chosen devices

        printOrWriteData(filePath, UUID, isJson, WS); // print the data in the console or save it in the given filepath
    }
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

double round_to(double value, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(value * factor) / factor;
}

void WSTest(){
    crow::SimpleApp app;

    std::mutex mtx;
    std::unordered_set<crow::websocket::connection*> users;
    std::unordered_map<crow::websocket::connection*, std::atomic<bool>> thread_control_flags;
    std::thread dequeThread; 


    CROW_WEBSOCKET_ROUTE(app, "/ws")
      .onopen([&](crow::websocket::connection& conn) {
          websocketActive = true; 
          CROW_LOG_INFO << "new websocket connection from " << conn.get_remote_ip();
          std::lock_guard<std::mutex> _(mtx);
          users.insert(&conn);
          conn.send_text("Hello, connection established");      
      })
        .onclose([&](crow::websocket::connection& conn, const std::string& reason) {
            websocketActive = false;
            while(!dequeThread.joinable()){
                std::cout << "thread not joinable" << std::endl; 
            }
            dequeThread.join(); 
           CROW_LOG_INFO << "websocket connection closed: " << reason;

          std::lock_guard<std::mutex> _(mtx);
          users.erase(&conn);
      })
     .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
        CROW_LOG_INFO << "Received message: " << data;
        std::lock_guard<std::mutex> _(mtx);
            if(!HeaderJSON.empty()){
                conn.send_text(HeaderJSON.dump()); 
                HeaderJSON.clear(); 
            }            

            // Überprüfe den Inhalt der Nachricht
            if (data == "start") {
                conn.send_text("pong"); // test pong 

                // deque processing in extra thread 
                dequeThread = std::thread(processDeque, std::ref(conn));
                if(verbose){
                conn.send_text("Thread was started");
                }
            }
    });


    app.port(8080).multithreaded().run();
}

void processDeque(crow::websocket::connection& conn) {
    while (websocketActive) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Polling-Intervall

        std::lock_guard<std::mutex> lock(jsonMutex);
        if (!packageDeque.empty()) {
            // Hole das erste Element aus der deque
            nlohmann::json firstElement = packageDeque.front();
            packageDeque.pop_front();

            // Sende das JSON über den WebSocket
            conn.send_text(firstElement.dump());
        }
    }

    std::cout << "Background thread stopped." << std::endl;
}

