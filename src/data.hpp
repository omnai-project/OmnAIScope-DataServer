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

void waitForExit();
void initDevices();
void writeDatatoFile(std::map<Omniscope::Id, std::vector<std::pair<double, double>>>&, std::string &, std::vector<std::string> &, bool &);
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void startMeasurementAndWrite(std::vector<std::string> &, std::string &, bool &);
void selectDevices();
void printOrWrite(std::string &, std::vector<std::string> &, bool &);
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& );
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t>& );

// Initialization

class Transformater{
private:
    std::deque<sample_T>& handle;

    void transformData(std::map<Omniscope::Id, std::vector<std::pair<double,double>>>& captureData, std::deque<sample_T>&handle,std::vector<std::string>& UUID, std::atomic<int>& counter) {

        int currentPosition = 0;
        bool startSample{true};
        ts_T timeStamp = 0;
        val_T firstX = 0;
        std::optional<std::vector<val_T>> otherX;

        if (captureData.empty()) {
            return;
        }

// Zugriff auf das erste Gerät
        const auto& [firstId, firstDeviceData] = *captureData.begin();
        size_t vectorSize = firstDeviceData.size();

        for (currentPosition = 0; currentPosition < vectorSize; ++currentPosition) {
            if (!running) {
                return;
            }

            // Werte aus dem ersten Gerät
            timeStamp = firstDeviceData[currentPosition].first;
            firstX = firstDeviceData[currentPosition].second;

            // Werte aus den anderen Geräten
            otherX = std::vector<val_T>();
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it) {
                const auto& deviceData = it->second;
                if (currentPosition < deviceData.size()) {
                    otherX->push_back(deviceData[currentPosition].second);
                }
            }

            sample_T sample = std::make_tuple(timeStamp, firstX, otherX);
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
    ~Transformater() {}
};

class Writer{
private:
    std::string format;
    std::ofstream outFile;
    std::deque<sample_T>& handle;
    std::thread writerThread;

    void write(std::string &filePath, std::atomic<int> &counter) {

        if(verbose) {
            std::cout << "Writer startet" << std::endl;
        }
        if (!outFile) {
            std::cout << "File nicht korrekt geöffnet" << std::endl;
            throw std::ios_base::failure("Error opening file: " + filePath);
        }

        outFile << "Timestamp [s]" << "  " << "UUID1" << " " << "UUID2" << std::endl;

        outFile.flush();

        while(running) {
            if(counter > 0) {
                sample_T sample = handle.front();
                handle.pop_front();

                outFile << std::get<0>(sample) << " " << std::get<1>(sample) << " ";
                const auto& optionalValues = std::get<2>(sample);
                if(optionalValues) {
                    for(const auto& value : optionalValues.value()) {
                        outFile << value << " ";
                    }
                }
                outFile << "\n";
                counter --;
            }
        }

        if(!running) {
            outFile.flush();
            outFile.close();
        }
        if(verbose) {
            std::cout << "Schreiben beendet" << std::endl;
        }
    }

public:
    Writer(std::string& format, std::string &filePath, std::deque<sample_T>& handle, std::atomic<int>& counter)
        :handle(handle), format(format) {
        outFile.open(filePath, std::ios::app);
        if (!outFile) {
            throw std::ios_base::failure("Error opening file: " + filePath);
        }

        // Starte den Thread erst nach dem Öffnen der Datei
        writerThread = std::thread(&Writer::write, this, std::ref(filePath), std::ref(counter));
    }

    ~Writer() {
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

void writeDatatoFile(std::map<Omniscope::Id, std::vector<std::pair<double, double>>> &captureData, std::string &filePath, std::vector<std::string> &UUID, bool &isJson) {
    // Datei öffnen
    if(filePath.empty()) {
        std::string filePath = "data.txt";
    }
    if(!isJson) {
        std::ofstream outFile(filePath, std::ios::app);
        if (!outFile.is_open()) {
            std::cerr << "Failed to open file: " << filePath << "\n";
            return;
        }

        // Überprüfen, ob die Datei leer ist, um die Kopfzeile nur einmal zu schreiben
        outFile.seekp(0, std::ios::end);
        if (outFile.tellp() == 0) { // Datei ist leer
            // Kopfzeile schreiben
            outFile << "Timestamp [s]" << "  ";
            for (const auto& id : UUID) {
                outFile << id <<" [V] , " ;
            }
            outFile << "\n";
        }

        // Iteriere durch die erste ID, um die Reihenfolge der x-Werte zu bestimmen
        auto firstDevice = captureData.begin();
        const auto& firstDeviceData = firstDevice->second;

        // Anzahl der Zeilen basierend auf der Größe des ersten Geräts
        size_t numRows = firstDeviceData.size();

        for (size_t i = 0; i < numRows; ++i) {
            // Schreibe x-Wert und y-Wert der ersten ID
            double xValue = firstDeviceData[i].first;
            double yValueFirst = firstDeviceData[i].second;

            outFile << fmt::format("{},{}", xValue, yValueFirst);

            // Schreibe die y-Werte der restlichen IDs für denselben Index
            for (auto it = std::next(captureData.begin()); it != captureData.end(); ++it) {
                const auto& deviceData = it->second;

                if (i < deviceData.size()) {
                    double yValue = deviceData[i].second;
                    outFile << fmt::format(",{}", yValue);
                }
                else {
                    // Falls keine weiteren Werte für dieses Gerät vorhanden sind
                    outFile << ",N/A";
                }
            }
            outFile << "\n"; // Zeilenumbruch nach allen IDs
        }

        outFile.close(); // Datei schließen
        if(verbose) {
            std::cout << "Daten wurden geschrieben" << std::endl;
        }
    }
    /*else {
         // JSON-Objekt erstellen
    nlohmann::json jsonData;

    // UUIDs und Daten hinzufügen
    for (const auto& [id, data] : captureData) {
        // Wenn die ID nicht in der UUID-Liste enthalten ist, überspringen
        if (std::find(UUID.begin(), UUID.end(), id) == UUID.end()) {
            continue;
        }

        // JSON-Eintrag für die aktuelle ID
        nlohmann::json deviceData = nlohmann::json::array();

        for (const auto& [x, y] : data) {
            deviceData.push_back({{"timestamp", x}, {"value", y}});
        }

        std::string ID = std::to_string(id.serial);
        // Daten der ID hinzufügen
        jsonData[ID] = deviceData;
    }

    // JSON in Datei schreiben
    std::ofstream outFile(filePath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filePath << "\n";
        return;
    }

    outFile << jsonData.dump(4); // JSON-Daten mit 4 Leerzeichen als Einrückung schreiben
    outFile.close();

    fmt::print("Data successfully written to {}\n", filePath);
    }*/
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

void printOrWrite(std::string &filePath, std::vector<std::string> &UUID, bool &isJson) {
    static bool printHeader = true;
    if(sampler.has_value()) { // write Data into file
        captureData.clear();
        sampler->copyOut(captureData);

        // Transform Data and push in deque -> das muss hier einmal komplett erfolgen sonst bekomm ich Datenverlustprobleme
        // starte seperaten WriterThread der die ganze Zeit aus der deque liest und schreibt -> Das Objekt läuft die ganze Zeit bis der Nutzer das Programm schließt
        // Das darf also nur einmal ausgeführt werden
        if(!captureData.empty()) {
            if(filePath.empty()) {
                for(const auto& [id, vec] : captureData) {
                    if(printHeader) {
                        std::cout << "Time[s]" << " , " << "Voltage[V]" <<std::endl;
                        printHeader = false;
                    }
                    for(const auto& [first, second] : vec) {
                        std::cout << first << " " << second << std::endl;
                        if(!running) {
                            break;
                        }
                    }
                    if(!running) {
                        break;
                    }
                }
            }
            else {
                writeDatatoFile(captureData, filePath, UUID, isJson);
            }
        }
    }
}

void printOrWrite2(std::string &filePath, std::vector<std::string> &UUID, bool &isJson) {
    static bool startWriter = true;
    static std::atomic<int> counter(0);
    if(sampler.has_value()) { // write Data into file
        captureData.clear();
        sampler->copyOut(captureData);

        // Transform Data and push in deque -> das muss hier einmal komplett erfolgen sonst bekomm ich Datenverlustprobleme
        Transformater* transformi = new Transformater(captureData, dataDeque, UUID, counter);
        delete transformi;

        // starte seperaten WriterThread der die ganze Zeit aus der deque liest und schreibt -> Das Objekt läuft die ganze Zeit bis der Nutzer das Programm schließt
        // Das darf also nur einmal ausgeführt werden
        std::string format = "csv";
        if(startWriter && !filePath.empty()) {
            Writer* writi = new Writer(format, filePath, dataDeque, counter);
            startWriter = false;
        }
    }
}

void startMeasurementAndWrite(std::vector<std::string> &UUID, std::string &filePath, bool &isJson) {
    while(running) {
        searchDevices();   // Init Scopes

        selectDevices(UUID);  // select only chosen devices

        printOrWrite2(filePath, UUID, isJson); // print the data in the console or save it in the given filepath
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


