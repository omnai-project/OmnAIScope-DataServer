#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "../ai_omniscope-v2-communication_sw/src/OmniscopeSampler.hpp"
#include "CLI/CLI.hpp"
#include <tuple>
#include <functional>

// Declaration

inline OmniscopeDeviceManager deviceManager{};
inline std::vector<std::shared_ptr<OmniscopeDevice>> devices;
inline std::optional<OmniscopeSampler> sampler{};
inline std::map<Omniscope::Id, std::vector<std::pair<double, double>>> captureData;
std::atomic<bool> running{true};

void waitForExit();
void initDevices();
void writeDatatoFile(const std::vector<std::pair<double, double>>&, std::string &);
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void startMeasurementAndWrite(std::vector<std::string> &, std::string &);
void selectDevices();
void printOrWrite(std::string &); 
std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& );
std::string rgbToAnsi(const std::tuple<uint8_t, uint8_t, uint8_t>& );

// Initialization

void waitForExit() { // wait until the user closes the programm by pressing ENTER
    std::cout << "OmnAIView is starting. Press enter to stop the programm." << std::endl;
    std::cin.sync();
    std::cin.get();
    running = false;
}

void initDevices() { // Initalize the connected devices
    constexpr int VID = 0x2e8au;
    constexpr int PID = 0x000au;

    devices = deviceManager.getDevices(VID, PID);
    std::cout << "Found " << devices.size() << " devices.\n";
    for(const auto& device : devices){
        auto [r, g, b] = uuidToColor(device->getId()->serial);
        device->send(Omniscope::SetRgb{static_cast<std::uint8_t>(static_cast<int>(r)),
                                   static_cast<std::uint8_t>(static_cast<int>(g)),
                                   static_cast<std::uint8_t>(static_cast<int>(b))});
    }
}

void writeDatatoFile(const std::vector<std::pair<double, double>>& data, std::string &filePath) {
    const std::string filename = filePath;
    int valWrittenAtSameTime = 0;

    std::ofstream outFile(filename, std::ios::app);
    if (!outFile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    outFile.close();

    for(const auto& [first, second] : data) {
        if(running) {
            if(valWrittenAtSameTime == 0) {
                outFile.open(filename, std::ios::app);
                if (!outFile) {
                    std::cerr << "Error opening file: " << filename << std::endl;
                    return;
                }
            }
            if (outFile.is_open()) {
                //std::cout << "Schreiben beginnt" << std::endl;
                //auto start = std::chrono::high_resolution_clock::now();
                outFile << first << "," << second << "\n";
                if(!running) {
                    outFile.flush();
                    outFile.close();
                }
                //auto end = std::chrono::high_resolution_clock::now();
                valWrittenAtSameTime++;
                //std::cout << valWrittenAtSameTime << std::endl;
            }
            /* if(valWrittenAtSameTime > ) {
                 outFile.flush();
                 outFile.close();
                 valWrittenAtSameTime = 0;
             }*/
            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else {
            if(sampler.has_value()) {
                if(!outFile.is_open()) {
                    outFile.open(filename, std::ios::app);
                }
                if (!outFile) {
                    std::cerr << "Error opening file:" << filename << std::endl;
                    return;
                }
                std::cout << "Stop the Scope" << std::endl;
                outFile << "STOP\n"; // to stop the python script
                outFile.flush();
                outFile.close();
                for (auto &device : sampler->sampleDevices)
                {
                    device.first->send(Omniscope::Stop{});
                }
            }
            break;
        }
        if(!running) {
            break;
        }
    }
    if(outFile.is_open()) {
        outFile.flush();
        outFile.close();
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
        std::cout << "With -p, --play and the UUID you can start a measurement. With -f, --file you can choose a path with the filename to save the data to. Press Enter to stop the measurement." << std::endl;
        devices.clear();
        deviceManager.clearDevices();
        running = false;
    }
}

void searchDevices() {
    std::cout << "Geräte werden gesucht" << std::endl;
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

void printOrWrite(std::string &filePath) {
    std::cout << "hello" <<std::endl; 
    if(sampler.has_value()) { // write Data into file
        captureData.clear();
        sampler->copyOut(captureData);
        for(const auto& [id, vec] : captureData) {
            if(filePath.empty()) {
                std::cout << "filepath empty" << std::endl; 
                fmt::print("dev: {}\n", id);
                for(const auto& [first, second] : vec) {
                    std::cout << "Time:" << first << "," << "Voltage" << second << std::endl;
                    if(!running) {
                        break;
                    }
                }
                if(!running) {
                    break;
                }
            }
            else {
                std::cout << "filepath nicht empty" << std::endl; 
                fmt::print("dev: {}\n", id);
                writeDatatoFile(vec, filePath);
            }
        }
    }
}

void startMeasurementAndWrite(std::vector<std::string> &UUID, std::string &filePath) {
    while(running) {
        searchDevices();   // Init Scopes

        selectDevices(UUID);  // select only chosen devices

        printOrWrite(filePath); // print the data in the console or save it in the given filepath 
    }
}

std::tuple<uint8_t, uint8_t, uint8_t> uuidToColor(const std::string& uuid) {
    // std::hash anwenden
    size_t hashValue = std::hash<std::string>{}(uuid);

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


