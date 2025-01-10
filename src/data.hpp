#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include "../ai_omniscope-v2-communication_sw/src/OmniscopeSampler.hpp"
#include "CLI/CLI.hpp"

inline OmniscopeDeviceManager deviceManager{};
inline std::vector<std::shared_ptr<OmniscopeDevice>> devices;
inline std::optional<OmniscopeSampler> sampler{};
inline std::map<Omniscope::Id, std::vector<std::pair<double, double>>> captureData;
std::atomic<bool> running{true};

void waitForExit();
void initDevices();
void sampleAndWriteToFile(const std::vector<std::pair<double, double>>&, std::string &);
void printDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &);
void searchDevices();
void startMeasurementAndPrintInConsole(std::string &);
void startMeasurementAndSaveInFile(std::string &);

// CLI Tools
std::map<Omniscope::Id, std::vector<float>> deviceRgbMap;


// Functionality


void waitForExit() { // too Close the programm with enter
    std::cout << "Programm startet. Drücke Enter um das Programm zu beenden: " << std::endl;
    std::cin.sync();
    std::cin.get();
    running = false;
}

void initDevices() { // Initalize the connected devices
    constexpr int VID = 0x2e8au;
    constexpr int PID = 0x000au;

    devices = deviceManager.getDevices(VID, PID);
    std::cout << "Found " << devices.size() << " devices.\n";
}

void sampleAndWriteToFile(const std::vector<std::pair<double, double>>& data, std::string &filePath) {
    const std::string filename = filePath;
    int valWrittenAtSameTime = 0;

    std::ofstream outFile(filename, std::ios::app);
    if (!outFile) {
        std::cerr << "Fehler beim Öffnen der Datei: " << filename << std::endl;
        return;
    }
    outFile.close();
    // std::cout << "Datei geöffnet und Inhalt gelöscht" << std::endl;
    for(const auto& [first, second] : data) {
        if(running) {
            if(valWrittenAtSameTime == 0) {
                outFile.open(filename, std::ios::app);
                if (!outFile) {
                    std::cerr << "Fehler beim Öffnen der Datei: " << filename << std::endl;
                    return;
                }
                //std::cout << "Datei geöffnet" << std::endl;
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
                    std::cerr << "Fehler beim Öffnen der Datei: " << filename << std::endl;
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
            fmt::print("Device: {}\n", deviceId);
        }
        std::cout << "With -p, --play and the UUID you can start a measurement. With -f, file you can choose a path to save the data. Press Enter to stop the measurement." << std::endl;
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

void startMeasurementAndPrintInConsole(std::string &UUID) {
    while(running) {
        // Init Scopes
        if(!sampler.has_value()) {
            devices.clear();
            deviceManager.clearDevices();
            initDevices(); // check if devices are connected
        }
        
        // delete not selected devices from the list 
        if(!devices.empty() && !sampler.has_value()) { // move the device in the sampler
            std::cout <<"Devices where found and are emplaced"<< std::endl;
            for(const auto &device : devices){
            devices.erase(
                std::remove_if(
                    devices.begin(),
                    devices.end(),
            [&UUID](const std::shared_ptr<OmniscopeDevice>& device) {
                return !(device->getId()->serial == UUID); // Bedingung
            }),
            devices.end()
            );
            }
            if(!devices.empty()){
            sampler.emplace(deviceManager, std::move(devices));
            if(sampler.has_value()) {
                for (auto &device : sampler->sampleDevices) {
                    device.first->send(Omniscope::Start{});
                }
            }
            }
        }

        if(sampler.has_value()) { // write Data into file
            captureData.clear();
            sampler->copyOut(captureData);
            for(const auto& [id, vec] : captureData) {
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
        }
    }
}

void startMeasurementAndSaveInFile(std::string &filePath) {
    while(running) {
        // Init Scopes
        if(!sampler.has_value()) {
            devices.clear();
            deviceManager.clearDevices();
            initDevices(); // check if devices are connected
        }

        if(!devices.empty() && !sampler.has_value()) { // move the device in the sampler
            std::cout <<"Devices where found and are emplaced"<< std::endl;
            sampler.emplace(deviceManager, std::move(devices));
            if(sampler.has_value()) {
                for (auto &device : sampler->sampleDevices) {
                    device.first->send(Omniscope::Start{});
                }
            }
        }

        if(sampler.has_value()) { // write Data into file
            captureData.clear();
            sampler->copyOut(captureData);
            for(const auto& [id, vec] : captureData) {
                fmt::print("dev: {}\n", id);
                sampleAndWriteToFile(vec, filePath);
            }
        }
    }
}