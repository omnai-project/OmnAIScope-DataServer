#include "data.hpp"


int main(int argc, char **argv) {

    std::cout << "Hallo ich fange überhaupt an" <<std::endl; 

// Create Optional CLI Tool
    CLI::App app{"OmnAI CLI"};
    //Search for Devices
    bool search = false;
    app.add_flag("-s,--search", search, "Prints all connected devices");

    std::string startUUID;

    app.add_option("-p,--play", startUUID, "Start the device with the given UUID");

    std::string filePath;

    app.add_option("-f,--file", filePath, "Add a file you want the data to be saved in");


    CLI11_PARSE(app, argc, argv);

    /*std::thread pythonThread([]() {
        std::system("python ../../src/show.py"); // Python-Visualisierung starten
    });*/

    if(search) {
        searchDevices();
        printDevices();
    }

    while(running) {
        if(!startUUID.empty() && filePath.empty()) {
            std::thread exitThread(waitForExit); //Otherwise exit the programm with Enter
            startMeasurementAndPrintInConsole(startUUID);
            exitThread.join();
        }
        else if(!startUUID.empty() && !filePath.empty()) {
            std::thread exitThread(waitForExit); //Otherwise exit the programm with Enter
            startMeasurementAndSaveInFile(filePath);
            exitThread.join();
        }
        else {

            std::thread exitThread(waitForExit); //Otherwise exit the programm with Enter
            std::string defaultPath ="data.txt";

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
                auto start = std::chrono::high_resolution_clock::now();

                for(const auto& [id, vec] : captureData) {
                    fmt::print("dev: {}\n", id);
                    sampleAndWriteToFile(vec, defaultPath);
                }
                // std::this_thread::sleep_for(std::chrono::seconds(1)); // Pause between checks
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

                std::cout << "Zeit: " << duration << std::endl;
            }
            if(!running) {
                exitThread.join();
            }
        }
    }

    // pythonThread.join();

    std::cout << "Programm beendet" << std::endl;

}
