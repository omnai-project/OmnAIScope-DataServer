#include "data.hpp"


int main(int argc, char **argv) {

// Create Optional CLI Tool
    CLI::App app{"OmnAI CLI"};

    //Search for devices
    bool search = false;
    app.add_flag("-s,--search", search, "Prints all connected devices");

    std::vector<std::string> startUUID;

    app.add_option("-d,--device, --dev", startUUID, "Start the devices with the given UUIDs");

    std::string filePath;

    app.add_option("-o,--output", filePath, "Add a file you want the data to be saved in");


    CLI11_PARSE(app, argc, argv);

    if(search) { // search for devices and print the UUID
        searchDevices();
        printDevices();
    }

    while(running) {
        if(!startUUID.empty()) { // Start the measurment with a set UUID and FilePath, data will be written in the filepath or in the console
            std::thread exitThread(waitForExit);
            startMeasurementAndWrite(startUUID, filePath);
            exitThread.join(); // exit the programm with enter
        }
        else { // else all connected devices will be started and the data will be written in the default path

            std::thread exitThread(waitForExit);
            std::string defaultPath ="data.txt";

            // Init Scopes
            searchDevices();

            if(!devices.empty() && !sampler.has_value()) { // move the device in the sampler
                std::cout <<"Devices where found and are emplaced"<< std::endl;
                sampler.emplace(deviceManager, std::move(devices));
                if(sampler.has_value()) {
                    for (auto &device : sampler->sampleDevices) {
                        device.first->send(Omniscope::Start{});
                    }
                }
            }

            printOrWrite(defaultPath);

            if(!running) {
                exitThread.join();
            }
        }
    }

    std::cout << "End of the program" << std::endl;

}
