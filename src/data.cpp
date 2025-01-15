#include "data.hpp"


int main(int argc, char **argv) {

// Create Optional CLI Tool
    CLI::App app{"OmnAI CLI"};

    //Search for devices
    bool search = false;
    app.add_flag("-s,--search", search, "Prints all connected devices, color identical to current LED color");

    std::vector<std::string> startUUID;
    app.add_option("-d,--device, --dev", startUUID, "Start the devices with the given UUIDs");

    std::string filePath;
    app.add_option("-o,--output", filePath, "Add a file you want the data to be saved in");

    bool isJson = false;
    //app.add_flag("-j,--json", isJson, "Add if you want the file to be in a JSON format");

    if (argc <= 1) {// if no parameters are given
        std::cout << app.help() << std::endl;
        return 0;
    }
    try {
        CLI11_PARSE(app, argc, argv);
    } catch (const CLI::ParseError &e) {
        // if input is incorrect
        std::cerr << app.help() << std::endl;
        return app.exit(e);
    }


    if(search) { // search for devices and print the UUID
        searchDevices();
        printDevices();
    }

    while(running) {
        if(!startUUID.empty()) { // Start the measurment with a set UUID and FilePath, data will be written in the filepath or in the console
            std::thread exitThread(waitForExit);
            startMeasurementAndWrite(startUUID, filePath, isJson);
            exitThread.join(); // exit the programm with enter
        }
    }
}
