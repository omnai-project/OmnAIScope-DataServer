#include "data.hpp"

// Buffer size is currently set according to feeling and can still be adjusted
inline constexpr std::size_t BufferMinMiB = 10;
inline constexpr std::size_t BufferMaxMiB = 1000;

int main(int argc, char **argv) {

    GOOGLE_PROTOBUF_VERIFY_VERSION;

    std::signal(SIGINT, customSignalHandler);

    running = true;

    ControlWriter controlWriter{}; 


// Create Optional CLI Tool
    CLI::App app{"OmnAI CLI"};

    //Search for devices
    bool search = false;
    app.add_flag("-s,--search", search, "Prints all connected devices, color identical to current LED color");

    std::vector<std::string> startUUID;
    app.add_option("-d,--device, --dev", startUUID, "Start the devices with the given UUIDs");

    std::string filePath;
    app.add_option("-o,--output", filePath, "Add a file you want the data to be saved in");

    app.add_flag("-v,--verbose", verbose, "Add extra for debugging information");

    bool isJson = false;
    app.add_flag("-j,--json", isJson, "Add if you want the file to be in a JSON format");

    bool WS = false;
    app.add_flag("-w,--websocket", WS, "Starts the websocket. To send data a UUID has to be given");

    int port = 0;
    app.add_option("-p, --port", port, "Sets the port for the websocket to start on.");

    bool printVersion = false;
    app.add_flag("--version", printVersion, "Prints the current version. Version is set via a git tag.");

    int bufferSizeMB = -1;
    app.add_option("-b, --buffer-size", bufferSizeMB, "Set save-buffer size in MiB (10–1000)");

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

    if (bufferSizeMB != -1) {
    	if (bufferSizeMB < BufferMinMiB || bufferSizeMB > BufferMaxMiB) {
        	std::cerr << "Error: Buffer size must be between 10 and 1000 MiB\n";
           	return 1;
        }
        size_t newBytes = static_cast<size_t>(bufferSizeMB) * 1024 * 1024;
        bool ok = saveDataQueue.resizeMaxBytes(newBytes);
        std::cout << "Save-buffer resize returned: "
                 << (ok ? "OK" : "FAILED")
                 << ", new capacity = " << saveDataQueue.maxBytes()/(1024*1024)
                 << " MiB\n";
    }


    if(printVersion) {
        std::cout << "Version " << PROJECT_VERSION << std::endl;
        running = false;
    }

    if(port != 0 && WS == false ) {
        std::cout << "You cant set a port if you dont start the websocket. Usage: Programm -w -p <port> ." << std::endl;
        running = false;
    }
    if(port == 0 && WS == true ) {
        std::cout << "You cant use a websocket without setting a port. Usage: Programm -w -p <port> ." << std::endl;
        running = false;
    }

    if(search && running) { // search for devices and print the UUID
        searchDevices();
        printDevices();
    }

    if(WS && running) {
        websocket = std::thread(StartWS, std::ref(port), std::ref(controlWriter));
    }

    DataDestination destination = DataDestination::WS;
    FormatType format = FormatType::CSV;
    while(running) {
        if(!startUUID.empty()) { // Start the measurment with a set UUID and FilePath, data will be written in the filepath or in the console
            destination = DataDestination::LOCALFILE;
            if(isJson) {
                format = FormatType::JSON;
            }
            auto measurement = std::make_shared<Measurement>(startUUID, filePath, 10000, format, destination);
            measurement->startLocal(controlWriter);
        }
    }

    if(!running) {
        ExitProgramm();
    }
}
