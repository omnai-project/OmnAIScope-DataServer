#include "data.hpp"

void waitForExit(){
    std::cout << "Programm startet. Drücke Enter um das Programm zu beenden: " << std::endl; 
    std::cin.sync();
    std::cin.get(); 
    running = false; 
}

void initDevices(std::vector<std::shared_ptr<OmniscopeDevice>> &devices) {
  constexpr int VID = 0x2e8au;
  constexpr int PID = 0x000au;

  devices = deviceManager.getDevices(VID, PID);
  std::cout << "Found " << devices.size() << " devices.\n";
}

void sampleAndWriteToFile(const std::vector<std::pair<double, double>>& data){
    const std::string filename = "data.txt";

     {
        std::ofstream outFile(filename, std::ios::trunc);
        if (!outFile) {
            std::cerr << "Fehler beim Öffnen der Datei: " << filename << std::endl;
            return;
        }
    }
    for(const auto& [first, second] : data){
        if(running){
            std::ofstream outFile(filename, std::ios::app);
             if (!outFile) {
                 std::cerr << "Fehler beim Öffnen der Datei: " << filename << std::endl;
                return;
                }
             if (outFile.is_open()) {
                outFile << first << "," << second << "\n";
                outFile.flush();
                outFile.close();
            }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else {
            if(sampler.has_value()){
                 std::ofstream outFile(filename, std::ios::app);
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
    }
}

int main(){

// to exit the programm with enter 
std::thread exitThread(waitForExit);

std::thread pythonThread([]() {
        std::system("python ../../src/show.py"); // Python-Visualisierung starten
    });

while(running){

    // Init Scopes 
    if(!sampler.has_value()){
    devices.clear();
    deviceManager.clearDevices();
    initDevices(devices); // check if devices are connected
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Pause between checks 
    }

    if(!devices.empty() && !sampler.has_value()){ // move the device in the sampler
        std::cout <<"Devices where found and are emplaced"<< std::endl; 
        sampler.emplace(deviceManager, std::move(devices));  
    }

    if(sampler.has_value()){ // write Data into file 

         for (auto &device : sampler->sampleDevices) {
          device.first->send(Omniscope::Start{});
        }
        sampler->copyOut(captureData);
        for(const auto& [id, vec] : captureData){
            fmt::print("dev: {}\n", id);
            sampleAndWriteToFile(vec); 
        }
    }
}

pythonThread.join();
exitThread.join(); 

std::cout << "Programm beendet" << std::endl; 

}