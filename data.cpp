#include "data.hpp"

void waitForExit(){
    std::cout << "Programm startet. Drücke Enter um das Programm zu beenden: " << std::endl; 
    std::cin.get(); 
    running = false; 
}

void initDevices() {
  constexpr int VID = 0x2e8au;
  constexpr int PID = 0x000au;

  auto devices = deviceManager.getDevices(VID, PID);
  std::cout << "Found " << devices.size() << " devices.\n";
}


int main(){

// to exit the programm with enter 
std::thread exitThread(waitForExit);

while(running){

    // Init Scopes 
    if(!sampler.has_value()){
    devices.clear();
    deviceManager.clearDevices();
    initDevices(); // check if devices are connected
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Pause between checks 
    }

    // Get Data if Scopes are connected 
    if(!devices.empty()){
        sampler(deviceManager, std::move(devices)); 
        sampler.copyOut(captureData); 

        if(sampler.has_value()){
        for(const auto& [id, vec] : data){
            std::cout << "Id:" << id << "\n";
            for(const auto& [first, second] : vec) {
                std::cout << first << "," << second << "\n"; 
            }
            std::this_thread::sleep_for(std::chrono::seconds(5); 
        }
        }
    }
}

exitThread.join(); 
std::cout << "Programm beendet" << std::endl; 

}