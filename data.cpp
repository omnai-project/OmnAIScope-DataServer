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
  devices.clear();
  deviceManager.clearDevices();
}


int main(){

// to exit the programm with enter 
std::thread exitThread(waitForExit);

while(running){
    initDevices(); // check if devices are connected 
    std::this_thread::sleep_for(std::chrono::seconds(5)); 
}

exitThread.join(); 
std::cout << "Programm beendet" << std::endl; 

// instanciate DataObject 

// load data from scope 

// write data into txt file

// wait 

}