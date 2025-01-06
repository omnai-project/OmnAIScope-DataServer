#include "data.hpp"

int main(){

// load submodule 

// hardcoded VID and PID 

static constexpr int VID = 0x2e8au;
static constexpr int PID = 0x000au;

// Init devices 

auto devices = deviceManager.getDevices(VID, PID); 

std::cout << devices.size() << std::endl; 

// instanciate DataObject 

// load data from scope 

// write data into txt file

// wait 

}