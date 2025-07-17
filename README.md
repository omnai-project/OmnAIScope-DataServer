## Introduction

The OmnAIScope-DataServer is a Minimum Viable Product (MVP) of the OmnAIScopeBackend designed as a Command-Line Interface (CLI) tool. 
The project’s primary purpose is to create a backend system capable of retrieving data from multiple OmnAIScopes and providing access to this data in multiple formats via a websocket connection. This can be used in various front-end interfaces (e.g. python, angular). 
Additionally, the Command-Line Interface (CLI) provides direct data output into multiple file formats or directly into the console.

## How to use the CLI-Tool 

The OmnAIScope-DataServer can be executed via the command line on Windows, Linux, and Raspberry Pi devices running ARM architecture.

### Commands

The CLI-Tool includes the commands: 

-h, --help Displays a help message with detailed usage instructions in the console.
-s, --search Lists the UUIDs of all connected devices, displaying their current LED color for easy identification.
-d, --device, --dev Initiates a measurement session for specified devices by their UUIDs. Multiple devices can be specified, and their data will be displayed directly in the console. 
-o, --output Writes the data from the selected devices to the specified file path. By default, the data is saved in .csv format.
-j, --json Changes the output file format to JSON. 
-w, --websocket Opens a Websocket, as well as a REST API. 
-p, --port Set a port for the Websocket to start on. 
-v, --verbose Prints out additional information about the software's process.
--version Prints out the current SW version 

#### How to use the Websocket: 

#### **The API documentation can be found under :https://github.com/AI-Gruppe/OmnAIScope_DataServer_API_Doc**

1. Start the Websocket 
Start the executable via 
   ```
   .\OmnAIScopeBackend -w -p <port>
   ```
1. Retrieve UUIDS via REST API 
Request the available UUIDs by accessing the REST API endpoint:
   ```
   curl http://<ip>:<port>/UUID
   ```
   Request additional information ( HW version, FW version, offset, scale of the calibration of each scope, UUIDs and colors)
   via 
   ```
   curl http://<ip>:<port>/v1/get_info
   ```
1. Establish a WS connection

   Connect to the WS via 
   ```
   wscat -c ws://<ip>:<port>/ws
   ```
1. Start the measurement
To start the measurement the client has to send the start command to the websocket:
   ```
   {"type":"start", "uuids":["<UUID>", "<UUID>",...], "rate":<int>,"format":"<format>"}
   ```

The client can set the : 
- UUID: datastreams measured via the device UUIDs
- RATE: sampling rate of measurement 
- FORMAT: format of the measurement 
> The default sample rate is 60 Sa/s, the maximal sample rate is 100000 Sa/s. At the end of the command you can also set a format, available formats are "json, csv, binary". The default > format is json. 

You should receive data in the chosen format from the chosen devices now. 

2.Stop the measurement 
To stop the measurement send the stop command to the websocket : 
```
{"type":"stop"}
```

3.Save the measurement 
AFTER stopping the measurement send the save command to the websocket, it is important to send the same uuids as used for the measurement: 
```
 {"type":"save", "uuids":["<UUID>", "<UUID>",...], "path":"<filepath>","format":"<format>"}
`` 

path: filename in which the file is saved 
format: format in which the file is saved 

Download the file from the API endpoint : /download/<filename>

## How to build the project
**Important: If you don't have access to the private communication submodule it is currently not possible to build this project locally. A CI Build is integrated for Linux that can be used in a fork** 

This document provides step-by-step instructions to set up your environment, build, and run the OmnAIScope-DataServer  on Windows as well as on Rasberry Pi devices running ARM architecture.

---
## Windows

### 1. Download and Install Visual Studio
To develop and build C++ applications, you need to install the required tools using Visual Studio.

1. Download the **Visual Studio Installer** from [Visual Studio](https://visualstudio.microsoft.com/downloads/).
2. Open the installer and choose to install the **Desktop Development with C++** workload.
3. Select the following individual components within the workload:
   - **MSVC v143 or newer**
   - ***MSVC 142 - VS 2019 C++ -x64/x86 Buildtools**
   - **Windows 11 SDK (10.0.22621...) or newer**
   - **C++ CMake Tools for Windows**
   - **Core Features of Test Tools - Build Tools**
   - **C++ Address Sanitizer**
4. Click **Install** and wait for the process to complete.

---

### 2. Open Command Prompt and Set Up the Development Environment

1. Open the Command Prompt (CMD).
2. Source the environment variables required for the Visual Studio development environment by running:
   ```cmd
   "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
   ```
   This ensures the required compilers and tools are available in your command prompt session.

---

### 3. Create a New Folder for Your Work

1. Navigate to a location where you want to set up your project, e.g., `C:\Users\<username>`.
2. Create a new folder named `cpp` for your C++ projects:
   ```cmd
   mkdir cpp
   cd cpp
   ```

---

### 4. Clone the Project Repository and Initialize Submodules

1. Clone the OmnAIScope-DataServer project from the repository:
   ```cmd
   git clone git@github.com:omnai-project/OmnAIScope-DataServer.git
   ```
2. Navigate into the cloned project directory:
   ```cmd
   cd OmnAIScope-DataServer
   ```
3. Initialize and update the submodules:
   ```cmd
   git submodule update --init --recursive
   ```
   This ensures all dependencies and additional components required by the project are retrieved.

---

### 5. Install vcpkg in the Project Directory

The `vcpkg` tool helps manage and install dependencies for your project.

1. Clone the `vcpkg` repository into your project directory:
   ```cmd
   git clone https://github.com/microsoft/vcpkg.git
   ```
2. Navigate into the `vcpkg` directory:
   ```cmd
   cd vcpkg
   ```
3. Bootstrap `vcpkg` to make it ready for use:
   ```cmd
   .\bootstrap-vcpkg.bat
   ```

If you know how to configure paths and toolchain files manually, `vcpkg` can also be installed elsewhere.

---

### 6. Download Python if necessary 

### 7. Generate the Build System with CMake

1. Return to the root project directory (e.g., `OmnAIScopeBackend`):
   ```cmd
   cd ..
   ```
1. Use `CMake` to configure the project and generate the build system:
   ```cmd
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
   ```
   This command specifies the source directory (`.`), the build directory (`build`), and the toolchain file provided by `vcpkg`. It also configures the project to use the `x64-windows` triplet for linking.

---

### 8. Build the Project with CMake

1. Build the project by running:
   ```cmd
   cmake --build build --config Release
   ```
   This will compile the source code and generate the `OmnAIScopeBackend.exe` file in the `build/Release` directory.

---

### 9. Run the Built Release

1. Navigate to the directory containing the compiled executable:
   ```cmd
   cd build/Release
   ```
1. Run the program:
   ```cmd
   .\OmnAIScopeBackend.exe
   ```

The program should start, and you can interact with it as instructed.

---

By following these steps, you should be able to successfully set up, build, and run the OmnAIScope-DataServer. If you encounter any issues, ensure all dependencies are correctly installed and that your environment variables are properly configured.

## Rasberry Pi

## Build and Run Instructions on ARM

This guide explains how to build and run the program on ARM-based platforms. Follow the steps below to set up, compile, and run the program.

---

### Prerequisites

Make sure the following packages are installed:

- `build-essential`
- `cmake`
- `autoconf`
- `libudev-dev`
- `automake`
- `autoconf-archive`

#### Installing Required Packages (Debian-based Systems)

Run the following command to install the necessary packages:

``` cmd
sudo apt update
sudo apt install build-essential cmake autoconf libudev-dev
```

#### Set VCPKG Variable 

```
export VCPKG_FORCE_SYSTEM_BINARIES=1
```
#### Clone the Project Repository and Initialize Submodules

1. Clone the OmnAIScope-DataServer project from the repository:
   ```cmd
   git clone git@github.com:omnai-project/OmnAIScope-DevDataServer.git
   ```
1. Navigate into the cloned project directory:
   ```cmd
   cd OmnAIScope-DevDataServer
   ```
1. Initialize and update the submodules:
   ```cmd
   git submodule update --init --recursive
   ```
   This ensures all dependencies and additional components required by the project are retrieved.

### Building 

Run the following commands to build the project 
```
   cmake ..
   cmake --build .
```

You can start the project with 
```
   ./OmnAIScopeBackend
``` 

By following these steps, you should be able to successfully set up, build, and run the OmnAIScope-DataServer project. If you encounter any issues, ensure all dependencies are correctly installed and that your environment variables are properly configured.
---


