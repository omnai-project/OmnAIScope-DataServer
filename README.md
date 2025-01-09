# Setting Up and Building the MiniOmni Project

This document provides step-by-step instructions to set up your environment, build, and run the MiniOmni project on Windows as well as a Rasberry Pi with Debian and ARM. 

---
# Windows

## 1. Download and Install Visual Studio
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

## 2. Open Command Prompt and Set Up the Development Environment

1. Open the Command Prompt (CMD).
2. Source the environment variables required for the Visual Studio development environment by running:
   ```cmd
   "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
   ```
   This ensures the required compilers and tools are available in your command prompt session.

---

## 3. Create a New Folder for Your Work

1. Navigate to a location where you want to set up your project, e.g., `C:\Users\<username>`.
2. Create a new folder named `cpp` for your C++ projects:
   ```cmd
   mkdir cpp
   cd cpp
   ```

---

## 4. Clone the Project Repository and Initialize Submodules

1. Clone the MiniOmni project from the repository:
   ```cmd
   git clone git@github.com:AI-Gruppe/mini_omnai.git
   ```
2. Navigate into the cloned project directory:
   ```cmd
   cd mini_omnai
   ```
3. Initialize and update the submodules:
   ```cmd
   git submodule update --init --recursive
   ```
   This ensures all dependencies and additional components required by the project are retrieved.

---

## 5. Install vcpkg in the Project Directory

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

## 6. Download Python if necessary 

## 7. Generate the Build System with CMake

1. Return to the root project directory (e.g., `mini_omnai`):
   ```cmd
   cd ..
   ```
2. Use `CMake` to configure the project and generate the build system:
   ```cmd
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-static
   ```
   This command specifies the source directory (`.`), the build directory (`build`), and the toolchain file provided by `vcpkg`. It also configures the project to use the `x64-windows-static` triplet for static linking.

---

## 8. Build the Project with CMake

1. Build the project by running:
   ```cmd
   cmake --build build --config Release
   ```
   This will compile the source code and generate the `MiniOmni.exe` file in the `build/Release` directory.

---

## 9. Run the Built Release

1. Navigate to the directory containing the compiled executable:
   ```cmd
   cd build/Release
   ```
2. Run the program:
   ```cmd
   .\MiniOmni.exe
   ```

The program should start, and you can interact with it as instructed.

---

By following these steps, you should be able to successfully set up, build, and run the MiniOmni project. If you encounter any issues, ensure all dependencies are correctly installed and that your environment variables are properly configured.

# Rasberry Pi

# Build and Run Instructions on ARM

This guide explains how to build and run the program on ARM-based platforms. Follow the steps below to set up, compile, and run the program.

---

## Prerequisites

Make sure the following packages are installed:

- `build-essential`
- `cmake`
- `autoconf`
- `libudev-dev`

### Installing Required Packages (Debian-based Systems)

Run the following command to install the necessary packages:

``` cmd
sudo apt update
sudo apt install build-essential cmake autoconf libudev-dev
```

### Set VCPKG Variable 

```
export VCPKG_FORCE_SYSTEM_BINARIES=1
```
### Clone the Project Repository and Initialize Submodules

1. Clone the MiniOmni project from the repository:
   ```cmd
   git clone git@github.com:AI-Gruppe/mini_omnai.git
   ```
2. Navigate into the cloned project directory:
   ```cmd
   cd mini_omnai
   ```
3. Initialize and update the submodules:
   ```cmd
   git submodule update --init --recursive
   ```
   This ensures all dependencies and additional components required by the project are retrieved.

## Building 

Run the following commands to build the project 
```
   cmake ..
   cmake --build .
```

You can start the project with 
```
   ./MiniOmni
``` 

By following these steps, you should be able to successfully set up, build, and run the MiniOmni project. If you encounter any issues, ensure all dependencies are correctly installed and that your environment variables are properly configured.
---


