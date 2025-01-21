cmake_minimum_required(VERSION 3.20)

include(FetchContent)
FetchContent_Populate(
  vcpkg
  GIT_REPOSITORY https://github.com/microsoft/vcpkg.git
  GIT_TAG        2024.12.16
  SOURCE_DIR     "${CMAKE_SOURCE_DIR}/vcpkg"
)

FetchContent_Populate(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.3.2
)

list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES CMAKE_PROJECT_TOP_LEVEL_INCLUDES)

set(CMAKE_TOOLCHAIN_FILE "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")

project("MiniOmni" VERSION 0.0.1)

# Fetch Crow
FetchContent_Populate(
    crow
    GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
    GIT_TAG v1.2.0
    SOURCE_DIR "${CMAKE_BINARY_DIR}/crow"
)

if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MD> #---------|
        $<$<CONFIG:Debug>:/MDd> #---|-- Statically link the runtime libraries
        $<$<CONFIG:Release>:/MD> #--|
    )
endif()


include(cmake_helpers/BuildOptions.cmake)
include(cmake_helpers/FindOrFetch.cmake)

add_subdirectory(${cli11_SOURCE_DIR} ${cli11_BINARY_DIR})
add_definitions(-D_WIN32_WINNT=0x0A00) 

add_executable(MiniOmni
               src/data.cpp)

#target_add_default_build_options(MiniOmni PUBLIC)

find_or_fetch_package(
  cmake_git_version 0.1 GIT_REPOSITORY
  https://github.com/dominicpoeschko/cmake_git_version.git GIT_TAG master)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR}/crow/include/)

find_package(Boost REQUIRED COMPONENTS system date_time optional)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::system Boost::date_time Boost::optional)

add_subdirectory(aglio)
target_link_libraries(MiniOmni PUBLIC aglio::aglio)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

target_link_libraries(MiniOmni PRIVATE PkgConfig::libusb)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(MiniOmni PUBLIC fmt::fmt-header-only)

target_link_libraries(MiniOmni PRIVATE CLI11::CLI11)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(MiniOmni PRIVATE nlohmann_json::nlohmann_json)

target_compile_features(MiniOmni PUBLIC cxx_std_23)
