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

# get latest git tag
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Fallback if no git tag exists
if (NOT GIT_VERSION)
    set(GIT_VERSION "0.0.0") # standard version
endif()

# Setze die Version in deinem Projekt
message(STATUS "Project version: ${GIT_VERSION}")
add_definitions(-DPROJECT_VERSION="${GIT_VERSION}")

list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES CMAKE_PROJECT_TOP_LEVEL_INCLUDES)

set(CMAKE_TOOLCHAIN_FILE "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")


project("MiniOmni" VERSION 0.0.1)

if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MD> #---------|
        $<$<CONFIG:Debug>:/MDd> #---|-- Statically link the runtime libraries
        $<$<CONFIG:Release>:/MD> #--|
    )
    #add_compile_options(/permissive-)
    #set(Boost_USE_STATIC_LIBS "On")
    #find_package( Boost 1.70 COMPONENTS system thread regex REQUIRED )
endif()


include(cmake_helpers/BuildOptions.cmake)
include(cmake_helpers/FindOrFetch.cmake)

add_subdirectory(${cli11_SOURCE_DIR} ${cli11_BINARY_DIR})

add_executable(MiniOmni
               src/data.cpp)

#target_add_default_build_options(MiniOmni PUBLIC)

find_or_fetch_package(
  cmake_git_version 0.1 GIT_REPOSITORY
  https://github.com/dominicpoeschko/cmake_git_version.git GIT_TAG master)

add_subdirectory(aglio)
target_link_libraries(MiniOmni PUBLIC aglio::aglio)

find_package(Boost REQUIRED COMPONENTS system thread filesystem asio regex)
target_include_directories(MiniOmni PRIVATE ${Boost_INCLUDE_DIRS})
target_link_libraries(MiniOmni PRIVATE Boost::asio Boost::system Boost::thread Boost::filesystem Boost::regex)
#target_compile_definitions(MiniOmni PUBLIC ASIO_STANDALONE=0)
#target_compile_definitions(MiniOmni PUBLIC BOOST_ASIO_USE_TS_EXECUTOR_AS_DEFAULT)

find_package(OpenSSL REQUIRED)
target_link_libraries(MiniOmni PRIVATE OpenSSL::SSL OpenSSL::Crypto)

find_package(Crow CONFIG REQUIRED)
target_link_libraries(MiniOmni PRIVATE Crow::Crow)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0)

target_link_libraries(MiniOmni PRIVATE PkgConfig::libusb)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(MiniOmni PUBLIC fmt::fmt-header-only)

target_link_libraries(MiniOmni PRIVATE CLI11::CLI11)
#target_include_directories(MiniOmni PRIVATE ${CLI11_SOURCE_DIR}/include)
#find_package(CLI11 CONFIG REQUIRED)
#target_link_libraries(MiniOmni PUBLIC CLI11::CLI11)

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(MiniOmni PRIVATE nlohmann_json::nlohmann_json)

target_compile_definitions(MiniOmni PRIVATE BOOST_ASIO_ENABLE_HANDLER_TRACKING)

target_compile_features(MiniOmni PUBLIC cxx_std_23)
