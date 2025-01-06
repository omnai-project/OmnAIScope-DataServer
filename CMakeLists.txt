cmake_minimum_required(VERSION 3.20)

include(FetchContent)
FetchContent_Populate(
  vcpkg
  GIT_REPOSITORY https://github.com/microsoft/vcpkg.git
  GIT_TAG        2023.11.20
  SOURCE_DIR     "${CMAKE_SOURCE_DIR}/vcpkg"
)

list(APPEND CMAKE_PROJECT_TOP_LEVEL_INCLUDES "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake")
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES CMAKE_PROJECT_TOP_LEVEL_INCLUDES)

set(CMAKE_TOOLCHAIN_FILE "${vcpkg_SOURCE_DIR}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

project("MiniOmni" VERSION 0.0.1)

if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT> #---------|
        $<$<CONFIG:Debug>:/MTd> #---|-- Statically link the runtime libraries
        $<$<CONFIG:Release>:/MT> #--|
    )
endif()


include(cmake_helpers/BuildOptions.cmake)
include(cmake_helpers/FindOrFetch.cmake)

add_executable(MiniOmni
               data.cpp)

target_add_default_build_options(MiniOmni PUBLIC)

find_or_fetch_package(
  cmake_git_version 0.1 GIT_REPOSITORY
  https://github.com/dominicpoeschko/cmake_git_version.git GIT_TAG master)

add_subdirectory(aglio)
target_link_libraries(MiniOmni PUBLIC aglio::aglio)

find_package(libusb CONFIG REQUIRED)
target_include_directories(MiniOmni PUBLIC ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(MiniOmni PUBLIC ${LIBUSB_LIBRARIES})

find_package(fmt CONFIG REQUIRED)
target_link_libraries(MiniOmni PUBLIC fmt::fmt-header-only)

target_compile_features(MiniOmni PUBLIC cxx_std_23)