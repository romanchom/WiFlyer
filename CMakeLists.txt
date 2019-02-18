cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE "$ENV{IDF_PATH}/tools/cmake/toolchain-esp32.cmake")
set(IDF_TARGET "esp32")
set(CMAKE_CXX_STANDARD 17)

project(WiFlyer)
add_compile_options(-fdiagnostics-color=always)

set (IDF_EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/config")
include("$ENV{IDF_PATH}/tools/cmake/idf_functions.cmake")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

add_subdirectory(Icarus)

add_executable(${PROJECT_NAME}.elf
    "main.cpp"
    "WiFi.c"
    "SensorsImpl.cpp"
)

target_link_libraries(${PROJECT_NAME}.elf Icarus)

set(IDF_BUILD_ARTIFACTS ON)
set(IDF_BUILD_ARTIFACTS_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(IDF_PROJECT_EXECUTABLE ${PROJECT_NAME}.elf)

set(IDF_COMPONENTS
    driver
    bootloader
    freertos
    newlib
    esp32
    cxx
    esptool_py
    config
)

idf_import_components(COMPONENTS $ENV{IDF_PATH} esp-idf)
idf_link_components(${PROJECT_NAME}.elf "${COMPONENTS}")
