cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-fdiagnostics-color=always)
add_compile_options(-Wno-switch)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(WiFlyer)

add_subdirectory(Icarus/icarus)
