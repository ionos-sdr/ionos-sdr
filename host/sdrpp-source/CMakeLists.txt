cmake_minimum_required(VERSION 3.13)
project(fg23_scan_source)

file(GLOB SRC "src/*.cpp")

include(${SDRPP_MODULE_CMAKE})

target_include_directories(fg23_scan_source PRIVATE "src/")
