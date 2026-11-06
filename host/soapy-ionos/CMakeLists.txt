# SPDX-License-Identifier: MIT
# SoapyIonos - SoapySDR driver for the Ionos SDR (USB CDC I/Q stream)
# Copyright (c) 2026 Zoltan Doczi HA7DCD

cmake_minimum_required(VERSION 3.10)
project(SoapyIonos CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    # VS 2022 17.10+ made std::mutex's constructor constexpr; a module built that way
    # deadlocks on its first lock when the host process has already loaded an older
    # msvcp140.dll (PothosSDR, radioconda and SDR++ ship their own). Seen on hardware:
    # the reader thread hung on its first mutex, the hello was never registered.
    add_compile_definitions(_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR _CRT_SECURE_NO_WARNINGS)
endif()

find_package(SoapySDR "0.7" CONFIG REQUIRED)
find_package(Threads REQUIRED)

set(IONOS_LIBS Threads::Threads)
if(WIN32)
    list(APPEND IONOS_LIBS setupapi advapi32)
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET ionosSupport
    SOURCES
        src/IonosDevice.cpp
        src/IonosRegistration.cpp
        src/IonosSerial.cpp
        src/IonosProtocol.cpp
    LIBRARIES ${IONOS_LIBS}
)

option(IONOS_BUILD_TESTS "Build the parser unit test and the stream test" ON)
if(IONOS_BUILD_TESTS)
    enable_testing()
    add_executable(ionos_parser_test test/parser_test.cpp src/IonosProtocol.cpp)
    target_include_directories(ionos_parser_test PRIVATE src)
    add_test(NAME parser COMMAND ionos_parser_test)

    add_executable(ionos_stream_test test/stream_test.cpp)
    target_link_libraries(ionos_stream_test SoapySDR Threads::Threads)
endif()
