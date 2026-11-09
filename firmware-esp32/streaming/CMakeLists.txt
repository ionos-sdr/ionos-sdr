# SPDX-License-Identifier: MIT
#
# FG23 SDR — ESP32-S3 side: ESP-IDF project file for the hybrid environments
# (framework = arduino, espidf in platformio.ini). The plain Arduino
# environments never read this file.
#
# What the hybrid build is: the Arduino core is compiled from source as one
# ESP-IDF component next to src/, with the sdkconfig coming from the
# SDKCONFIG_DEFAULTS file named in platformio.ini (board_build.cmake_extra_args).
# There is no precompiled lib-builder layer and no custom_sdkconfig rewrite in
# between: whatever the defaults file says is what kconfgen gets.
#
# The per-environment knobs arrive as CMake cache variables from
# platformio.ini (board_build.cmake_extra_args):
#   SDKCONFIG_DEFAULTS       the defaults file (consumed by project.cmake)
#   ARDUINO_USB_MODE         1 = HWCDC (USB-Serial/JTAG), 0 = TinyUSB OTG
#   ARDUINO_USB_CDC_ON_BOOT  1 = Serial is the USB CDC, Serial0 is UART0
#
# WHY THE USB MACROS GO THROUGH HERE AND NOT ONLY THROUGH build_flags: in the
# hybrid build the Arduino core is an IDF component and is compiled with the
# flags CMake assigns to it; PlatformIO's build_flags reach only src/ and the
# lib_deps. HardwareSerial.h decides the TYPE of "Serial" from
# ARDUINO_USB_CDC_ON_BOOT / ARDUINO_USB_MODE, so the core and the application
# MUST see the same values or they link against two different objects named
# Serial. Making them global compile options is the only safe way.

cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

foreach(_usb_macro ARDUINO_USB_MODE ARDUINO_USB_CDC_ON_BOOT ARDUINO_USB_MSC_ON_BOOT ARDUINO_USB_DFU_ON_BOOT)
  if(DEFINED ${_usb_macro})
    idf_build_set_property(COMPILE_OPTIONS "-D${_usb_macro}=${${_usb_macro}}" APPEND)
  endif()
endforeach()

project(fg23_sdr_esp32)
