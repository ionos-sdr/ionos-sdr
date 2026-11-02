# SPDX-License-Identifier: MIT
#
# FG23 SDR — the application as an ESP-IDF component (hybrid build only).
#
# Every .c/.cpp directly under src/ is part of the firmware. bench/ is NOT
# under src/ precisely so that its setup()/loop() pairs never get in here;
# PlatformIO's build_src_filter is ignored by the ESP-IDF builder, so this glob
# is what decides.
#
# secrets.h and station_config.h are git-ignored: copy them from the
# *.example.h files next to them before the first build.

file(GLOB app_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

# The Arduino core component takes its name from its directory: pioarduino
# hands over its package as framework-arduinoespressif32[-x.y.z], upstream
# arduino-esp32 dropped into components/ is simply "arduino". Find it by
# content among the registered components instead of hard-coding the name.
# (This runs in ESP-IDF's early requirement-expansion pass too, where only the
# component properties are visible — not the CMake cache.)
set(arduino_component "arduino")
idf_build_get_property(_component_targets __COMPONENT_TARGETS)
foreach(_target ${_component_targets})
  __component_get_property(_dir ${_target} COMPONENT_DIR)
  if(EXISTS "${_dir}/cores/esp32/Arduino.h")
    __component_get_property(arduino_component ${_target} COMPONENT_NAME)
  endif()
endforeach()

idf_component_register(
  SRCS ${app_sources}
  INCLUDE_DIRS "."
  REQUIRES
    ${arduino_component}
    # IDF APIs called directly from src/ (see the includes in main.cpp,
    # rgb_load.cpp, fg23_prog.cpp, tft.cpp)
    driver              # driver/spi_slave.h, driver/gpio.h
    esp_driver_spi
    esp_driver_gpio
    esp_wifi            # esp_wifi_set_bandwidth()
    lwip                # lwip/sockets.h, lwip/tcpip.h, lwip/dns.h
    esp_timer
    esp_hw_support      # esp_intr_alloc.h
    heap                # esp_heap_caps.h
    soc                 # soc/gpio_struct.h
    esp_rom             # esp_rom_crc.h
    esp_system          # esp_system.h, esp_freertos_hooks.h
    freertos)
