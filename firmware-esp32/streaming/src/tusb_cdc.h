/* SPDX-License-Identifier: MIT
 *
 * tusb_cdc.h - native USB CDC-ACM on the ESP32-S3 USB-OTG peripheral (TinyUSB
 *   through esp_tinyusb), with the subset of the HWCDC interface the firmware
 *   uses, so main.cpp / usb_host.cpp only swap the object behind RAW.
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Why (2026-11-04): the USB-Serial/JTAG (HWCDC) delivered at most ~776 kB/s;
 * the 200 ksps int16 stream needs 812 kB/s. Writes go into a PSRAM ring; a
 * pump task on core 0 moves the ring into the TinyUSB CDC FIFO, so a stalled
 * main loop (OLED 22 ms) or a slow host read (Windows) does not drop blocks
 * until the ring (default 256 kB, ~315 ms at 812 kB/s) is full.
 *
 * Host-visible differences to the HWCDC: a new USB PID (new COM number), no
 * chip reset on DTR/RTS, no USB-JTAG. Flashing stays on the CH343 UART.
 */
#pragma once
#include <Arduino.h>

class TusbCdc {
public:
  void setTxBufferSize(size_t bytes) { m_ring_req = bytes; }   /* before begin() */
  void setTxTimeoutMs(uint32_t) {}                              /* writes never block */
  void begin();
  /* true = the host has the port open (DTR asserted) and the device is mounted. */
  explicit operator bool() const;
  int    availableForWrite() const;      /* free bytes in the ring */
  size_t write(const uint8_t *buf, size_t n);   /* all or nothing */
  int    available();
  int    read();
  /* diagnostics */
  uint32_t ring_max_fill() const { return m_max_fill; }
  uint32_t ring_size() const { return m_size; }
  uint32_t pump_bytes() const { return m_pumped; }

  /* internal, used by the pump task */
  void pump_once(uint32_t *moved);
private:
  size_t   m_ring_req = 256u * 1024u;
  uint8_t *m_ring = nullptr;
  uint32_t m_size = 0;
  volatile uint32_t m_head = 0;    /* written by write() (producer) */
  volatile uint32_t m_tail = 0;    /* written by the pump (consumer) */
  volatile uint32_t m_max_fill = 0;
  volatile uint32_t m_pumped = 0;
  bool     m_ok = false;
};

extern TusbCdc TusbSerial;
