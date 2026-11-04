/* SPDX-License-Identifier: MIT
 *
 * tusb_cdc.cpp - see tusb_cdc.h
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 */
#include "tusb_cdc.h"

#if IONOS_USB_TINYUSB

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TusbCdc TusbSerial;

static TaskHandle_t s_pump_task = nullptr;

/* TinyUSB (cdc_device.c, weak): an IN transfer finished, the FIFO has room.
 * Wakes the pump at once instead of at its next 1 ms tick - with the tick
 * alone the CDC stayed at ~800 kB/s, under the 812 kB/s of 200 ksps. */
extern "C" void tud_cdc_tx_complete_cb(uint8_t itf)
{
  (void)itf;
  if (s_pump_task) xTaskNotifyGive(s_pump_task);
}

static void pump_task(void *arg)
{
  TusbCdc *c = (TusbCdc *)arg;
  for (;;) {
    uint32_t moved = 0;
    c->pump_once(&moved);
    /* Nothing moved: the FIFO is full (host not reading yet) or the ring is
     * empty. Sleep until a transfer completes, at most one tick (the ring
     * may have been filled by the main loop meanwhile). */
    if (!moved) ulTaskNotifyTake(pdTRUE, 1);
  }
}

void TusbCdc::begin()
{
  if (m_ok) return;
  m_size = (uint32_t)m_ring_req;
  m_ring = (uint8_t *)heap_caps_malloc(m_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!m_ring) {
    m_size = 32u * 1024u;
    m_ring = (uint8_t *)heap_caps_malloc(m_size, MALLOC_CAP_8BIT);
  }
  if (!m_ring) return;

  const tinyusb_config_t tcfg = {};          /* descriptors from Kconfig, internal PHY */
  if (tinyusb_driver_install(&tcfg) != ESP_OK) return;

  tinyusb_config_cdcacm_t acm = {};
  acm.usb_dev = TINYUSB_USBDEV_0;
  acm.cdc_port = TINYUSB_CDC_ACM_0;
  acm.callback_rx = nullptr;                 /* polled from the main loop */
  acm.callback_rx_wanted_char = nullptr;
  acm.callback_line_state_changed = nullptr;
  acm.callback_line_coding_changed = nullptr;
  if (tusb_cdc_acm_init(&acm) != ESP_OK) return;

  m_ok = true;
  /* Core 0, above the Arduino loop (1) and the TinyUSB task (5), below WiFi. */
  xTaskCreatePinnedToCore(pump_task, "usbpump", 3072, this, 6, &s_pump_task, 0);
}

TusbCdc::operator bool() const
{
  return m_ok && tud_mounted() && tud_cdc_n_connected(0);
}

int TusbCdc::availableForWrite() const
{
  if (!m_ok) return 0;
  uint32_t fill = m_head - m_tail;
  return (int)(m_size - fill);
}

size_t TusbCdc::write(const uint8_t *buf, size_t n)
{
  if (!m_ok || !buf || !n) return 0;
  uint32_t head = m_head, tail = m_tail;
  uint32_t fill = head - tail;
  if (n > m_size - fill) return 0;           /* all or nothing: blocks stay whole */
  uint32_t pos = head % m_size;
  uint32_t first = m_size - pos;
  if (first > n) first = (uint32_t)n;
  memcpy(m_ring + pos, buf, first);
  if (n > first) memcpy(m_ring, buf + first, n - first);
  __sync_synchronize();                      /* data before the index */
  m_head = head + (uint32_t)n;
  fill += (uint32_t)n;
  if (fill > m_max_fill) m_max_fill = fill;
  return n;
}

void TusbCdc::pump_once(uint32_t *moved)
{
  *moved = 0;
  if (!m_ok) return;
  if (!tud_cdc_n_connected(0)) {
    /* No host: keep the ring empty, like the HWCDC dropping on "no host"
     * (the main loop counts the drops itself, see flush_usb()). */
    m_tail = m_head;
    return;
  }
  uint32_t head = m_head;
  __sync_synchronize();
  uint32_t tail = m_tail;
  uint32_t fill = head - tail;
  if (!fill) return;
  uint32_t room = tud_cdc_n_write_available(0);
  if (!room) { tud_cdc_n_write_flush(0); return; }
  uint32_t n = fill < room ? fill : room;
  uint32_t pos = tail % m_size;
  uint32_t first = m_size - pos;
  if (first > n) first = n;
  uint32_t w = tud_cdc_n_write(0, m_ring + pos, first);
  if (w == first && n > first) w += tud_cdc_n_write(0, m_ring, n - first);
  tud_cdc_n_write_flush(0);
  m_tail = tail + w;
  m_pumped += w;
  *moved = w;
}

int TusbCdc::available()
{
  return m_ok ? (int)tud_cdc_n_available(0) : 0;
}

int TusbCdc::read()
{
  if (!m_ok || !tud_cdc_n_available(0)) return -1;
  uint8_t c;
  return tud_cdc_n_read(0, &c, 1) == 1 ? c : -1;
}

#endif /* IONOS_USB_TINYUSB */
