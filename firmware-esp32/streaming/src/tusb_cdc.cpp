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
#include "esp_timer.h"

TusbCdc TusbSerial;

static TaskHandle_t s_pump_task = nullptr;

/* TinyUSB (cdc_device.c, weak): an IN transfer finished, the FIFO has room.
 * Wakes the pump at once instead of at its next 1 ms tick - with the tick
 * alone the CDC stayed at ~800 kB/s, under the 812 kB/s of 200 ksps. */
/* Statistics for the throughput investigation (US command). */
static volatile uint32_t st_txc = 0;          /* IN transfers completed */
static volatile uint32_t st_txc_gap_max = 0;  /* longest time between two completions [us] */
static volatile uint64_t st_txc_last_us = 0;
static volatile uint32_t st_fifo_full = 0;    /* pump found the TinyUSB FIFO full */
static volatile uint32_t st_ring_empty = 0;   /* pump found nothing to send */
static volatile uint32_t st_moves = 0;        /* pump calls that moved data */
static volatile uint64_t st_bytes = 0;        /* bytes handed to the TinyUSB FIFO */
static volatile uint32_t st_fifo_min_room = 0xFFFFFFFFu;
/* completion-interval histogram [us]: <3.5 ms, <5, <7, <15, <50, >=50 ms; and the total
 * time spent in gaps >= 15 ms (where the bus time goes) */
static volatile uint32_t st_hist[6];
static volatile uint64_t st_long_us = 0;

extern "C" void tud_cdc_tx_complete_cb(uint8_t itf)
{
  (void)itf;
  uint64_t now = esp_timer_get_time();
  if (st_txc_last_us) {
    uint32_t g = (uint32_t)(now - st_txc_last_us);
    if (g > st_txc_gap_max) st_txc_gap_max = g;
    int k = g < 3500 ? 0 : g < 5000 ? 1 : g < 7000 ? 2 : g < 15000 ? 3 : g < 50000 ? 4 : 5;
    st_hist[k]++;
    if (g >= 15000) st_long_us += g;
  }
  st_txc_last_us = now;
  st_txc++;
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

void TusbCdc::discard()
{
  if (!m_ok) return;
  m_tail = m_head;                           /* the pump only advances m_tail */
  tud_cdc_n_write_clear(0);
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

/* DWC2 (ESP32-S2/S3 OTG FS at 0x60080000): DIEPINT2 bit 4 INTKNTXFEMP = an IN
 * token arrived for EP2 (CDC data IN) while its TX FIFO was empty, i.e. the
 * DEVICE answered NAK. TinyUSB does not use this bit; we read and clear it.
 * DIEPTXF2 shows the TX FIFO depth actually allocated (words, upper 16 bits). */
#define DWC2_DIEPINT2  (*(volatile uint32_t *)0x60080948u)
#define DWC2_DIEPTXF2  (*(volatile uint32_t *)0x60080108u)
static volatile uint32_t st_nak_seen = 0, st_nak_polls = 0;
/* Gap probe: while no IN transfer has completed for > 10 ms, look at EP2 and the bus.
 * DIEPCTL2 bit31 EPENA = a transfer is armed; DIEPTSIZ2 [18:0] bytes left;
 * DSTS [21:8] frame number of the last SOF (does the host still send frames?). */
#define DWC2_DIEPCTL2  (*(volatile uint32_t *)0x60080940u)
#define DWC2_DIEPTSIZ2 (*(volatile uint32_t *)0x60080950u)
#define DWC2_DTXFSTS2  (*(volatile uint32_t *)0x60080958u)
#define DWC2_DSTS      (*(volatile uint32_t *)0x60080808u)
static volatile uint32_t gp_samples = 0, gp_armed = 0, gp_unarmed = 0, gp_progress = 0,
                         gp_still = 0, gp_sof_stuck = 0, gp_gaps = 0,
                         gp_fifo_full = 0, gp_fifo_empty = 0, gp_nak = 0;
static uint32_t gp_t_ms[8], gp_len_ms[8];
static uint32_t gp_last_tsiz = 0, gp_last_fn = 0; static uint64_t gp_gap_id = 0;
/* EP2 TX FIFO resize (experiment, "UT<words>"): TinyUSB gives the bulk IN only 2 packets
 * (32 words, double-buffered); the S3 has 256 words in all and ~130 sit unused between the
 * RX FIFO (62) and EP2's start (192). Grow EP2 downward while its transfer is idle. */
#define DWC2_GRSTCTL   (*(volatile uint32_t *)0x60080010u)
#define DWC2_GRXFSIZ   (*(volatile uint32_t *)0x60080024u)
static bool ep2_fifo_resize(uint32_t words)
{
  uint32_t f = DWC2_DIEPTXF2, top = (f & 0xFFFFu) + (f >> 16);   /* end stays where it is */
  if (words < 16 || words > top - (DWC2_GRXFSIZ & 0xFFFFu)) return false;
  TaskHandle_t th = xTaskGetHandle("TinyUSB");
  if (th) vTaskSuspend(th);
  bool ok = false;
  for (int i = 0; i < 200; i++) {                 /* wait for the armed transfer to finish */
    if (!(DWC2_DIEPCTL2 & (1u << 31))) { ok = true; break; }
    vTaskDelay(1);
  }
  if (ok) {
    DWC2_DIEPTXF2 = (words << 16) | (top - words);
    DWC2_GRSTCTL = (2u << 6) | (1u << 5);          /* flush TX FIFO 2 */
    for (int i = 0; i < 10000 && (DWC2_GRSTCTL & (1u << 5)); i++) { }
  }
  if (th) vTaskResume(th);
  return ok;
}

static void gap_probe()
{
  uint64_t last = st_txc_last_us;
  if (!last) return;
  uint64_t now = esp_timer_get_time();
  if (now - last < 10000) return;
  uint32_t ctl = DWC2_DIEPCTL2, tsiz = DWC2_DIEPTSIZ2 & 0x7FFFFu, fn = (DWC2_DSTS >> 8) & 0x3FFFu;
  if (gp_gap_id != last) {
    gp_gap_id = last; gp_t_ms[gp_gaps & 7] = (uint32_t)(last / 1000); gp_gaps++;
    gp_last_tsiz = tsiz; gp_last_fn = fn; return;
  }
  gp_len_ms[(gp_gaps - 1) & 7] = (uint32_t)((now - last) / 1000);
  gp_samples++;
  uint32_t room = DWC2_DTXFSTS2 & 0xFFFFu, depth = DWC2_DIEPTXF2 >> 16;
  if (room == 0) gp_fifo_full++; else if (room >= depth) gp_fifo_empty++;
  if (DWC2_DIEPINT2 & (1u << 4)) { DWC2_DIEPINT2 = (1u << 4); gp_nak++; }
  if (ctl & (1u << 31)) gp_armed++; else gp_unarmed++;
  if (tsiz != gp_last_tsiz) gp_progress++; else gp_still++;
  if (fn == gp_last_fn) gp_sof_stuck++;
  gp_last_tsiz = tsiz; gp_last_fn = fn;
}

void TusbCdc::pump_once(uint32_t *moved)
{
  *moved = 0;
  st_nak_polls++;
  if (DWC2_DIEPINT2 & (1u << 4)) { DWC2_DIEPINT2 = (1u << 4); st_nak_seen++; }
  gap_probe();
  if (!m_ok) return;
  if (m_flood_until_ms) {
    /* Throughput test: fill the FIFO with dummy bytes, the stream is dropped. */
    if ((int32_t)(millis() - m_flood_until_ms) >= 0) { m_flood_until_ms = 0; return; }
    m_tail = m_head;
    static uint8_t junk[4096];
    uint32_t room = tud_cdc_n_write_available(0);
    if (room < st_fifo_min_room) st_fifo_min_room = room;
    if (!room) { st_fifo_full++; tud_cdc_n_write_flush(0); return; }
    if (room > sizeof junk) room = sizeof junk;
    uint32_t w = tud_cdc_n_write(0, junk, room);
    tud_cdc_n_write_flush(0);
    st_moves++; st_bytes += w; *moved = w;
    return;
  }
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
  if (!fill) { st_ring_empty++; return; }
  uint32_t room = tud_cdc_n_write_available(0);
  if (room < st_fifo_min_room) st_fifo_min_room = room;
  if (!room) { st_fifo_full++; tud_cdc_n_write_flush(0); return; }
  uint32_t n = fill < room ? fill : room;
  uint32_t pos = tail % m_size;
  uint32_t first = m_size - pos;
  if (first > n) first = n;
  uint32_t w = tud_cdc_n_write(0, m_ring + pos, first);
  if (w == first && n > first) w += tud_cdc_n_write(0, m_ring, n - first);
  tud_cdc_n_write_flush(0);
  m_tail = tail + w;
  m_pumped += w;
  st_moves++; st_bytes += w;
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

bool tusb_cdc_cmd(const char *line)
{
  if (line[0] != 'U' || !line[1]) return false;
  static uint64_t t_last = 0, b_last = 0; static uint32_t c_last = 0;
  if (line[1] == 'S' && !line[2]) {
    uint64_t now = esp_timer_get_time();
    double dt = t_last ? (now - t_last) / 1e6 : 0.0;
    uint64_t b = st_bytes; uint32_t c = st_txc;
    TaskHandle_t th = xTaskGetHandle("TinyUSB");
    Serial0.printf("USBST: %.1f s  in-xfers %lu (%.0f/s)  %.1f kB/s to FIFO  max gap %lu us  "
                   "fifo-full %lu  ring-empty %lu  moves %lu  min FIFO room %lu  ring max %lu/%lu  "
                   "tusb prio %u core %d\n",
                   dt, (unsigned long)(c - c_last), dt > 0 ? (c - c_last) / dt : 0.0,
                   dt > 0 ? (b - b_last) / dt / 1000.0 : 0.0, (unsigned long)st_txc_gap_max,
                   (unsigned long)st_fifo_full, (unsigned long)st_ring_empty, (unsigned long)st_moves,
                   (unsigned long)st_fifo_min_room, (unsigned long)TusbSerial.ring_max_fill(),
                   (unsigned long)TusbSerial.ring_size(),
                   th ? (unsigned)uxTaskPriorityGet(th) : 0u, th ? (int)xTaskGetCoreID(th) : -1);
    Serial0.printf("USBHIST: <3.5ms %lu  <5 %lu  <7 %lu  <15 %lu  <50 %lu  >=50 %lu  | time in gaps>=15ms %.1f ms\n",
                   (unsigned long)st_hist[0], (unsigned long)st_hist[1], (unsigned long)st_hist[2],
                   (unsigned long)st_hist[3], (unsigned long)st_hist[4], (unsigned long)st_hist[5],
                   st_long_us / 1000.0);
    Serial0.printf("USBDWC2: device-NAK seen in %lu of %lu pump polls  EP2 TX FIFO %lu words @%lu\n",
                   (unsigned long)st_nak_seen, (unsigned long)st_nak_polls,
                   (unsigned long)(DWC2_DIEPTXF2 >> 16), (unsigned long)(DWC2_DIEPTXF2 & 0xFFFFu));
    Serial0.printf("USBGAP: gaps>10ms %lu  samples %lu  armed %lu unarmed %lu  progress %lu still %lu  sof-stuck %lu\n",
                   (unsigned long)gp_gaps, (unsigned long)gp_samples, (unsigned long)gp_armed,
                   (unsigned long)gp_unarmed, (unsigned long)gp_progress, (unsigned long)gp_still,
                   (unsigned long)gp_sof_stuck);
    Serial0.printf("USBGAP2: in gap: TX FIFO full %lu empty %lu  NAK %lu |",
                   (unsigned long)gp_fifo_full, (unsigned long)gp_fifo_empty, (unsigned long)gp_nak);
    for (uint32_t i = gp_gaps > 8 ? gp_gaps - 8 : 0; i < gp_gaps; i++)
      Serial0.printf(" %lu+%lu", (unsigned long)gp_t_ms[i & 7], (unsigned long)gp_len_ms[i & 7]);
    Serial0.printf("\n");
    gp_fifo_full = gp_fifo_empty = gp_nak = 0;
    gp_gaps = gp_samples = gp_armed = gp_unarmed = gp_progress = gp_still = gp_sof_stuck = 0;
    st_nak_seen = 0; st_nak_polls = 0;
    for (int i = 0; i < 6; i++) st_hist[i] = 0;
    st_long_us = 0;
    t_last = now; b_last = b; c_last = c;
    st_txc_gap_max = 0; st_fifo_full = 0; st_ring_empty = 0; st_moves = 0; st_fifo_min_room = 0xFFFFFFFFu;
    return true;
  }
  if (line[1] == 'F') {
    int sec = atoi(line + 2); if (sec <= 0 || sec > 600) sec = 10;
    TusbSerial.m_flood_until_ms = millis() + (uint32_t)sec * 1000u;
    if (!TusbSerial.m_flood_until_ms) TusbSerial.m_flood_until_ms = 1;
    Serial0.printf("USB flood %d s\n", sec);
    return true;
  }
  if (line[1] == 'T') {
    uint32_t w = (uint32_t)atoi(line + 2);
    bool ok = ep2_fifo_resize(w);
    Serial0.printf("EP2 TX FIFO resize %lu: %s, now %lu words @%lu (GRXFSIZ %lu)\n", (unsigned long)w,
                   ok ? "ok" : "FAILED", (unsigned long)(DWC2_DIEPTXF2 >> 16),
                   (unsigned long)(DWC2_DIEPTXF2 & 0xFFFFu), (unsigned long)(DWC2_GRXFSIZ & 0xFFFFu));
    return true;
  }
  if (line[1] == 'P') {
    int p = atoi(line + 2); TaskHandle_t th = xTaskGetHandle("TinyUSB");
    if (th && p > 0 && p < configMAX_PRIORITIES) vTaskPrioritySet(th, (UBaseType_t)p);
    Serial0.printf("TinyUSB task priority %u\n", th ? (unsigned)uxTaskPriorityGet(th) : 0u);
    return true;
  }
  return false;
}

#else
bool tusb_cdc_cmd(const char *) { return false; }
#endif /* IONOS_USB_TINYUSB */
