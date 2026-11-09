/* SPDX-License-Identifier: MIT
 *
 * usb_host.h - host command channel on the native USB CDC (HOST-M1 v2)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * docs/ionos-usb-protocol.md: the host sends text lines on the same CDC
 * port that carries the 1040-byte blocks; the device answers with MSG1
 * blocks (same 16-byte header, magic 'M','S','G','1', text payload), so
 * the host keeps ONE parser. Blocks written here never split an IQB2
 * block: usbhost_flush() runs right after flush_usb() and only writes when
 * a whole block fits.
 *
 * Commands handled here: FH<Hz>, R<sps>, G<shift>, CAP?, VER?, MODE,
 * STAT. Forwarded to the FG23 over the cmdlink: F o f i W T t s. Rejected:
 * the TX/diagnostic letters (c w p x b z y d k n g v M).
 */
#ifndef USB_HOST_H
#define USB_HOST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSG1_MAGIC 0x3147534Du
enum { MSG1_RESPONSE = 0, MSG1_HELLO = 1, MSG1_STATUS = 2, MSG1_ERROR = 3 };

void usbhost_init(void);
void usbhost_tick(uint32_t now_ms);          /* parse host lines, hello on connect, periodic status */
void usbhost_flush(void);                    /* write queued MSG1 blocks (after flush_usb) */
bool usbhost_send(uint8_t type, const char *text);   /* queue one MSG1 block; false = queue full (dropped) */
bool usbhost_usb_mode(void);                 /* MODE USB active */
void usbhost_set_mode(bool usb);
uint32_t usbhost_dropped(void);

#ifdef __cplusplus
}
#endif
#endif /* USB_HOST_H */
