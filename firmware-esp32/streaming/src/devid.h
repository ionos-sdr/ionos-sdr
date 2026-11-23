/* devid.h — unit identity: serial number, firmware version, build, manufacturing
 * and programming dates. MIT, HA7DCD.
 *
 *  serial     NVS "ionos/serial", set once with  SERIAL <id>  (e.g. SERIAL ID2);
 *             until then "ESP-xxxxxx" from the MAC
 *  mfg date   NVS "ionos/mfg",    set with       MFG <yyyy-mm-dd>
 *  prog date  NVS "ionos/prog":   written automatically the first time a new
 *             firmware build (FW_VERSION + FW_BUILD_UTC) runs with valid NTP time
 *  build      FW_BUILD_UTC (compile time), version FW_VERSION (git describe)
 */
#ifndef DEVID_H
#define DEVID_H
#include <stdbool.h>

void        devid_init(void);              /* after NVS init (menu_init) */
void        devid_tick(bool wifi_up);      /* main loop: NTP, programming-date stamp */
bool        devid_cmd(const char *cmd);    /* console: SERIAL / MFG / ID */
const char *devid_serial(void);
const char *devid_fw(void);
const char *devid_build(void);
const char *devid_mfg(void);               /* "-" if not set */
const char *devid_prog(void);              /* "pending" until NTP */
void        devid_print(void);             /* one console line */

#endif
