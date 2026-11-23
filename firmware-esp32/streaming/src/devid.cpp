/* devid.cpp — unit identity, see devid.h. MIT, HA7DCD. */
#include "devid.h"
#include <Arduino.h>
#include <time.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_mac.h"

#if defined(__has_include)
#  if __has_include("fw_version.h")
#    include "fw_version.h"
#  endif
#endif
#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif
#ifndef FW_BUILD_UTC
#define FW_BUILD_UTC __DATE__ " " __TIME__
#endif

Stream &diag_stream(void);

static char s_serial[24] = "";
static char s_mfg[16]    = "-";
static char s_prog[24]   = "pending";
static char s_buildid[96];
static bool s_prog_done  = false;
static bool s_ntp_started = false;

static void nvs_str_get(const char *key, char *out, size_t n)
{
  nvs_handle_t h;
  if (nvs_open("ionos", NVS_READONLY, &h) != ESP_OK) return;
  size_t len = n;
  if (nvs_get_str(h, key, out, &len) != ESP_OK) { /* keep the default */ }
  nvs_close(h);
}

static bool nvs_str_set(const char *key, const char *val)
{
  nvs_handle_t h;
  if (nvs_open("ionos", NVS_READWRITE, &h) != ESP_OK) return false;
  bool ok = nvs_set_str(h, key, val) == ESP_OK && nvs_commit(h) == ESP_OK;
  nvs_close(h);
  return ok;
}

void devid_init(void)
{
  snprintf(s_buildid, sizeof s_buildid, "%s|%s", FW_VERSION, FW_BUILD_UTC);
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(s_serial, sizeof s_serial, "ESP-%02X%02X%02X", mac[3], mac[4], mac[5]);
  nvs_str_get("serial", s_serial, sizeof s_serial);
  nvs_str_get("mfg", s_mfg, sizeof s_mfg);
  /* programming date belongs to THIS build only */
  char pv[96] = "";
  nvs_str_get("prog_ver", pv, sizeof pv);
  if (!strcmp(pv, s_buildid)) {
    nvs_str_get("prog", s_prog, sizeof s_prog);
    s_prog_done = true;
  }
  devid_print();
}

void devid_tick(bool wifi_up)
{
  if (s_prog_done) return;
  if (wifi_up && !s_ntp_started) {
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com");
    s_ntp_started = true;
  }
  if (!s_ntp_started) return;
  time_t t = time(NULL);
  struct tm lt;
  localtime_r(&t, &lt);
  if (lt.tm_year + 1900 < 2026) return;              /* not synced yet */
  strftime(s_prog, sizeof s_prog, "%Y-%m-%d %H:%M", &lt);
  if (nvs_str_set("prog", s_prog) && nvs_str_set("prog_ver", s_buildid))
    diag_stream().printf("DEVID: programming date of this build recorded: %s\n", s_prog);
  s_prog_done = true;
}

bool devid_cmd(const char *cmd)
{
  if (!strncmp(cmd, "SERIAL ", 7) && cmd[7]) {
    char v[24]; snprintf(v, sizeof v, "%s", cmd + 7);
    if (nvs_str_set("serial", v)) { snprintf(s_serial, sizeof s_serial, "%s", v); }
    devid_print();
    return true;
  }
  if (!strncmp(cmd, "MFG ", 4) && cmd[4]) {
    char v[16]; snprintf(v, sizeof v, "%s", cmd + 4);
    if (nvs_str_set("mfg", v)) { snprintf(s_mfg, sizeof s_mfg, "%s", v); }
    devid_print();
    return true;
  }
  if (!strcmp(cmd, "ID")) { devid_print(); return true; }
  return false;
}

const char *devid_serial(void) { return s_serial; }
const char *devid_fw(void)     { return FW_VERSION; }
const char *devid_build(void)  { return FW_BUILD_UTC; }
const char *devid_mfg(void)    { return s_mfg; }
const char *devid_prog(void)   { return s_prog; }

void devid_print(void)
{
  diag_stream().printf("DEVID: S/N %s | fw %s | built %s | mfg %s | programmed %s\n",
                       s_serial, FW_VERSION, FW_BUILD_UTC, s_mfg, s_prog);
}
