/* Copy this file to secrets.h (git-ignored) and fill in your own values.
 * WARNING: these strings end up in plain text inside the compiled .bin. */
#pragma once

/* Soft-AP fallback when no known network is found. Min. 8-character password. */
#define WIFI_AP_SSID   "IONOS-SDR"
#define WIFI_AP_PASS   "changeme123"

/* Known station networks, tried in order. Leave unused slots empty. */
static const struct { const char *ssid; const char *pass; } WIFI_APS[] = {
  { "your-ssid",  "your-password" },
  { "",           ""              },
  { "",           ""              },
  { "",           ""              },
};
