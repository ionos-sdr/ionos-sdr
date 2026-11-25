/* aisport3.c — PC harness for ais_multi (DDC + hypotheses, A and B): ./aisport3 f.iq16 sps tune_khz [-hyp3] [-step2000] [-v] */
#include "ais_rx.h"
#include <stdlib.h>
#include <string.h>
static int verbose = 0; static uint32_t nA, nB;
static void cb(const uint8_t *fr, int len, char chan, uint32_t t, void *ctx)
{
  ais_multi_t *m = (ais_multi_t *)ctx;
  if (chan == 'A') nA++; else nB++;
  if (verbose) printf("t=%.3f %c foff %+.0f lvl %.1f snr %.1f\n", t / 48000.0, chan, m->last->last_foff_hz, m->last->last_lvl_dbfs, m->last->last_snr_db);
  (void)fr; (void)len;
}
int main(int argc, char **argv)
{
  uint32_t sps = (uint32_t)atoi(argv[2]); uint32_t tune = (uint32_t)atoi(argv[3]);
  int H = 3, step = 2000;
  for (int k = 4; k < argc; k++) {
    if (!strncmp(argv[k], "-hyp", 4)) H = atoi(argv[k] + 4);
    if (!strncmp(argv[k], "-step", 5)) step = atoi(argv[k] + 5);
    if (!strcmp(argv[k], "-v")) verbose = 1;
  }
  static ais_multi_t M;
  ais_multi_init(&M, sps, tune, H, step, cb, &M, malloc);
  fprintf(stderr, "# %u ch, ddc %u sps, D %d, taps %d\n", M.nch, M.sps_ddc, M.ch[0].D, M.ch[0].n);
  FILE *f = fopen(argv[1], "rb");
  static int16_t buf[4096]; size_t n;
  while ((n = fread(buf, sizeof(int16_t), 4096, f)) > 0) ais_multi_feed(&M, buf, (int)(n / 2));
  fclose(f);
  printf("frames: A %u  B %u\n", nA, nB);
  return 0;
}
