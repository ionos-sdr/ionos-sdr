/* aisport.c — PC harness for ais_rx.c (the SAME file the firmware compiles).
 *   build: gcc -O2 -o aisport aisport.c ais_rx.c -lm
 *   run:   ./aisport file.iq16 sps tune_khz [-swap] [-q]
 *   file.iq16 = raw interleaved int16 I,Q (iq2raw.py makes it from .npy/.wav)
 */
#include "ais_rx.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int quiet = 0;
static void on_frame(const uint8_t *fr, int len, char chan, uint32_t t, void *ctx)
{
  (void)ctx;
  if (quiet) return;
  char d[160], n[400];
  ais_describe(fr, len, d, sizeof d);
  ais_nmea(fr, len, chan, n, sizeof n);
  printf("  t=%7.2f s  %c  %s\n              %s\n", t / 48000.0, chan, d, n);
}

int main(int argc, char **argv)
{
  if (argc < 4) { fprintf(stderr, "usage: aisport file.iq16 sps tune_khz [-swap] [-q]\n"); return 2; }
  uint32_t sps = (uint32_t)atoi(argv[2]); double tune = atof(argv[3]);
  int swap = 0, fc = 0, mm = 0;
  for (int k = 4; k < argc; k++) {
    if (!strcmp(argv[k], "-swap")) swap = 1;
    if (!strcmp(argv[k], "-q")) quiet = 1;
    if (!strncmp(argv[k], "-fc", 3)) fc = atoi(argv[k] + 3);
    if (!strncmp(argv[k], "-mm", 3)) mm = atoi(argv[k] + 3) * 5;     /* bits -> samples */
  }
  static ais_rx_t A, B;
  int32_t offA = (int32_t)llround((161975.0 - tune) * 1000.0), offB = (int32_t)llround((162025.0 - tune) * 1000.0);
  int useA = abs(offA) + 8000 <= (int)sps / 2, useB = abs(offB) + 8000 <= (int)sps / 2;
  A.fc_hz = fc; A.mm_n = mm; B.fc_hz = fc; B.mm_n = mm;
  ais_rx_init(&A, sps, offA, 'A', on_frame, NULL); A.swap_iq = (uint8_t)swap;
  ais_rx_init(&B, sps, offB, 'B', on_frame, NULL); B.swap_iq = (uint8_t)swap;
  fprintf(stderr, "# sps %u tune %.3f MHz  A %+d Hz %s  B %+d Hz %s  FIR %d taps fc %d  mm %d bits\n", sps, tune / 1000,
          offA, useA ? "on" : "off", offB, useB ? "on" : "off", A.ch_n, (int)A.fc_hz, A.mm_n / 5);
  FILE *f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 1; }
  static int16_t buf[8192]; size_t n; uint64_t tot = 0;
  while ((n = fread(buf, sizeof(int16_t), 8192, f)) > 0) {
    if (useA) ais_rx_feed(&A, buf, (int)(n / 2));
    if (useB) ais_rx_feed(&B, buf, (int)(n / 2));
    tot += n / 2;
  }
  fclose(f);
  printf("frames: A %u  B %u   (crc_bad A %u B %u)  %.1f s\n", A.frames, B.frames, A.crc_bad, B.crc_bad, (double)tot / sps);
  return 0;
}
