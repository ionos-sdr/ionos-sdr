/*
 * trainer.c  --  Koch-method character drill (pure logic).
 *  Concept by: N7HPR   Design by: HA7DCD
 */
#include "trainer.h"
#include <string.h>

/* Standard Koch teaching order. */
static const char KOCH[] = "KMRSUAPTLOWI.NJEF0Y,VG5/Q9ZH38B?427C1D6X";

const char *trainer_koch_order(void) { return KOCH; }
uint8_t     trainer_koch_len(void)   { return (uint8_t)(sizeof(KOCH) - 1); }

static uint8_t  s_level = 2;
static uint32_t s_lfsr  = 0xACE1u;   /* non-zero seed */

void trainer_seed(uint32_t s) { if (s) s_lfsr = s; }

void trainer_init(void)
{
    s_level = 2;
    s_lfsr  = 0xACE1u;
}

void trainer_set_lesson(uint8_t level)
{
    uint8_t n = trainer_koch_len();
    if (level < 2) level = 2;
    if (level > n) level = n;
    s_level = level;
}
uint8_t trainer_get_lesson(void) { return s_level; }

/* 32-bit xorshift for a light, self-contained PRNG. */
static uint32_t rng_next(void)
{
    uint32_t x = s_lfsr;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    s_lfsr = x;
    return x;
}

char trainer_next_char(void)
{
    uint8_t idx = (uint8_t)(rng_next() % s_level);
    return KOCH[idx];
}
