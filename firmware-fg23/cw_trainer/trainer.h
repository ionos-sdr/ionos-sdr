/*
 * trainer.h  --  Koch-method character drill (pure logic, host-testable).
 *  Concept by: N7HPR   Design by: HA7DCD
 */
#ifndef TRAINER_H_INCLUDED
#define TRAINER_H_INCLUDED

#include <stdint.h>

/* The Koch teaching order (index 0 = first character learned). */
const char *trainer_koch_order(void);
uint8_t     trainer_koch_len(void);

void    trainer_init(void);
void    trainer_seed(uint32_t s);

/* Active lesson = the first 'level' characters of the Koch order.
 * level is clamped to [2, trainer_koch_len()]. */
void    trainer_set_lesson(uint8_t level);
uint8_t trainer_get_lesson(void);

/* A random character drawn from the active lesson set. */
char    trainer_next_char(void);

#endif /* TRAINER_H_INCLUDED */
