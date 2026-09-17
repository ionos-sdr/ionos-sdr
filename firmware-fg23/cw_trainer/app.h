/*
 * app.h  --  FG23 CW Trainer application entry points.
 *  Concept by: N7HPR   Design by: HA7DCD
 */
#ifndef APP_H_INCLUDED
#define APP_H_INCLUDED

/* Called once after sl_system_init(). */
void app_init(void);

/* Called repeatedly from the main super-loop. */
void app_process_action(void);

#endif /* APP_H_INCLUDED */
