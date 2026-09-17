/*
 * app.c  --  FG23 CW Trainer: intro splash + Koch drill + real CW TX.
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  State machine driven from the super-loop:
 *      INTRO  -> splash with credits, PB0 starts
 *      RUN    -> sends a random Koch character, then reveals it
 *      PAUSE  -> PB0 short = resume, PB0 HELD = TX menu, PB1 = lesson +1
 *      TX     -> canned message on a keyed carrier, 70 cm CW segment
 *
 *  The drill NEVER transmits. The radio is only keyed from the TX screen,
 *  and only while a message is being sent (cwtx_set_enabled()).
 *
 *  Buttons use the Simple Button component (instances btn0, btn1).
 *  The keying LED uses the Simple LED component (instance led0).
 */
#include "app.h"
#include "oled.h"
#include "cw.h"
#include "tone.h"
#include "trainer.h"
#include "cwtx.h"

#include "sl_sleeptimer.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"

#define FW_VERSION      "v0.7"
#define LONG_PRESS_MS   700u
#define TX_WPM          18          /* on the air: no Farnsworth padding */

typedef enum { ST_INTRO, ST_RUN, ST_PAUSE, ST_TX } state_t;
static state_t s_state = ST_INTRO;

/* Button press events, latched in the change callback (interrupt context).
 * PB0 is decided on release: 1 = short press, 2 = held >= LONG_PRESS_MS. */
static volatile uint8_t  ev_btn0 = 0;
static volatile uint8_t  ev_btn1 = 0;
static volatile uint32_t s_btn0_tick = 0;

static const uint8_t EFF_STEPS[] = { 8, 10, 12, 15, 18 };
#define EFF_N (sizeof(EFF_STEPS)/sizeof(EFF_STEPS[0]))
static uint8_t s_eff_idx = 1;             /* -> 10 wpm */
static const uint8_t CHAR_WPM = 18;

/* ---- canned TX messages; the last menu slot is "back" ---- */
typedef struct { const char *label; const char *text; } tx_msg_t;
static const tx_msg_t TX_MSG[] = {
    { "TEST VVV", "VVV VVV DE " CWTX_MYCALL },
    { "CQ",       "CQ CQ CQ DE " CWTX_MYCALL " " CWTX_MYCALL " K" },
    { "BEACON",   "DE " CWTX_MYCALL " " CWTX_MYCALL " JN97" },
};
#define TX_MSG_N (sizeof(TX_MSG)/sizeof(TX_MSG[0]))
/* two extra menu slots after the messages */
#define TX_MENU_CAL   (TX_MSG_N)
#define TX_MENU_RAMP  (TX_MSG_N + 1u)
#define TX_MENU_BACK  (TX_MSG_N + 2u)
#define TX_MENU_N     (TX_MSG_N + 3u)
static uint8_t s_tx_idx = 0;

static const char *tx_menu_label(uint8_t i)
{
    if (i < TX_MSG_N)      return TX_MSG[i].label;
    if (i == TX_MENU_CAL)  return "CAL STEPS";
    if (i == TX_MENU_RAMP) return "CAL RAMP";
    return "< BACK";
}

/* ---- button callback (overrides the component's weak default) ---- */
void sl_button_on_change(const sl_button_t *handle)
{
    bool pressed = (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED);

    if (handle == &sl_button_btn0) {
        if (pressed) {
            s_btn0_tick = sl_sleeptimer_get_tick_count();
        } else {
            uint32_t held = sl_sleeptimer_tick_to_ms(
                    sl_sleeptimer_get_tick_count() - s_btn0_tick);
            ev_btn0 = (held >= LONG_PRESS_MS) ? 2u : 1u;
        }
    } else if (handle == &sl_button_btn1 && pressed) {
        ev_btn1 = 1;
    }
}

/* Key line: LED0 always, carrier only while the transmitter is armed. */
static void key_out(bool down)
{
    if (down) sl_led_turn_on(&sl_led_led0);
    else      sl_led_turn_off(&sl_led_led0);
    cwtx_key(down);
}

/* ---------------- tiny number formatting (no printf) ---------------- */
static int put_u32(char *d, int i, uint32_t v, uint8_t min_digits)
{
    char t[10];
    int n = 0;
    do { t[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v && n < 10);
    while (n < (int)min_digits) t[n++] = '0';
    while (n) d[i++] = t[--n];
    return i;
}

/* "432.050 MHz" */
static void fmt_freq(char *d)
{
    uint32_t hz = cwtx_freq_hz();
    int i = 0;
    i = put_u32(d, i, hz / 1000000u, 1);
    d[i++] = '.';
    i = put_u32(d, i, (hz % 1000000u) / 1000u, 3);
    d[i++] = ' '; d[i++] = 'M'; d[i++] = 'H'; d[i++] = 'z';
    d[i] = 0;
}

/* ---------------- screens ---------------- */
static void draw_intro(bool show_prompt)
{
    oled_clear();
    oled_text_center_scaled(0, "CW", 3);
    oled_text_center(23, "TRAINER  " FW_VERSION);
    oled_hline(8, 119, 34, true);
    oled_text_center(39, "CONCEPT  N7HPR");
    oled_text_center(48, "DESIGN   HA7DCD");
    if (show_prompt)
        oled_text_center(57, "PB0 = START");
    oled_flush();
}

static void draw_header(void)
{
    char line[22];
    uint8_t lvl = trainer_get_lesson();
    /* "L07  10/18WPM" */
    int i = 0;
    line[i++] = 'L';
    line[i++] = (char)('0' + (lvl / 10) % 10);
    line[i++] = (char)('0' + lvl % 10);
    line[i++] = ' '; line[i++] = ' ';
    line[i++] = (char)('0' + (cw_get_eff_wpm() / 10) % 10);
    line[i++] = (char)('0' + cw_get_eff_wpm() % 10);
    line[i++] = '/';
    line[i++] = (char)('0' + (cw_get_char_wpm() / 10) % 10);
    line[i++] = (char)('0' + cw_get_char_wpm() % 10);
    line[i++] = 'W'; line[i++] = 'P'; line[i++] = 'M';
    line[i] = 0;
    oled_text(0, 0, line);
    oled_hline(0, 127, 9, true);
}

static void draw_sending(void)
{
    oled_clear();
    draw_header();
    oled_big_char('?');          /* copy by ear first */
    oled_flush();
}

static void draw_reveal(char c)
{
    oled_clear();
    draw_header();
    oled_big_char(c);
    oled_flush();
}

static void draw_pause(void)
{
    char l[22];
    oled_clear();
    oled_text_center_scaled(0, "PAUSED", 2);
    oled_hline(0, 127, 18, true);

    uint8_t lvl = trainer_get_lesson();
    const char *k = trainer_koch_order();
    int i = 0;
    l[i++]='L';l[i++]='E';l[i++]='S';l[i++]='S';l[i++]='O';l[i++]='N';l[i++]=' ';
    l[i++]=(char)('0'+(lvl/10)%10); l[i++]=(char)('0'+lvl%10);
    l[i++]=' '; l[i++]='['; l[i++]=k[lvl-1]; l[i++]=']';
    l[i]=0;
    oled_text(0, 22, l);

    i = 0;
    l[i++]='S';l[i++]='P';l[i++]='E';l[i++]='E';l[i++]='D';l[i++]=' ';
    l[i++]=(char)('0'+(cw_get_eff_wpm()/10)%10); l[i++]=(char)('0'+cw_get_eff_wpm()%10);
    l[i++]='/'; l[i++]=(char)('0'+(CHAR_WPM/10)%10); l[i++]=(char)('0'+CHAR_WPM%10);
    l[i++]='W';l[i++]='P';l[i++]='M';
    l[i]=0;
    oled_text(0, 31, l);

    oled_text(0, 40, "PB0 = RUN");
    oled_text(0, 48, "PB1 = LESSON +");
    oled_text(0, 56, "HOLD PB0 = TX");
    oled_flush();
}

/* status == NULL -> show the key hints */
static void draw_tx(const char *status)
{
    char l[24];
    oled_clear();
    oled_text_center_scaled(0, "TX", 2);
    oled_hline(0, 127, 18, true);

    fmt_freq(l);
    oled_text_center(21, l);

    int i = 0;
    int32_t pw = (int32_t)CWTX_POWER_DDBM / 10;
    if (pw < 0) { l[i++] = '-'; pw = -pw; }
    i = put_u32(l, i, (uint32_t)pw, 1);
    l[i++]=' ';l[i++]='d';l[i++]='B';l[i++]='m';l[i++]=' ';l[i++]=' ';
    i = put_u32(l, i, (uint32_t)TX_WPM, 1);
    l[i++]=' ';l[i++]='W';l[i++]='P';l[i++]='M';
    l[i] = 0;
    oled_text_center(31, l);

    oled_text_center(43, tx_menu_label(s_tx_idx));
    oled_text_center(55, status ? status : "PB1 NEXT  PB0 GO");
    oled_flush();
}

/* ---------------- TX ---------------- */
static void tx_send(uint8_t idx)
{
    uint8_t save_char = cw_get_char_wpm();
    uint8_t save_eff  = cw_get_eff_wpm();

    if (!cwtx_ready() && !cwtx_init()) {
        draw_tx("NO RADIO");
        return;
    }

    draw_tx("SENDING...");
    cw_set_speed(TX_WPM, TX_WPM);       /* straight speed on the air */
    cw_set_key_comp_ms(CWTX_KEY_COMP_MS);   /* give back what the PA ramp eats */
    cwtx_set_enabled(true);
    cw_play_str(TX_MSG[idx].text);      /* blocking; LED + sidetone + carrier */
    cwtx_set_enabled(false);
    cwtx_idle();
    cw_set_key_comp_ms(0);
    cw_set_speed(save_char, save_eff);

    ev_btn0 = 0;                        /* swallow presses made while sending */
    ev_btn1 = 0;

    if (cwtx_last_error()) {
        char e[16];
        int i = 0;
        e[i++]='T';e[i++]='X';e[i++]=' ';e[i++]='E';e[i++]='R';e[i++]='R';e[i++]=' ';
        i = put_u32(e, i, (uint32_t)cwtx_last_error(), 1);
        e[i] = 0;
        draw_tx(e);
    } else {
        draw_tx("SENT");
    }
}

/* Diagnostic power staircase - see cwtx_cal_sweep(). ~6.4 s of carrier. */
static void tx_cal(void)
{
    if (!cwtx_ready() && !cwtx_init()) {
        draw_tx("NO RADIO");
        return;
    }
    draw_tx("CAL  6s...");
    cwtx_set_enabled(true);
    cwtx_cal_sweep();
    cwtx_set_enabled(false);
    cwtx_idle();
    ev_btn0 = 0;
    ev_btn1 = 0;
    draw_tx("CAL DONE");
}

/* PA ramp-time sweep - see cwtx_ramp_sweep(). ~11 s of carrier. */
static void tx_ramp(void)
{
    if (!cwtx_ready() && !cwtx_init()) {
        draw_tx("NO RADIO");
        return;
    }
    draw_tx("RAMP 11s...");
    cwtx_set_enabled(true);
    cwtx_ramp_sweep();
    cwtx_set_enabled(false);
    cwtx_idle();
    ev_btn0 = 0;
    ev_btn1 = 0;
    draw_tx("RAMP DONE");
}

/* ---------------- lifecycle ---------------- */
void app_init(void)
{
    tone_init();
    trainer_init();
    cw_set_key_cb(key_out);
    cw_set_speed(CHAR_WPM, EFF_STEPS[s_eff_idx]);

    /* Grab the RAIL handle now; harmless if the radio is not up yet, the
     * TX screen retries. Nothing is keyed until cwtx_set_enabled(true). */
    (void)cwtx_init();

    if (!oled_init()) {
        /* No panel ACK: keep going, the audio drill still works blind. */
    }
    draw_intro(true);
    s_state = ST_INTRO;
}

void app_process_action(void)
{
    switch (s_state) {

    case ST_INTRO: {
        static bool blink = false;
        draw_intro(blink);
        blink = !blink;
        sl_sleeptimer_delay_millisecond(400);
        if (ev_btn0) {
            ev_btn0 = 0;
            trainer_seed((uint32_t)sl_sleeptimer_get_tick_count() | 1u);
            s_state = ST_RUN;
        }
        ev_btn1 = 0;
        break;
    }

    case ST_RUN: {
        char c = trainer_next_char();
        draw_sending();
        cw_play_char(c);           /* blocks ~one character; audible */
        draw_reveal(c);
        /* inter-character dwell so the answer is readable */
        for (int i = 0; i < 8 && !ev_btn0 && !ev_btn1; i++)
            sl_sleeptimer_delay_millisecond(100);

        if (ev_btn0) { ev_btn0 = 0; s_state = ST_PAUSE; draw_pause(); }
        if (ev_btn1) {
            ev_btn1 = 0;
            s_eff_idx = (uint8_t)((s_eff_idx + 1) % EFF_N);
            cw_set_speed(CHAR_WPM, EFF_STEPS[s_eff_idx]);
        }
        break;
    }

    case ST_PAUSE: {
        sl_sleeptimer_delay_millisecond(120);
        if (ev_btn0 == 2u) {            /* held -> TX menu */
            ev_btn0 = 0;
            s_tx_idx = 0;
            s_state = ST_TX;
            draw_tx(NULL);
            break;
        }
        if (ev_btn0) { ev_btn0 = 0; s_state = ST_RUN; break; }
        if (ev_btn1) {
            ev_btn1 = 0;
            uint8_t lvl = trainer_get_lesson();
            lvl = (lvl >= trainer_koch_len()) ? 2 : (uint8_t)(lvl + 1);
            trainer_set_lesson(lvl);
            draw_pause();
        }
        break;
    }

    case ST_TX: {
        sl_sleeptimer_delay_millisecond(120);
        if (ev_btn1) {
            ev_btn1 = 0;
            s_tx_idx = (uint8_t)((s_tx_idx + 1u) % TX_MENU_N);
            draw_tx(NULL);
        }
        if (ev_btn0) {
            ev_btn0 = 0;
            if (s_tx_idx == TX_MENU_BACK) {
                s_state = ST_PAUSE;
                draw_pause();
            } else if (s_tx_idx == TX_MENU_CAL) {
                tx_cal();
            } else if (s_tx_idx == TX_MENU_RAMP) {
                tx_ramp();
            } else {
                tx_send(s_tx_idx);
            }
        }
        break;
    }
    }
}
