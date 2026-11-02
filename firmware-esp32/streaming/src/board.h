/* SPDX-License-Identifier: MIT
 *
 * board.h - pin profiles of the ESP32-S3 side.
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * BOARD_IONOS_MINI = 1 : ionos-sdr-mini rev-B3 carrier (ESP32-S3 DevKitC clone
 *                        + BRD42xx radio board on the mezzanine + ER-TFTM024-3).
 *                        Pin map v2 (2026-10-09), bring-up verified 2026-11-02.
 * BOARD_IONOS_MINI = 0 : the bench wiring (DevKit + jumper wires to the WSTK EXP
 *                        header + ILI9341 breakout), as before.
 *
 * On the mini the FG23 link gets the SPI2/FSPI IO_MUX pins (10/11/12) and the
 * display moves to SPI3 - the "final layout" foreseen in main.cpp.
 */
#pragma once

#ifndef BOARD_IONOS_MINI
#define BOARD_IONOS_MINI 0
#endif

#if BOARD_IONOS_MINI
/* ---- FG23 link (mezzanine; series R1..R5 on the carrier) ---- */
#define BRD_FG23_SCLK      GPIO_NUM_12   /* FG23 PC05 */
#define BRD_FG23_MOSI      GPIO_NUM_11   /* FG23 PC00 */
#define BRD_FG23_CS        GPIO_NUM_10   /* FG23 PA07 */
#define BRD_FG23_SPI_HOST  SPI2_HOST     /* IO_MUX pins of FSPI */
#define BRD_RDY_PIN        14            /* -> FG23 PD02 */
#define BRD_CMDLINK_TX_PIN 13            /* -> FG23 PA06 (EUSART1 RX) */
/* ---- TFT (ER-TFTM024-3, ILI9341, 4-wire SPI) + resistive touch ---- */
#define BRD_TFT_HOST       SPI3_HOST
#define BRD_TFT_SCK        4
#define BRD_TFT_MOSI       5
#define BRD_TFT_MISO       6
#define BRD_TFT_CS         7
#define BRD_TFT_DC         15
#define BRD_TFT_RST        16
#define BRD_TFT_BL         17
#define BRD_TFT_TE         18
#define BRD_TFT_ROTATION   2             /* the panel sits upside-down on the cradle */
/* ---- buttons: two ADC1 ladders, 10 k pull-up, 0R / 2k2 / 6k8 / 22k ---- */
#define BRD_BTN_LADDER     1
#define BRD_BTN_A_PIN      1             /* BACK, OK, (SPARE) */
#define BRD_BTN_B_PIN      2             /* UP, DOWN, LEFT, RIGHT */
#else
#define BRD_FG23_SCLK      GPIO_NUM_15   /* FG23 PC05 / EXP 15 */
#define BRD_FG23_MOSI      GPIO_NUM_16   /* FG23 PC00 / EXP 10 */
#define BRD_FG23_CS        GPIO_NUM_17   /* FG23 PA07 / EXP 13 */
#define BRD_FG23_SPI_HOST  SPI3_HOST
#define BRD_RDY_PIN        2
#define BRD_CMDLINK_TX_PIN 18
#define BRD_TFT_HOST       SPI2_HOST
#define BRD_TFT_SCK        12
#define BRD_TFT_MOSI       11
#define BRD_TFT_MISO       13
#define BRD_TFT_CS         10
#define BRD_TFT_DC         14
#define BRD_TFT_RST        9
#define BRD_TFT_BL         21
#define BRD_TFT_TE         48
#define BRD_TFT_ROTATION   0
#define BRD_BTN_LADDER     0
#define BRD_BTN_PIN        0             /* Heltec V3 PRG button (GPIO0) */
#endif
