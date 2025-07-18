/*!
 * Eurorack Playground Mk II GPIO Mapping
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#define PWM_INTR_PIN D25 // PMW4 B

#define EC1B D0
#define EC1A D2
#define BTN1 D6
#define BTN2 D3
#define BTN3 D1 // Encoder Button
#define GATE D7
#define LED1 D8 // Button1 LED (TX1)
#define LED2 D9 // Button2 LED (RX1)
#define OUT1 D12 // SPI MISO(RX)PWM6 A
#define OUT2 D13 // SPI CS      PMW6 B
#define OUT3 D14 // SPI SCK     PMW7 A
#define OUT4 D15 // SPI MOSI(TX)PMW7 B
#define OUT5 D11 //             PWM5 B
#define OUT6 D10 //             PWM5 A
#define POT1 A0
#define VOCT A1
#define CV1 A2
#define CV2 A3
