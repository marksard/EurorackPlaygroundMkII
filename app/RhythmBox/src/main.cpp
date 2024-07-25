/*!
 * Rhythm Box
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/TriggerOut.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "SingleShotWave.hpp"

// #include "wavetable/909_01/909_BD_Norm.h"
// #include "wavetable/909_01/909_OH.h"
// #include "wavetable/909_01/909_RS.h"
// #include "wavetable/909_01/909_SD_Snap.h"

// SingleShotWave BD(buf_909_BD_Norm, buf_size_909_BD_Norm);
// SingleShotWave OH(buf_909_OH, buf_size_909_OH);
// SingleShotWave RS(buf_909_RS, buf_size_909_RS);
// SingleShotWave SD(buf_909_SD_Snap, buf_size_909_SD_Snap);

// #include "wavetable/Disco/Disco_BD_01.h"
// #include "wavetable/Disco/Disco_OHH_01.h"
// #include "wavetable/Disco/Disco_Rim_01.h"
// #include "wavetable/Disco/Disco_SD_01.h"

// SingleShotWave BD(buf_Disco_BD_01, buf_size_Disco_BD_01);
// SingleShotWave OH(buf_Disco_OHH_01, buf_size_Disco_OHH_01);
// SingleShotWave RS(buf_Disco_Rim_01, buf_size_Disco_Rim_01);
// SingleShotWave SD(buf_Disco_SD_01, buf_size_Disco_SD_01);

#include "wavetable/808_01/808_BD_02.h"
#include "wavetable/808_01/808_OHH_01.h"
#include "wavetable/808_01/808_Rim_01.h"
#include "wavetable/808_01/808_SD_01.h"

SingleShotWave BD(buf_808_BD_02, buf_size_808_BD_02);
SingleShotWave OH(buf_808_OHH_01, buf_size_808_OHH_01);
SingleShotWave RS(buf_808_Rim_01, buf_size_808_Rim_01);
SingleShotWave SD(buf_808_SD_01, buf_size_808_SD_01);

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 2048         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
#define SAMPLE_FREQ 44100
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// setting values
float bdPitch = 1.0;
float bdDecay = 1.0;
float sdPitch = 1.0;
float sdDecay = 1.0;
float hhPitch = 1.0;
float hhDecay = 0.8;
float rmPitch = 1.0;
float rmDecay = 1.0;

// 画面周り
#define MENU_MAX (8)
static int16_t menuIndex = 0;
static uint8_t requiresUpdate = 1;

typedef struct
{
    char title[12];
    SettingItemF items[MENU_MAX];
} SettingMenu;

SettingMenu set = {
    "RHYTHM BOX",
    {
        SettingItemF(0.1, 2.0, 0.01, &bdPitch, "%cBD Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &bdDecay, "%cBD Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &sdPitch, "%cSD Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &sdDecay, "%cSD Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &hhPitch, "%cHH Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &hhDecay, "%cHH Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &rmPitch, "%cRM Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &rmDecay, "%cRM Decay: %4.2f", NULL, 0),
    }};

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

inline uint8_t updateMenuIndex(uint8_t btn0, uint8_t btn1)
{
    if (btn0 == 2)
    {
        menuIndex = constrain(menuIndex - 1, 0, MENU_MAX - 1);
        return 1;
    }
    if (btn1 == 2)
    {
        menuIndex = constrain(menuIndex + 1, 0, MENU_MAX - 1);
        return 1;
    }

    return 0;
}

void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
}

void dispOLED()
{
    if (!requiresUpdate)
    {
        return;
    }

    // gpio_put(LED1, HIGH);
    requiresUpdate = 0;
    u8g2.clearBuffer();
    drawSetting(&u8g2, set.title, set.items, menuIndex, MENU_MAX);
    u8g2.sendBuffer();
    // gpio_put(LED1, LOW);
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    pwm_set_gpio_level(OUT1, BD.updateWave());
    pwm_set_gpio_level(OUT2, SD.updateWave());
    pwm_set_gpio_level(OUT3, OH.updateWave());
    pwm_set_gpio_level(OUT4, RS.updateWave());

    // gpio_put(LED1, LOW);
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    analogReadResolution(12);

    pot.init(POT1);
    enc.init(EC1A, EC1B);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    pinMode(GATE, INPUT);

    BD.init();
    SD.init();
    OH.init();
    RS.init();

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    // initPWM(OUT5, PWM_RESO);
    // initPWM(OUT6, PWM_RESO);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void loop()
{
    uint16_t voct = vOct.analogReadDirect();
    int16_t cv1Value = cv1.analogReadDirect();
    uint16_t cv2Value = cv2.analogReadDirect();
    uint8_t gateValue = digitalReadFast(GATE) ? 1 : 0;

    BD.setSpeed(bdPitch);
    BD.updateDecay(bdDecay);
    SD.setSpeed(sdPitch);
    SD.updateDecay(sdDecay);
    OH.setSpeed(hhPitch);
    OH.updateDecay(hhDecay);
    RS.setSpeed(rmPitch);
    RS.updateDecay(rmDecay);

    // 閾値は適当
    BD.play(gateValue);
    SD.play(voct > 2048);
    OH.play(cv1Value > 3000);
    RS.play(cv2Value > 3000);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    // Serial.print(voct);
    // Serial.print(", ");
    // Serial.print(cv1Value);
    // Serial.print(", ");
    // Serial.print(cv2Value);
    // Serial.print(", ");
    // Serial.print(gateValue);
    // Serial.println();
    // }
}

void setup1()
{
    initOLED();
}

void loop1()
{
    uint16_t potValue = pot.analogReadDropLow4bit();
    int8_t encValue = enc.getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    requiresUpdate |= updateMenuIndex(btn0, btn1);
    requiresUpdate |= set.items[menuIndex].add(encValue);

    dispOLED();
    sleep_ms(1);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    // Serial.print(potValue);
    // Serial.print(", ");
    // Serial.print(encValue);
    // Serial.print(", ");
    // Serial.print(btn0);
    // Serial.print(", ");
    // Serial.print(btn1);
    // Serial.print(", ");
    // Serial.print(btn2);
    // Serial.print(", ");
    // Serial.print(menuIndex);
    // Serial.println();
    // }
}
