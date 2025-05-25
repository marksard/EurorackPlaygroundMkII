/*!
 * Clock Divider
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

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 2048         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 10)
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// clock divider
static int16_t bpm = 0;
static uint8_t clockGate = 0;
static int16_t clockCount = 0;

// setting values
static EdgeChecker clockEdge;
static TriggerOut triggerOuts[6];
static int16_t trigMode = 1;
static int16_t trigDuration = 5;
static int16_t resetCount = 32;
static int16_t divCount[6] = {1, 2, 4, 8, 3, 5};
static int16_t backBeats[6] = {0};

// 画面周り
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;

const char trigModeName[][5] = {"RAW", "TRIG"};
const char offon[][5] = {"OFF", "ON"};

SettingItem16 commonSettings[] =
{
    SettingItem16(0, 32767, 1, &bpm, "BPM: %d", NULL, 0),
    SettingItem16(0, 1, 1, &trigMode, "Mode: %s", trigModeName, 2),
    SettingItem16(5, 100, 5, &trigDuration, "TrigDuration: %d", NULL, 0),
    SettingItem16(1, 128, 1, &resetCount, "ResetCount: %d", NULL, 0),
};

SettingItem16 outSettings[] =
{
    SettingItem16(1, 256, 1, &divCount[0], "OUT1: /%d", NULL, 0),
    SettingItem16(1, 256, 1, &divCount[1], "OUT2: /%d", NULL, 0),
    SettingItem16(1, 256, 1, &divCount[2], "OUT3: /%d", NULL, 0),
    SettingItem16(1, 256, 1, &divCount[3], "OUT4: /%d", NULL, 0),
    SettingItem16(1, 256, 1, &divCount[4], "OUT5: /%d", NULL, 0),
    SettingItem16(1, 256, 1, &divCount[5], "OUT6: /%d", NULL, 0),
};

SettingItem16 backbeatSettings[] =
{
    SettingItem16(0, 1, 1, &backBeats[0], "OUT1 BB: %s", offon, 2),
    SettingItem16(0, 1, 1, &backBeats[1], "OUT2 BB: %s", offon, 2),
    SettingItem16(0, 1, 1, &backBeats[2], "OUT3 BB: %s", offon, 2),
    SettingItem16(0, 1, 1, &backBeats[3], "OUT4 BB: %s", offon, 2),
    SettingItem16(0, 1, 1, &backBeats[4], "OUT5 BB: %s", offon, 2),
    SettingItem16(0, 1, 1, &backBeats[5], "OUT6 BB: %s", offon, 2),
};

static MenuSection16 menu[] = {
    {"COMMON", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
    {"OUTPUT", outSettings, sizeof(outSettings) / sizeof(outSettings[0])},
    {"BACKBEAT", backbeatSettings, sizeof(backbeatSettings) / sizeof(backbeatSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

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

    requiresUpdate = 0;
    u8g2.clearBuffer();
    menuControl.draw(&u8g2, encMode);
    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    bool trig = clockEdge.isEdgeHigh();
    clockGate = clockEdge.getValue();
    if (trig)
    {
        clockCount++;
        clockCount = clockCount % resetCount; // % 0x7FFF
    }

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
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    clockEdge.init(GATE);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    triggerOuts[0].init(OUT1);
    triggerOuts[1].init(OUT2);
    triggerOuts[2].init(OUT3);
    triggerOuts[3].init(OUT4);
    triggerOuts[4].init(OUT5);
    triggerOuts[5].init(OUT6);
    triggerOuts[0].setDuration(5);
    triggerOuts[1].setDuration(5);
    triggerOuts[2].setDuration(5);
    triggerOuts[3].setDuration(5);
    triggerOuts[4].setDuration(5);
    triggerOuts[5].setDuration(5);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void loop()
{
    enc.getDirection();
    uint16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    uint16_t cv2Value = cv2.analogReadDirectFast();
    // uint8_t gateValue = digitalReadFast(GATE) ? 1 : 0;

    static int16_t lastBpm = 0;
    bpm = clockEdge.getBPM();
    requiresUpdate |= bpm != lastBpm ? 1 : 0;
    lastBpm = bpm;

    int duration = clockEdge.getDurationMills();
    duration = map(trigDuration, 0, 100, 0, duration);

    for (int i = 0; i < 6; ++i)
    {
        triggerOuts[i].setDuration(duration);

        int16_t divCountAdd = divCount[i];

        int div = clockGate;
        if (backBeats[i])
            // 裏拍モード
            div = (clockCount % divCountAdd) >= (divCountAdd >> 1);
        else
            // 通常
            div = (clockCount % divCountAdd) < (divCountAdd >> 1);
        int state = divCountAdd == 1 ? clockGate : div;
        int out = triggerOuts[i].getTriggerGate(state, trigMode);
        triggerOuts[i].set(out);
    }

    // カウンタリセット時点灯
    gpio_put(LED1, clockCount == 0 ? HIGH : LOW);

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

    sleep_us(100);
}

void setup1()
{
    initOLED();
}

void loop1()
{
    // uint16_t potValue = pot.analogReadDropLow4bit();
    // int8_t encValue = enc.getDirection();
    int8_t encValue = enc.getValue();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    // requiresUpdate |= updateMenuIndex(btn0, btn1);
    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        requiresUpdate |= menuControl.select(encValue);
        encValue = 0;
    }

    requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);

    if (btn0 == 1)
    {
        clockCount = resetCount - 1;
    }

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
    // Serial.println();
    // }
}
