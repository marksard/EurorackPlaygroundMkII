/*!
 * SmoothRandomCV
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
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/epmkii_basicconfig.h"
#include "../../commonlib/common/pwm_wrapper.h"

#define PWM_RESO 4096         // 12bit
#define SAMPLE_FREQ 44100
static uint interruptSliceNum;

static const float semi2DacRatio = (PWM_RESO - 1) / 60.0;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static EdgeChecker gate;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

//////////////////////////////////////////
// entries
static int16_t vOctValue = 0;
static int16_t gateValue = 0;
static int16_t potValue = 0;
static int16_t cv1Value = 0;
static int16_t cv2Value = 0;
static int16_t btn1Value = 0;
static int16_t btn2Value = 0;
static int16_t bias = 0;
static int16_t outputValue = 0;
static int16_t outputSemiSelect = 0;

//////////////////////////////////////////

// 画面周り
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

const char biasONOFF[][5] = {"OFF", "ON"};

SettingItem16 settings[] =
{
    SettingItem16(0, 4095, 1, &btn1Value, "chk btnA: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &btn2Value, "chk btnB: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &potValue, "chk pot: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &vOctValue, "chk voct: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &gateValue, "chk gate: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &cv1Value, "chk cv1: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &cv2Value, "chk cv2: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &outputValue, "chk out: %d", NULL, 0),
    SettingItem16(0, 60, 1, &outputSemiSelect, "sel semi: %d", NULL, 0),
    SettingItem16(0, 1, 1, &bias, "sel bias: %s", biasONOFF, 2),
};

static MenuSection16 menu[] = {
    {"ADJUSTMENT", settings, sizeof(settings) / sizeof(settings[0])},
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
    static char disp_buf[33] = {0};

    if (!requiresUpdate)
        return;

    requiresUpdate = 0;
    u8g2.clearBuffer();

    menuControl.draw(&u8g2, encMode);

    u8g2.sendBuffer();
}

void checkVOct()
{
    // voct誤差表示用
    static int16_t lastVOctValue = 0;
    static bool flag = false;
    static int8_t count = 0;
    static int32_t vOctMean = 0;
    vOctValue = vOctValue - VOCTInputErrorLUT[vOctValue];
    if (vOctValue > lastVOctValue + 30 || vOctValue < lastVOctValue - 30)
    {
        flag = true;
    }
    lastVOctValue = vOctValue;
    if (flag)
    {
        vOctMean += vOctValue;
        count++;
        if (count >= 16)
        {
            vOctMean = vOctMean >> 4;
            Serial.print(vOctMean);
            Serial.println("");
            flag = false;
            count = 0;
            vOctMean = 0;
            // outputSemiSelect++;
            // if (outputSemiSelect > 60)
            // {
            //     outputSemiSelect = 0;
            // }
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

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
    gate.init(GATE);
    cv1.init(CV1);
    cv2.init(CV2);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    // delay(500);
}

void loop()
{
    enc.getDirection();
    vOctValue = vOct.analogReadDirectFast();
    gateValue = gate.isEdgeHigh();
    cv1Value = cv1.analogReadDirectFast();
    cv2Value = cv2.analogReadDirectFast();
    potValue = pot.analogRead(false);
    // potValue = pot.analogReadDirectFast();
    // potValue = pot.analogReadDropLow4bit();

    outputValue = bias ? ((PWM_RESO >> 1) - 1) : constrain(((float)outputSemiSelect * semi2DacRatio), 0, PWM_RESO - 1);
    outputValue = constrain(outputValue - PWMCVDCOutputErrorLUT[outputSemiSelect], 0, PWM_RESO - 1);
    pwm_set_gpio_level(OUT1, outputValue);
    pwm_set_gpio_level(OUT2, outputValue);
    pwm_set_gpio_level(OUT3, outputValue);
    pwm_set_gpio_level(OUT4, outputValue);
    pwm_set_gpio_level(OUT5, outputValue);
    pwm_set_gpio_level(OUT6, outputValue);

    gpio_put(LED1, gateValue ? HIGH : LOW);
    gpio_put(LED2, vOctValue > PWM_RESO - 2 ? HIGH : LOW);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print("pot:");
    //     Serial.print(potValue);
    //     Serial.print(" voct:");
    //     Serial.print(vOctValue);
    //     Serial.print(" cv1:");
    //     Serial.print(cv1Value);
    //     Serial.print(" cv2:");
    //     Serial.print(cv2Value);
    //     Serial.println();
    // }

    sleep_ms(1);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    int8_t encValue = enc.getValue();
    btn1Value = buttons[0].getState();
    btn2Value = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    if (btn1Value > 0 || btn2Value > 0)
    {
        requiresUpdate |= 1;
    }

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
    requiresUpdate = 1;
    checkVOct();

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
