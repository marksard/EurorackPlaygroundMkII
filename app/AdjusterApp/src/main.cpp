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
#include "../../commonlib/common/pwm_wrapper.h"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 4096         // 12bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define SAMPLE_FREQ 44100
static uint interruptSliceNum;

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

//////////////////////////////////////////

// 画面周り
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

const char testName[][5] = {"INT", "EXT"};

SettingItem16 settings[] =
{
    SettingItem16(0, 4095, 1, &potValue, "pot: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &btn1Value, "btn1: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &btn2Value, "btn2: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &vOctValue, "voct: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &gateValue, "gate: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &cv1Value, "cv1: %d", NULL, 0),
    SettingItem16(0, 4095, 1, &cv2Value, "cv2: %d", NULL, 0),
    SettingItem16(0, 1, 1, &bias, "bias: %d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"TEST MODULE", settings, sizeof(settings) / sizeof(settings[0])},
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
    enc.init(EC1A, EC1B);
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
    vOctValue = vOct.analogReadDirectFast();
    gateValue = gate.isEdgeHigh();
    cv1Value = cv1.analogReadDirectFast();
    cv2Value = cv2.analogReadDirectFast();

    int16_t value = bias ? 2047 : 4095;
    pwm_set_gpio_level(OUT1, value);
    pwm_set_gpio_level(OUT2, value);
    pwm_set_gpio_level(OUT3, value);
    pwm_set_gpio_level(OUT4, value);
    pwm_set_gpio_level(OUT5, value);
    pwm_set_gpio_level(OUT6, value);

    gpio_put(LED1, gateValue ? HIGH : LOW);
    gpio_put(LED2, vOctValue > 4094 ? HIGH : LOW);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(gate.getBPM());
    //     Serial.print(", ");
    //     Serial.print(vOctValue);
    //     Serial.print(", ");
    //     Serial.print(cv2Value);
    //     Serial.println();
    // }

    // sleep_us(50); // 20kHz
    sleep_ms(1);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(100);
    updateOLED.start();
}

void loop1()
{
    btn1Value = buttons[0].getState();
    btn2Value = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();
    potValue = pot.analogRead(true, true);
    bool acc = encMode ? true : false;
    int8_t encValue = enc.getDirection(acc);

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

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
