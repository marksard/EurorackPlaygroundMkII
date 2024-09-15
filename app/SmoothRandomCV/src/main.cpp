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
#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "../../commonlib/common/RandomFast.hpp"
#include "SmoothRandomCV.hpp"
#include "Oscillator.hpp"

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
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

//////////////////////////////////////////
// entries
static int16_t wave = 4;
static int16_t minFreq = 0;
static int16_t maxFreq = 6;
static int16_t curve = 3;
static int16_t level = 100;
static int16_t triggerMode = 0;

static Oscillator osc;
static EdgeChecker clockEdge;
static PollingTimeEvent pollingEvent;

static bool edgeHigh = false;
//////////////////////////////////////////

// 画面周り
#define MENU_MAX (6)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

typedef struct
{
    char title[12];
    SettingItem16 items[MENU_MAX];
} SettingMenu;

const char waveName[][5] = {"SQU", "DSAW", "USAW", "TRI", "SIN"};
const char triggerName[][5] = {"INT", "EXT"};
SettingMenu set[] = {
    {"SMOOTH RND",
     {
         SettingItem16(0, (int16_t)Oscillator::Wave::MAX, 1, &wave, "Wave: %s", waveName, (int16_t)Oscillator::Wave::MAX + 1),
         SettingItem16(0, 1, 1, &triggerMode, "TriggerMode: %s", triggerName, 2),
         SettingItem16(0, 133, 1, &minFreq, "MinFreq: %d", NULL, 0),
         SettingItem16(0, 133, 1, &maxFreq, "MaxFreq: %d", NULL, 0),
         SettingItem16(1, 64, 1, &curve, "Curve: %d", NULL, 0),
         SettingItem16(0, 100, 1, &level, "Level: %d", NULL, 0)
     }}};

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

    drawSetting(&u8g2, set[0].title, set[0].items, menuIndex, MENU_MAX, encMode);

    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    uint16_t valueA = osc.getWaveValue();
    pwm_set_gpio_level(OUT1, valueA);

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
    // pinMode(GATE, INPUT);
    clockEdge.init(GATE);
    pollingEvent.setBPM(133, 4);
    pollingEvent.start();

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    osc.init(SAMPLE_FREQ);
    osc.setWave(Oscillator::Wave::SQU);
    osc.setFreqName(100);
    osc.setFrequency(100);
    osc.setPhaseShift(0);
    osc.setFolding(0);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    // delay(500);
}

void loop()
{
    uint16_t potValue = pot.analogRead(true, true);
    int8_t encValue = enc.getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();
    uint16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    uint16_t cv2Value = cv2.analogReadDirectFast();

    static uint16_t lastPotValue = potValue;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        int menu = constrain(menuIndex + encValue, 0, MENU_MAX - 1);
        requiresUpdate |= menuIndex != menu ? 1 : 0;
        menuIndex = menu;
        encValue = 0;
    }

    // メニュー変更時のポットロック・ロック解除
    if (!unlock)
    {
        if (lastPotValue + 10 < potValue || lastPotValue - 10 > potValue)
        {
            unlock = 1;
        }
    }
    else
    {
        lastPotValue = potValue;
    }

    if (lastMenuIndex != menuIndex)
    {
        unlock = 0;
        requiresUpdate = 1;
    }
    lastMenuIndex = menuIndex;

    requiresUpdate |= set[0].items[menuIndex].add(encValue);
    osc.setWave((Oscillator::Wave)wave);

    static SmoothRandomCV src(ADC_RESO);
    src.setCurve(curve);
    src.setMaxFreq(maxFreq);
    src.setMaxLevel(level);
    edgeHigh = clockEdge.isEdgeHigh();
    src.update(edgeHigh, triggerMode);
    float lastFreq = src.getFreq();
    float lastLevel = src.getLevel();

    osc.setFrequency(max(lastFreq, minFreq == 0 ? 0.05 : minFreq));
    pwm_set_gpio_level(OUT2, lastLevel);
    gpio_put(LED1, clockEdge.getValue() ? HIGH : LOW);

    pollingEvent.setBPM(clockEdge.getBPM());
    gpio_put(OUT3, pollingEvent.ready() ? HIGH : LOW);
    static uint8_t dispCount = 0;
    dispCount++;
    if (dispCount == 0)
    {
        Serial.print(clockEdge.getBPM());
        Serial.print(", ");
        Serial.print(pollingEvent.getBPM());
        // Serial.print(", ");
        // Serial.print(cv2Value);
        Serial.println();
    }

    sleep_us(50); // 20kHz
    // sleep_ms(1);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
