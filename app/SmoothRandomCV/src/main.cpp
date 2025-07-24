/*!
 * SmoothRandomCV
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include "lib/Button.hpp"
#include "lib/SmoothAnalogRead.hpp"
#include "lib/RotaryEncoder.hpp"
#include "lib/ADCErrorCorrection.hpp"
#include "lib/EEPROMConfigIO.hpp"
#include "ui_common/SettingItem.hpp"
#include "lib/pwm_wrapper.h"
#include "gpio_mapping.h"
#include "basic_definition.h"

#include "lib/PollingTimeEvent.hpp"
#include "lib/TriggerOut.hpp"
#include "lib/EdgeChecker.hpp"
#include "lib/RandomFast.hpp"

#include "MultiWaveOscEx.hpp"
#include "SmoothRandomCV.hpp"
#include "OscilloscopeLite.hpp"

// #undef SAMPLE_FREQ
// #define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8) // 32470.703125khz
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;
static ADCErrorCorrection adcErrorCorrection(3.3);

//////////////////////////////////////////
// entries
static float potValue = 0;
static float outputSemiSelect = 0;
static float clockMode = 0;
static float level = 100;
static float curve = 3;
static float wave = 4;
static float minFreq = 0;
static float maxFreq = 3;
static float minMultiply = 0;
static float maxMultiply = 3;

// static int16_t clockPPQ = 4;
static uint8_t selReso[] = { 4, 8, 12, 16 };

static MultiWaveOscEx lfo;
static EdgeChecker clockEdge;
static PollingTimeEvent rndMultiPollingEvent;
static PollingTimeEvent multiPollingEvent;
static SmoothRandomCV smoothRand(PWM_RESO);
static TriggerOut rndMultiOut;
static TriggerOut multiOut;
static RandomFast randFast;
//////////////////////////////////////////

// 画面周り
#define MENU_MAX (1 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

static uint8_t oscDataIndex = 0;
static bool oscBias = 0;
const char oscDataNames[][5] = {"VOCT", "CV1 ", "CV2 "};
static OscilloscopeLite oscillo(SAMPLE_FREQ);
static int16_t vOctValue = 0;
static int16_t cv1Value = 0;
static int16_t cv2Value = 0;
static const float voltPerTone = (PWM_RESO - 1) / 60.0;


const char waveName[][5] = {"SQU", "DSAW", "USAW", "TRI", "SIN"};
const char triggerName[][5] = {"INT", "EXT"};
const char selMultiplyName[][5] = { "x1", "x2", "x3", "x4" };

SettingItemF directOutSetting[] =
{
    SettingItemF(0.0, 60.0, 1.0, &outputSemiSelect, "Semi: %3.0f", NULL, 0),
    SettingItemF(0.0, 4095.0, 1.0, &potValue, "Vol: %6.0f", NULL, 0),
};

SettingItemF rndSettings[] =
{
    SettingItemF(0.0, 1.0, 1.0, &clockMode, "CLK Mode: %s", triggerName, 2),
    SettingItemF(0.0, 100.0, 1.0, &level, "CV Level: %3.0f", NULL, 0),
    SettingItemF(1.0, 50.0, 1.0, &curve, "CV Curve: %3.0f", NULL, 0),
    SettingItemF(0.0, (float)MultiWaveOscEx::Wave::MAX, 1.0, &wave, "LFO Wav: %s", waveName, (int16_t)MultiWaveOscEx::Wave::MAX + 1),
    SettingItemF(0.0, 20.0, 0.1, &minFreq, "LFO MinF:%3.1f", NULL, 0),
    SettingItemF(0.0, 20.0, 0.1, &maxFreq, "LFO MaxF:%3.1f", NULL, 0),
};

SettingItemF clkSettings[] =
{
    SettingItemF(0.0, 3.0, 1.0, &minMultiply, "MUL Min: %s", selMultiplyName, 4),
    SettingItemF(0.0, 3.0, 1.0, &maxMultiply, "MUL Max: %s", selMultiplyName, 4),
};

static MenuSectionF menu[] = {
    {"OUT MODULE", directOutSetting, sizeof(directOutSetting) / sizeof(directOutSetting[0])},
    {"RND MODULE", rndSettings, sizeof(rndSettings) / sizeof(rndSettings[0])},
    {"CLK MODULE", clkSettings, sizeof(clkSettings) / sizeof(clkSettings[0])}
};

static MenuControlF menuControl(menu, sizeof(menu) / sizeof(menu[0]));

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
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
    static char disp_buf[33] = {0};

    if (!requiresUpdate)
        return;

    requiresUpdate = 0;
    u8g2.clearBuffer();

    switch (menuIndex)
    {
    case 0:
        oscillo.draw(encMode, (char *)(oscDataNames[oscDataIndex]), oscBias);
        break;
    default:
        menuControl.draw(&u8g2, encMode);
        break;
    }

    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    uint16_t valueA = lfo.getWaveValue();
    pwm_set_gpio_level(OUT1, valueA);

    switch (oscDataIndex)
    {
    case 0:
        oscillo.write(vOctValue);
        break;
    case 1:
        oscillo.write(cv1Value);
        break;
    case 2:
        oscillo.write(cv2Value);
        break;
    default:
        break;
    }

    // gpio_put(LED1, LOW);
}

void setup()
{
    analogReadResolution(ADC_BIT);
    pinMode(23, OUTPUT);
    gpio_put(23, HIGH);

    pot.init(POT1);
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    clockEdge.init(GATE);
    multiPollingEvent.setBPM(133, 4);
    multiPollingEvent.start();
    rndMultiPollingEvent.setBPM(133, 4);
    rndMultiPollingEvent.start();
    multiOut.init(OUT5);
    rndMultiOut.init(OUT6);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    // initPWM(OUT5, PWM_RESO);
    // initPWM(OUT6, PWM_RESO);

    adcErrorCorrection.init(3.3, 20.0);

    lfo.init(SAMPLE_FREQ);
    lfo.setWave(MultiWaveOscEx::Wave::SQU);
    lfo.setFreqName(100);
    lfo.setFrequency(100);
    lfo.setPhaseShift(0);
    lfo.setFolding(0);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    pot.analogReadDirectFast();
    bool acc = encMode ? true : false;
    enc.getDirection(acc);
    vOctValue = vOct.analogReadDirectFast();
    cv1Value = cv1.analogReadDirectFast();
    cv2Value = cv2.analogReadDirectFast();
    
    // Smooth Random
    smoothRand.setCurve(curve);
    smoothRand.setMaxFreq(maxFreq);
    smoothRand.setMaxLevel(level);
    float lastFreq = smoothRand.getFreq();
    float lastLevel = smoothRand.getLevel();
    bool edgeHigh = clockEdge.isEdgeHigh();
    bool edgeGate = clockEdge.getValue();
    uint16_t edgeBPM = clockEdge.getBPM();
    smoothRand.update(edgeHigh, clockMode);
    lfo.setWave((MultiWaveOscEx::Wave)wave);
    lfo.setFrequency(max(lastFreq, minFreq == 0 ? 0.05 : minFreq));
    pwm_set_gpio_level(OUT2, lastLevel);

    // DC Outs
    uint16_t voct = outputSemiSelect * voltPerTone;
    voct = constrain(voct - PWMCVDCOutputErrorLUT[(int16_t)outputSemiSelect], 0, PWM_RESO - 1);
    pwm_set_gpio_level(OUT3, voct);
    pwm_set_gpio_level(OUT4, (int16_t)(potValue * 0.5));

    // Trigger Outs
    if (edgeHigh)
    {
        int8_t selResoIndex = randFast.getRandom16(minMultiply, maxMultiply + 1);
        rndMultiPollingEvent.setBPM(edgeBPM, selReso[selResoIndex]);
        rndMultiPollingEvent.stop();
        rndMultiPollingEvent.start();
        rndMultiOut.setDuration(rndMultiPollingEvent.getMills() >> (4 - selResoIndex));
        rndMultiOut.update(edgeHigh);
        multiPollingEvent.setBPM(edgeBPM, 8);
        multiPollingEvent.stop();
        multiPollingEvent.start();
        multiOut.setDuration(multiPollingEvent.getMills() >> 2);
        multiOut.update(edgeHigh);
    }
    else
    {
        rndMultiOut.update(rndMultiPollingEvent.ready());
        multiOut.update(multiPollingEvent.ready());
    }

    // LEDs
    if (menuIndex == 0)
    {
        switch (oscDataIndex)
        {
        case 0:
            gpio_put(LED1, vOctValue < 2 ? HIGH : LOW);
            gpio_put(LED2, vOctValue > ADC_RESO - 2 ? HIGH : LOW);
            break;
        case 1:
            gpio_put(LED1, cv1Value < 2 ? HIGH : LOW);
            gpio_put(LED2, cv1Value > ADC_RESO - 2 ? HIGH : LOW);
            break;
        case 2:
            gpio_put(LED1, cv2Value < 2 ? HIGH : LOW);
            gpio_put(LED2, cv2Value > ADC_RESO - 2 ? HIGH : LOW);
            break;
        default:
            break;
        }
    }
    else
    {
        gpio_put(LED1, edgeGate ? HIGH : LOW);
        gpio_put(LED2, vOctValue > ADC_RESO - 2 ? HIGH : LOW);
    }

    tight_loop_contents();
    sleep_us(250);
}

void setup1()
{
    oscillo.init(&u8g2, 0);

    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    tight_loop_contents();

    uint16_t potValueTemp = pot.getValue();
    int8_t encValue = enc.getValue();
    int8_t btn0 = buttons[0].getState();
    int8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        if (menuIndex >= MENU_MAX - 1)
        {
            requiresUpdate |= menuControl.select(encValue);
            if (menuControl.isUnder())
            {
                menuIndex--;
                requiresUpdate = true;
            }
        }
        else
        {
            int menu = 0;
            menu = constrain(menuIndex + encValue, 0, MENU_MAX - 1);
            requiresUpdate |= menuIndex != menu ? 1 : 0;
            menuIndex = menu;
        }

        encValue = 0;
    }

    switch (menuIndex)
    {
    case 0:
        requiresUpdate |= 1;
        if ((btn0 == 3 && btn1 == 2) || (btn0 == 2 && btn1 == 3))
        {
            oscBias = oscBias ? false : true;
        }
        else if (btn0 == 2)
        {
            oscDataIndex = constrainCyclic(oscDataIndex + 1, 0, 2);
        }
        else if (btn1 == 2)
        {
            oscillo.setTrigger(oscillo.getTrigger() ? false : true);
        }
        oscillo.addVerticalScale(encValue);
        oscillo.setHorizontalScale(map(potValueTemp, 0, ADC_RESO - 1, 0, SNAPSHOT_INDEX_MAX));
        break;
    default:
        if (btn0 > 0 || btn1 > 0)
        {
            requiresUpdate |= 1;
        }

        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        requiresUpdate = 1;

        potValue = adcErrorCorrection.correctedAdc(potValueTemp);
        break;
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
