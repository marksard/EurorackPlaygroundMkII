/*!
 * Adjuster APP
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

#include "lib/MultiWaveOsc.hpp"
#include "lib/PollingTimeEvent.hpp"
#include "lib/EdgeChecker.hpp"
#include "OscilloscopeLite.hpp"

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
static ADCErrorCorrection adcErrorCorrection(3.3);

//////////////////////////////////////////
// entries
static int16_t vOctValue = 0;
static int16_t gateValue = 0;
static int16_t potValue = 0;
static int16_t cv1Value = 0;
static int16_t cv2Value = 0;
static int16_t btnAValue = 0;
static int16_t btnBValue = 0;
static int16_t bias = 0;
static int16_t outputValue = 0;
static int16_t outputSemiSelect = 0;
static OscilloscopeLite oscillo(SAMPLE_FREQ);
// static MultiWaveOsc osc;

//////////////////////////////////////////

// 画面周り
#define MENU_MAX (1 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
static uint8_t oscDataIndex = 0;
static bool oscBias = 0;
const char oscDataNames[][5] = {"VOCT", "CV1 ", "CV2 "};
PollingTimeEvent updateOLED;

const char biasONOFF[][5] = {"OFF", "ON"};

SettingItem16 settings1[] =
    {
        SettingItem16(0, 60, 1, &outputSemiSelect, "sel semi: %d", NULL, 0),
        SettingItem16(0, 1, 1, &bias, "out bias: %s", biasONOFF, 2),
        SettingItem16(0, 4095, 1, &outputValue, "chk out: %d", NULL, 0),
};

SettingItem16 settings2[] =
    {
        SettingItem16(0, 4095, 1, &potValue, "chk pot: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &btnAValue, "chk btnA: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &btnBValue, "chk btnB: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &gateValue, "chk gate: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &vOctValue, "chk voct: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &cv1Value, "chk cv1: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &cv2Value, "chk cv2: %d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"OUTPUT CHK", settings1, sizeof(settings1) / sizeof(settings1[0])},
    {"INPUT CHK", settings2, sizeof(settings2) / sizeof(settings2[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));
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

void calibration(float &vref, float &noiseFloor)
{
    Serial.println("VOCT Calibration");
    pwm_set_gpio_level(OUT3, 0);
    sleep_ms(50);
    noiseFloor = adcErrorCorrection.getADCMax16(VOCT);
    pwm_set_gpio_level(OUT3, 2047);
    sleep_ms(50);
    float adc = adcErrorCorrection.getADCMax16(VOCT);
    Serial.print("ADC at 5V:");
    Serial.print(adc);
    if (adc >= 4093)
    {
        // 3.26989付近なので半分の電圧から推定しなおす
        pwm_set_gpio_level(OUT3, 1024);
        sleep_ms(50);
        adc = adcErrorCorrection.getADCMax16(VOCT);
        Serial.print(" at 2.5V:");
        Serial.print(adc);
        adc *= 2;
    }
    pwm_set_gpio_level(OUT3, 0);
    sleep_ms(50);
    vref = adcErrorCorrection.getADC2VRef(adc);
    Serial.print(" vref:");
    Serial.print(vref, 4);
    Serial.print(" noiseFloor:");
    Serial.println(noiseFloor);

    adcErrorCorrection.generateLUT(vref, noiseFloor);
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);
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
    
    // pwm_set_gpio_level(OUT1, osc.getWaveValue());
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

    adcErrorCorrection.init(3.3, 20.0);

    // osc.init(SAMPLE_FREQ, PWM_BIT);
    // osc.setWave(MultiWaveOsc::Wave::SQU);
    // osc.setFrequency(33);

    // ADC LOG Mode
    if (gpio_get(BTN1) == false)
    {
        while (!Serial)
        {
        }

        float vref = 0.0;
        float noiseFloor = 0.0;
        calibration(vref, noiseFloor);
        sleep_ms(100);
        Serial.println("out,raw_adc,raw_diff,cor_adc,cor_diff");
        for (int i = 0; i < PWM_RESO; ++i)
        {
            pwm_set_gpio_level(OUT3, i);
            sleep_ms(1);
            int16_t raw_adc = adcErrorCorrection.getADCAvg16(VOCT);
            int16_t raw_diff = (i * 2) - adcErrorCorrection.correctedInputScaleAdc(raw_adc);
            Serial.print(i * 2);
            Serial.print(",");
            Serial.print(raw_adc);
            Serial.print(",");
            Serial.print(raw_diff);
            int16_t cor_adc = (int)adcErrorCorrection.correctedAdc(raw_adc);
            int16_t cor_diff = (i * 2) - cor_adc;
            Serial.print(",");
            Serial.print(cor_adc);
            Serial.print(",");
            Serial.print(cor_diff);
            Serial.println();
        }
    }


    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
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

    // float voctPowV = adcErrorCorrection.voctPow(vOctValue);
    // float vOctFreq = osc.getFreqFromNoteIndex(24) * voctPowV;
    // osc.setFrequency(vOctFreq);

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
        gpio_put(LED1, gateValue ? HIGH : LOW);
        gpio_put(LED2, vOctValue > ADC_RESO - 2 ? HIGH : LOW);
    }

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

    sleep_us(50);
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
    int8_t encValue = enc.getValue();
    btnAValue = buttons[0].getState();
    btnBValue = buttons[1].getState();
    uint8_t btnREValue = buttons[2].getState();

    if (btnREValue == 2)
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
        if ((btnAValue == 3 && btnBValue == 2) || (btnAValue == 2 && btnBValue == 3))
        {
            oscBias = oscBias ? false : true;
        }
        else if (btnAValue == 2)
        {
            oscDataIndex = constrainCyclic(oscDataIndex + 1, 0, 2);
        }
        else if (btnBValue == 2)
        {
            oscillo.setTrigger(oscillo.getTrigger() ? false : true);
        }
        oscillo.addVerticalScale(encValue);
        oscillo.setHorizontalScale(map(potValue, 0, ADC_RESO - 1, 0, SNAPSHOT_INDEX_MAX));
        break;
    default:
        if (btnAValue > 0 || btnBValue > 0)
        {
            requiresUpdate |= 1;
        }

        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        requiresUpdate = 1;
        break;
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
