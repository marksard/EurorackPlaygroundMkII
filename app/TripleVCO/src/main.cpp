/*!
 * Triple VCO
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
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "Oscillator.hpp"
#include "EepromData.h"
#include "SmoothRandomCV.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 4096         // 12bit
// #define PWM_RESO 1024         // 10bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 4) // 64941.40625khz
// #define SAMPLE_FREQ 88200
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;
static int pwmOuts[6] = { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6 };

// ユーザー設定
static UserConfig userConfig;
static bool saveConfirm = false;

// triple vco
#define VCO_MAX_COARSE_FREQ 440
#define LFO_MAX_COARSE_FREQ 66
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO - 1)))) - 1) / (exp(ratio) - 1)
static Oscillator osc[3];
static float max_coarse_freq = VCO_MAX_COARSE_FREQ;

// 画面周り
#define MENU_MAX (3 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

const char vclfo[][5] = {"VCO", "LFO"};
const char onoff[][5] = {"OFF", "ON"};
const char selvoct[][5] = {"OFF", "VOCT", "CV1", "CV2"};
const char selcv[][5] = {"OFF", "CV1", "CV2"};
static SmoothRandomCV smoothRand(ADC_RESO);

SettingItem16 commonSettings[] =
{
    SettingItem16(0, 1, 1, &userConfig.rangeMode, "Range Mode: %s", vclfo, 2),
    SettingItem16(0, 3, 1, &userConfig.oscBVOct, "VCO B VOCT: %s", selvoct, 4),
    SettingItem16(0, 3, 1, &userConfig.oscCVOct, "LFO   VOCT: %s", selvoct, 4),
    SettingItem16(0, 2, 1, &userConfig.oscAParaCV, "VCO A Para: %s", selcv, 3),
    SettingItem16(0, 2, 1, &userConfig.oscBWaveCV, "VCO B Wave: %s", selcv, 3),
    SettingItem16(-200, 200, 1, &userConfig.voctTune, "Init  VOCT:%4d", NULL, 0)
};

SettingItem16 smoothRandSettings[] =
{
    SettingItem16(0, 100, 1, &userConfig.smoothLevel, "CV Level: %d", NULL, 0),
    SettingItem16(1, 100, 1, &userConfig.smoothncurve, "CV Curve: %d", NULL, 0),
    SettingItem16(0, 32, 1, &userConfig.smoothMaxFreq, "LFO MaxF:%d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"SETTINGS", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
    {"SMOOTH RND", smoothRandSettings, sizeof(smoothRandSettings) / sizeof(smoothRandSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

static char modeDisp[2][4] = {"VCO", "LFO"};
static char oscNames[3][2] = {"A", "B", "C"};
void drawOSC(uint8_t oscIndex, uint8_t rangeMode)
{
    static char disp_buf[33] = {0};
    // u8g2.setFont(u8g2_font_VCR_OSD_tf);
    u8g2.setFont(u8g2_font_7x14B_tf);
    if (oscIndex == 2)
    {
        sprintf(disp_buf, "%s", modeDisp[rangeMode]);
        u8g2.drawStr(0, 0, disp_buf);
        // sprintf(disp_buf, "%s", osc[oscIndex].getNoteNameOrFreq(rangeMode));
        // u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[oscIndex].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        if (encMode == 0)
            u8g2.drawBox(0, 13, 127, 2);
        else
            u8g2.drawBox(0, 44, 127, 2);
        return;
    }

    sprintf(disp_buf, "%s %s", modeDisp[rangeMode], oscNames[oscIndex]);
    u8g2.drawStr(0, 0, disp_buf);

        if (osc[0].getWave() == Oscillator::Wave::SAW || osc[0].getWave() == Oscillator::Wave::MUL_TRI)
    {
        if (userConfig.oscAParaCV > 0)
            sprintf(disp_buf, "%s p:cv%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), userConfig.oscAParaCV);
        else
            sprintf(disp_buf, "%s p:%02d", osc[oscIndex].getNoteNameOrFreq(rangeMode), osc[oscIndex].getPhaseShift());
    }
    else if (osc[oscIndex].getWave() == Oscillator::Wave::TRI ||
             osc[oscIndex].getWave() == Oscillator::Wave::SINE)
    {
        if (userConfig.oscAParaCV > 0)
            sprintf(disp_buf, "%s f:cv%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), userConfig.oscAParaCV);
        else
            sprintf(disp_buf, "%s f:%02d", osc[oscIndex].getNoteNameOrFreq(rangeMode), osc[oscIndex].getFolding());
    }
    else
    {
        sprintf(disp_buf, "%s", osc[oscIndex].getNoteNameOrFreq(rangeMode));
    }
    u8g2.drawStr(0, 48, disp_buf);
    u8g2.setFont(u8g2_font_logisoso26_tf);
    sprintf(disp_buf, "%s", osc[oscIndex].getWaveName());
    u8g2.drawStr(0, 16, disp_buf);

    if (encMode == 0)
        u8g2.drawBox(0, 13, 127, 2);
    else
        u8g2.drawBox(0, 44, 127, 2);
}

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
        drawOSC(0, userConfig.rangeMode);
        break;
    case 1:
        drawOSC(1, userConfig.rangeMode);
        break;
    case 2:
        drawOSC(2, 1);
        break;
    default:
        menuControl.draw(&u8g2, encMode);
        break;
    }

    if (saveConfirm)
    {
        u8g2.setDrawColor(0);
        u8g2.drawBox(0, 0, 128, 40);
        u8g2.setDrawColor(2);
        u8g2.drawFrame(0, 0, 128, 40);
        u8g2.setFont(u8g2_font_VCR_OSD_mf);
        sprintf(disp_buf, "SAVE?");
        u8g2.drawStr(6, 0, disp_buf);
        sprintf(disp_buf, "Yes:A No:B");
        u8g2.drawStr(5, 16, disp_buf);
    }

    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);
    uint16_t valueA = osc[0].getWaveValue();
    uint16_t valueB = osc[1].getWaveValue();
    uint16_t valueC = osc[2].getWaveValue();

    pwm_set_gpio_level(OUT1, valueA);
    pwm_set_gpio_level(OUT2, valueB);
    pwm_set_gpio_level(OUT3, valueC);
    pwm_set_gpio_level(OUT4, osc[2].getRandom16(2048));
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
    buttons[0].init(BTN1, false);
    buttons[1].init(BTN2, false);
    buttons[2].init(BTN3, false);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    // pinMode(GATE, INPUT);

    initEEPROM();
    loadUserConfig(&userConfig);

    max_coarse_freq = userConfig.rangeMode ? (float)LFO_MAX_COARSE_FREQ : (float)VCO_MAX_COARSE_FREQ;

    osc[0].init(SAMPLE_FREQ);
    osc[0].setWave((Oscillator::Wave)userConfig.oscAWave);
    osc[0].setFrequency(userConfig.oscACoarse);
    osc[0].setFreqName(userConfig.oscACoarse);
    osc[0].setPhaseShift(userConfig.oscAPhaseShift);
    osc[0].setFolding(userConfig.oscAFolding);
    osc[1].init(SAMPLE_FREQ);
    osc[1].setWave((Oscillator::Wave)userConfig.oscBWave);
    osc[1].setFrequency(userConfig.oscBCoarse);
    osc[1].setFreqName(userConfig.oscBCoarse);
    osc[1].setPhaseShift(userConfig.oscBPhaseShift);
    osc[1].setFolding(userConfig.oscBFolding);
    osc[2].init(SAMPLE_FREQ);
    osc[2].setWave((Oscillator::Wave)userConfig.oscCWave);
    osc[2].setFrequency(userConfig.oscCCoarse);
    osc[2].setFreqName(userConfig.oscCCoarse);
    osc[2].setPhaseShift(5);

    initPWM(OUT1, PWM_RESO, false);
    initPWM(OUT2, PWM_RESO, false);
    initPWM(OUT3, PWM_RESO, false);
    initPWM(OUT4, PWM_RESO, false);
    initPWM(OUT5, PWM_RESO, false);
    initPWM(OUT6, PWM_RESO, false);
    initPWM(LED1, PWM_RESO);
    initPWM(LED2, PWM_RESO);

    uint slice = 0;
    for (int i = 0; i < 6; ++i)
    {
        slice |= 0x01 << pwm_gpio_to_slice_num(pwmOuts[i]);
    }
    pwm_set_mask_enabled(slice);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    // delay(500);
    osc[0].startFolding(true);
    osc[1].startFolding(true);
}

void loop()
{
    uint16_t potValue = pot.analogRead(true, true);
    int8_t encValue = enc.getDirection();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();
    uint16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    uint16_t cv2Value = cv2.analogReadDirectFast();

    static float coarseA = userConfig.oscACoarse;
    static float coarseB = userConfig.oscBCoarse;
    static float coarseC = userConfig.oscCCoarse;
    static uint16_t lastPotValue = potValue;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    float powCv1 = (float)pow(2, map(cv1Value, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    float powCv2 = (float)pow(2, map(cv2Value, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);

    float freqencyA = max(coarseA * powVOct, 0.01);
    float selVoctB = 1;
    if (userConfig.oscBVOct == 1)
        selVoctB = powVOct;
    else if (userConfig.oscBVOct == 2)
        selVoctB = powCv1;
    else if (userConfig.oscBVOct == 3)
        selVoctB = powCv2;
    float freqencyB = max(coarseB * selVoctB, 0.01);

    float selVoctC = 1;
    if (userConfig.oscCVOct == 1)
        selVoctC = powVOct;
    else if (userConfig.oscCVOct == 2)
        selVoctC = powCv1;
    else if (userConfig.oscCVOct == 3)
        selVoctC = powCv2;
    float freqencyC = max(coarseC * selVoctC, 0.01);

    osc[0].setFrequency(freqencyA);
    osc[1].setFrequency(freqencyB);
    osc[2].setFrequency(freqencyC);
    userConfig.oscACoarse = coarseA;
    userConfig.oscBCoarse = coarseB;
    userConfig.oscCCoarse = coarseC;
    pwm_set_gpio_level(LED1, cv1Value);
    pwm_set_gpio_level(LED2, cv2Value);

    // requiresUpdate |= updateMenuIndex(btn0, btn1);
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
            // if (menuControl.isOver())
            // {
            //     menuIndex = 0;
            //     requiresUpdate = true;
            // }
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

    switch (menuIndex)
    {
    case 0:
        if (unlock)
        {
            coarseA = EXP_CURVE((float)potValue, 2.0) * max_coarse_freq;
        }
        if (userConfig.rangeMode)
            requiresUpdate |= osc[0].setFreqName(coarseA);
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[0].setNoteNameFromFrequency(coarseA);
        if (btn0 != 3)
        {
            requiresUpdate |= osc[0].setWave((Oscillator::Wave)
                                                 constrainCyclic((int)osc[0].getWave() + (int)encValue, 0, (int)Oscillator::Wave::MAX));
            userConfig.oscAWave = osc[0].getWave();
        }
        else if (btn0 == 3)
        {
            if (userConfig.oscAWave == Oscillator::Wave::SAW || userConfig.oscAWave == Oscillator::Wave::MUL_TRI)
            {
                osc[0].addPhaseShift((int)encValue);
                requiresUpdate |= userConfig.oscAPhaseShift != osc[0].getPhaseShift();
                userConfig.oscAPhaseShift = osc[0].getPhaseShift();
            }
            else if (userConfig.oscAWave == Oscillator::Wave::TRI ||
                     userConfig.oscAWave == Oscillator::Wave::SINE)
            {
                osc[0].addFolding((int)encValue);
                requiresUpdate |= userConfig.oscAFolding != osc[0].getFolding();
                userConfig.oscAFolding = osc[0].getFolding();
            }
        }
        break;
    case 1:
        if (unlock)
        {
            coarseB = EXP_CURVE((float)potValue, 2.0) * max_coarse_freq;
        }
        if (userConfig.rangeMode)
            requiresUpdate |= osc[1].setFreqName(coarseB);
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[1].setNoteNameFromFrequency(coarseB);
        if (btn0 != 3)
        {
            requiresUpdate |= osc[1].setWave((Oscillator::Wave)
                                                 constrainCyclic((int)osc[1].getWave() + (int)encValue, 0, (int)Oscillator::Wave::MAX));
            userConfig.oscBWave = osc[1].getWave();
        }
        else if (btn0 == 3)
        {
            if (userConfig.oscBWave == Oscillator::Wave::SAW || userConfig.oscBWave == Oscillator::Wave::MUL_TRI)
            {
                osc[1].addPhaseShift((int)encValue);
                requiresUpdate |= userConfig.oscBPhaseShift != osc[1].getPhaseShift();
                userConfig.oscBPhaseShift = osc[1].getPhaseShift();
            }
            else if (userConfig.oscBWave == Oscillator::Wave::TRI ||
                     userConfig.oscBWave == Oscillator::Wave::SINE)
            {
                osc[1].addFolding((int)encValue);
                requiresUpdate |= userConfig.oscBFolding != osc[1].getFolding();
                userConfig.oscBFolding = osc[1].getFolding();
            }
        }
        break;
    case 2:
        if (unlock)
        {
            coarseC = EXP_CURVE((float)potValue, 2.0) * LFO_MAX_COARSE_FREQ;
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        // requiresUpdate |= osc[2].setNoteNameFromFrequency(coarseC);
        requiresUpdate |= osc[2].setWave((Oscillator::Wave)
                                             constrainCyclic((int)osc[2].getWave() + (int)encValue, 0, (int)Oscillator::Wave::MAX));
        // requiresUpdate |= osc[2].setFreqName(coarseC);
        userConfig.oscCWave = osc[2].getWave();
        break;
    default:
        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        break;
    }

    // ec長押しで設定保存
    if (btn2 == 4)
    {
        saveConfirm = true;
        requiresUpdate |= 1;
    }
    if (saveConfirm)
    {
        if (btn0 == 2)
        {
            saveUserConfig(&userConfig);
            saveConfirm = false;
            requiresUpdate |= 1;
        }
        else if (btn1 == 2)
        {
            saveConfirm = false;
            requiresUpdate |= 1;
        }
    }

    // OSCB CVで波形変更
    if (userConfig.oscBWaveCV > 0)
    {
        int16_t shift = map(userConfig.oscBWaveCV == 1 ? cv1Value : cv2Value, 0, 4096, (int)Oscillator::Wave::SQU, (int)Oscillator::Wave::NOISE + 1);
        // requiresUpdate |= osc[0].setWave((Oscillator::Wave)shift);
        requiresUpdate |= osc[1].setWave((Oscillator::Wave)shift) & (menuIndex == 1);
    }

    // OSCA CVでパラメータ変更
    if (userConfig.oscAParaCV > 0)
    {
        int16_t shift = map(userConfig.oscAParaCV == 1 ? cv1Value : cv2Value, 0, 4096, 0, 50);
        requiresUpdate |= osc[0].setFolding(shift) & (menuIndex == 0);
        requiresUpdate |= osc[0].setPhaseShift(shift) & (menuIndex == 0);
    }

    max_coarse_freq = userConfig.rangeMode ? (float)LFO_MAX_COARSE_FREQ : (float)VCO_MAX_COARSE_FREQ;

    smoothRand.setCurve(userConfig.smoothncurve);
    smoothRand.setMaxFreq(userConfig.smoothMaxFreq);
    smoothRand.setMaxLevel(userConfig.smoothLevel);
    smoothRand.update(false, false);
    float lastFreq = smoothRand.getFreq();
    float lastLevel = smoothRand.getLevel();
    pwm_set_gpio_level(OUT5, lastLevel);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(voct);
    //     Serial.print(", ");
    //     Serial.print(cv1Value);
    //     Serial.print(", ");
    //     Serial.print(cv2Value);
    //     Serial.print(", ");
    //     // Serial.print(digitalRead(GATE));
    //     // Serial.print(userConfig.voctTune);
    //     // Serial.print(", ");
    //     Serial.print(coarseA);
    //     Serial.print(", ");
    //     Serial.print(freqencyA);
    //     Serial.print(", ");
    //     Serial.print(coarseB);
    //     Serial.print(", ");
    //     Serial.print(freqencyB);
    //     Serial.print(", ");
    //     Serial.println();
    // }

    sleep_us(100); // 10kHz
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
