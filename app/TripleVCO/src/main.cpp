/*!
 * Triple VCO
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include "../../common/lib/Button.hpp"
#include "../../common/lib/SmoothAnalogRead.hpp"
#include "../../common/lib/RotaryEncoder.hpp"
#include "../../common/lib/ADCErrorCorrection.hpp"
#include "../../common/lib/EepRomConfigIO.hpp"
#include "../../common/ui_common/SettingItem.hpp"
#include "../../common/lib/pwm_wrapper.h"
#include "../../common/gpio_mapping.h"
#include "../../common/basic_definition.h"

#include "../../common/lib/PollingTimeEvent.hpp"
#include "../../common/lib/EdgeChecker.hpp"

#include "../../common/MultiWaveOscEx.hpp"
#include "../../common/SmoothRandomCV.hpp"
#include "UserConfig.h"

// #define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8.0) // 32470.703125khz
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static EdgeChecker gate;
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;
static int pwmOuts[] = { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6 };
static uint8_t pwmOutsCount = sizeof(pwmOuts) / sizeof(pwmOuts[0]);
static ADCErrorCorrection adcErrorCorrection(3.3f);

// ユーザー設定
static EEPROMConfigIO<UserConfig> userConfig(0); 
static bool saveConfirm = false;

// triple vco
#define VCO_MAX_ROOT_INDEX 96 // noteNameのC7
#define LFO_MAX_COARSE_FREQ 33
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO - 1)))) - 1) / (exp(ratio) - 1)
static MultiWaveOscEx osc[3];
// static StateVariableFilter svf;

// smooth random
static SmoothRandomCV smoothRand(ADC_RESO);

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

SettingItem16 commonSettings[] =
{
    SettingItem16(0, 1, 1, &userConfig.Config.rangeMode, "Range Mode: %s", vclfo, 2),
    SettingItem16(0, 3, 1, &userConfig.Config.oscBVOct, "VCO B VOCT: %s", selvoct, 4),
    SettingItem16(0, 3, 1, &userConfig.Config.oscCVOct, "LFO   VOCT: %s", selvoct, 4),
    SettingItem16(0, 2, 1, &userConfig.Config.oscAParaCV, "VCO A Para: %s", selcv, 3),
    SettingItem16(0, 2, 1, &userConfig.Config.oscBWaveCV, "VCO B Wave: %s", selcv, 3),
    SettingItem16(-200, 200, 1, &userConfig.Config.voctTune, "Init  VOCT:%4d", NULL, 0)
};

SettingItem16 smoothRandSettings[] =
{
    SettingItem16(0, 100, 1, &userConfig.Config.smoothLevel, "CV Level: %d", NULL, 0),
    SettingItem16(1, 100, 1, &userConfig.Config.smoothncurve, "CV Curve: %d", NULL, 0),
    SettingItem16(0, 32, 1, &userConfig.Config.smoothMaxFreq, "LFO MaxF:%d", NULL, 0),
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
    u8g2.setFont(u8g2_font_7x14B_tf);
    if (oscIndex == 2)
    {
        sprintf(disp_buf, "%s", modeDisp[1]);
        u8g2.drawStr(0, 0, disp_buf);
        sprintf(disp_buf, "%s", osc[oscIndex].getNoteNameOrFreq(1));
        u8g2.drawStr(0, 48, disp_buf);
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

    if (osc[oscIndex].getWave() == MultiWaveOsc::Wave::SAW || osc[oscIndex].getWave() == MultiWaveOsc::Wave::MUL_TRI)
    {
        if (userConfig.Config.oscAParaCV > 0)
            sprintf(disp_buf, "%s p:cv%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), userConfig.Config.oscAParaCV);
        else
            sprintf(disp_buf, "%s p:%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), osc[oscIndex].getPhaseShift());
    }
    else if (osc[oscIndex].getWave() == MultiWaveOsc::Wave::TRI ||
             osc[oscIndex].getWave() == MultiWaveOsc::Wave::SINE)
    {
        if (userConfig.Config.oscAParaCV > 0)
            sprintf(disp_buf, "%s f:cv%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), userConfig.Config.oscAParaCV);
        else
            sprintf(disp_buf, "%s f:%d", osc[oscIndex].getNoteNameOrFreq(rangeMode), osc[oscIndex].getFolding());
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
        drawOSC(0, userConfig.Config.rangeMode);
        break;
    case 1:
        drawOSC(1, userConfig.Config.rangeMode);
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

    uint16_t valueA = osc[0].getWaveValue();
    uint16_t valueB = osc[1].getWaveValue();
    uint16_t valueC = osc[2].getWaveValue();

    pwm_set_gpio_level(OUT1, valueA);
    pwm_set_gpio_level(OUT2, valueB);
    pwm_set_gpio_level(OUT3, valueC);
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    analogReadResolution(ADC_BIT);
    pinMode(23, OUTPUT);
    gpio_put(23, HIGH);

    pot.init(POT1);
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[0].setHoldTime(250);
    buttons[1].init(BTN2);
    buttons[1].setHoldTime(250);
    buttons[2].init(BTN3);
    gate.init(GATE);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);

    userConfig.initEEPROM();
    userConfig.loadUserConfig();

    osc[0].init(SAMPLE_FREQ, PWM_BIT);
    osc[0].setWave((MultiWaveOsc::Wave)userConfig.Config.oscAWave);
    osc[0].setFrequency(userConfig.Config.oscACoarse);
    osc[0].setFreqName(userConfig.Config.oscACoarse);
    osc[0].setPhaseShift(userConfig.Config.oscAPhaseShift);
    osc[0].setFolding(userConfig.Config.oscAFolding);
    osc[1].init(SAMPLE_FREQ, PWM_BIT);
    osc[1].setWave((MultiWaveOsc::Wave)userConfig.Config.oscBWave);
    osc[1].setFrequency(userConfig.Config.oscBCoarse);
    osc[1].setFreqName(userConfig.Config.oscBCoarse);
    osc[1].setPhaseShift(userConfig.Config.oscBPhaseShift);
    osc[1].setFolding(userConfig.Config.oscBFolding);
    osc[2].init(SAMPLE_FREQ, PWM_BIT);
    osc[2].setWave((MultiWaveOsc::Wave)userConfig.Config.oscCWave);
    osc[2].setFrequency(userConfig.Config.oscCCoarse);
    osc[2].setFreqName(userConfig.Config.oscCCoarse);
    osc[2].setPhaseShift(0);

    initPWM(OUT1, PWM_RESO, false);
    initPWM(OUT2, PWM_RESO, false);
    initPWM(OUT3, PWM_RESO, false);
    initPWM(OUT4, PWM_RESO, false);
    initPWM(OUT5, PWM_RESO, false);
    initPWM(OUT6, PWM_RESO, false);
    initPWM(LED1, PWM_RESO);
    initPWM(LED2, PWM_RESO);

    uint slice = 0;
    for (int i = 0; i < pwmOutsCount; ++i)
    {
        slice |= 0x01 << pwm_gpio_to_slice_num(pwmOuts[i]);
    }
    pwm_set_mask_enabled(slice);

    adcErrorCorrection.init(3.295f, 25.0f);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    // delay(500);
    osc[0].startFolding(true);
    osc[1].startFolding(true);
}

void loop()
{
    pot.analogRead(false);
    enc.getDirection();
    int16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    int16_t cv2Value = cv2.analogReadDirectFast();

    float powVOct = adcErrorCorrection.voctPow(voct);
    float powCv1 = adcErrorCorrection.voctPow(cv1Value);
    float powCv2 = adcErrorCorrection.voctPow(cv2Value);

    float freqencyA = max(userConfig.Config.oscACoarse * powVOct, 0.002);
    float selVoctB = 1;
    if (userConfig.Config.oscBVOct == 1)
        selVoctB = powVOct;
    else if (userConfig.Config.oscBVOct == 2)
        selVoctB = powCv1;
    else if (userConfig.Config.oscBVOct == 3)
        selVoctB = powCv2;
    float freqencyB = max(userConfig.Config.oscBCoarse * selVoctB, 0.002);

    float selVoctC = 1;
    if (userConfig.Config.oscCVOct == 1)
        selVoctC = powVOct;
    else if (userConfig.Config.oscCVOct == 2)
        selVoctC = powCv1;
    else if (userConfig.Config.oscCVOct == 3)
        selVoctC = powCv2;
    float freqencyC = max(userConfig.Config.oscCCoarse * selVoctC, 0.002);

    osc[0].setFrequency(freqencyA);
    osc[1].setFrequency(freqencyB);
    osc[2].setFrequency(freqencyC);

    // OSCB CVで波形変更
    if (userConfig.Config.oscBWaveCV > 0)
    {
        int16_t shift = map(userConfig.Config.oscBWaveCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, (int)MultiWaveOsc::Wave::SQU, (int)MultiWaveOsc::Wave::NOISE + 1);
        // requiresUpdate |= osc[0].setWave((MultiWaveOsc::Wave)shift);
        requiresUpdate |= osc[1].setWave((MultiWaveOsc::Wave)shift) & (menuIndex == 1);
    }

    // OSCA CVでパラメータ変更
    if (userConfig.Config.oscAParaCV > 0)
    {
        int16_t shift = map(userConfig.Config.oscAParaCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, 0, (PWM_RESO >> 1) - 1);
        requiresUpdate |= osc[0].setFolding(shift) & (menuIndex == 0);
        requiresUpdate |= osc[0].setPhaseShift(shift) & (menuIndex == 0);
    }

    // smooth random
    smoothRand.setCurve(userConfig.Config.smoothncurve);
    smoothRand.setMaxFreq(userConfig.Config.smoothMaxFreq);
    smoothRand.setMaxLevel(userConfig.Config.smoothLevel);
    smoothRand.update(false, false);
    // float lastFreq = smoothRand.getFreq();
    int16_t lastLevel = smoothRand.getLevel();
    pwm_set_gpio_level(OUT5, lastLevel);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(voct);
    //     Serial.print(", ");
    //     // Serial.print(cv1Value);
    //     // Serial.print(", ");
    //     // Serial.print(cv2Value);
    //     // Serial.print(", ");
    //     Serial.print(userConfig.Config.oscACoarse);
    //     Serial.print(", ");
    //     Serial.print(freqencyA);
    //     Serial.print(", ");
    //     Serial.print(userConfig.Config.oscBCoarse);
    //     Serial.print(", ");
    //     Serial.print(freqencyB);
    //     Serial.print(", ");
    //     Serial.print(userConfig.Config.oscCCoarse);
    //     Serial.print(", ");
    //     Serial.print(freqencyC);
    //     Serial.print(", ");
    //     Serial.println();
    // }

    sleep_us(20); // 10kHz
    tight_loop_contents();
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    uint16_t potValue = pot.getValue();
    int8_t encValue = enc.getValue();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();
    int16_t cv1Value = cv1.getValue();
    int16_t cv2Value = cv2.getValue();

    static uint16_t lastPotValue = potValue;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0 && btn0 == 0)
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

    // メニュー変更時のポットロック・ロック解除
    if (!unlock)
    {
        if (lastPotValue + 20 < potValue || lastPotValue - 20 > potValue)
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
            if (userConfig.Config.rangeMode)
            {
                userConfig.Config.oscACoarse = EXP_CURVE((float)potValue, 2.0) * LFO_MAX_COARSE_FREQ;
                requiresUpdate |= osc[0].setNoteNameFromFrequency(userConfig.Config.oscACoarse);
                requiresUpdate |= osc[0].setFreqName(userConfig.Config.oscACoarse);
            }
            else
            {
                int16_t root = map(potValue, 0, ADC_RESO - 1, 0, VCO_MAX_ROOT_INDEX);
                requiresUpdate |= osc[0].setCourceFromNoteNameIndex(root);
                userConfig.Config.oscACoarse = osc[0].getCource();
            }
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        if (btn0 != 3)
        {
            requiresUpdate |= osc[0].setWave((MultiWaveOsc::Wave)
                                                 constrainCyclic((int)osc[0].getWave() + (int)encValue, 0, (int)MultiWaveOsc::Wave::MAX));
            userConfig.Config.oscAWave = osc[0].getWave();
        }
        else if (btn0 == 3)
        {
            if (userConfig.Config.oscAWave == MultiWaveOsc::Wave::SAW || userConfig.Config.oscAWave == MultiWaveOsc::Wave::MUL_TRI)
            {
                osc[0].addPhaseShift((int)encValue);
                requiresUpdate |= userConfig.Config.oscAPhaseShift != osc[0].getPhaseShift();
                userConfig.Config.oscAPhaseShift = osc[0].getPhaseShift();
            }
            else if (userConfig.Config.oscAWave == MultiWaveOsc::Wave::TRI ||
                     userConfig.Config.oscAWave == MultiWaveOsc::Wave::SINE)
            {
                osc[0].addFolding((int)encValue);
                requiresUpdate |= userConfig.Config.oscAFolding != osc[0].getFolding();
                userConfig.Config.oscAFolding = osc[0].getFolding();
            }
        }
        break;
    case 1:
        if (unlock)
        {
            if (userConfig.Config.rangeMode)
            {
                userConfig.Config.oscBCoarse = EXP_CURVE((float)potValue, 2.0) * LFO_MAX_COARSE_FREQ;
                requiresUpdate |= osc[1].setNoteNameFromFrequency(userConfig.Config.oscBCoarse);
                requiresUpdate |= osc[1].setFreqName(userConfig.Config.oscBCoarse);
            }
            else
            {
                int16_t root = map(potValue, 0, ADC_RESO - 1, 0, VCO_MAX_ROOT_INDEX);
                requiresUpdate |= osc[1].setCourceFromNoteNameIndex(root);
                userConfig.Config.oscBCoarse = osc[1].getCource();
            }
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        if (btn0 != 3)
        {
            requiresUpdate |= osc[1].setWave((MultiWaveOsc::Wave)
                                                 constrainCyclic((int)osc[1].getWave() + (int)encValue, 0, (int)MultiWaveOsc::Wave::MAX));
            userConfig.Config.oscBWave = osc[1].getWave();
        }
        else if (btn0 == 3)
        {
            if (userConfig.Config.oscBWave == MultiWaveOsc::Wave::SAW || userConfig.Config.oscBWave == MultiWaveOsc::Wave::MUL_TRI)
            {
                osc[1].addPhaseShift((int)encValue);
                requiresUpdate |= userConfig.Config.oscBPhaseShift != osc[1].getPhaseShift();
                userConfig.Config.oscBPhaseShift = osc[1].getPhaseShift();
            }
            else if (userConfig.Config.oscBWave == MultiWaveOsc::Wave::TRI ||
                     userConfig.Config.oscBWave == MultiWaveOsc::Wave::SINE)
            {
                osc[1].addFolding((int)encValue);
                requiresUpdate |= userConfig.Config.oscBFolding != osc[1].getFolding();
                userConfig.Config.oscBFolding = osc[1].getFolding();
            }
        }
        break;
    case 2:
        if (unlock)
        {
            userConfig.Config.oscCCoarse = EXP_CURVE((float)potValue, 2.0) * LFO_MAX_COARSE_FREQ;
            requiresUpdate |= osc[2].setNoteNameFromFrequency(userConfig.Config.oscCCoarse);
            requiresUpdate |= osc[2].setFreqName(userConfig.Config.oscCCoarse);
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[2].setWave((MultiWaveOsc::Wave)
                                             constrainCyclic((int)osc[2].getWave() + (int)encValue, 0, (int)MultiWaveOsc::Wave::MAX));
        userConfig.Config.oscCWave = osc[2].getWave();
        break;
    default:
        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        break;
    }

    if (menuIndex < 2 && btn0 == 3)
    {
        pwm_set_gpio_level(LED1, btn0 == 3 ? PWM_RESO - 1 : 0);
    }
    else {
        pwm_set_gpio_level(LED1, cv1Value > 4090 ? PWM_RESO - 1 : 0);
        pwm_set_gpio_level(LED2, cv2Value > 4090 ? PWM_RESO - 1 : 0);
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
            userConfig.saveUserConfig();
            saveConfirm = false;
            requiresUpdate |= 1;
        }
        else if (btn1 == 2)
        {
            saveConfirm = false;
            requiresUpdate |= 1;
        }
    }

    if (!updateOLED.ready())
    {
        tight_loop_contents();
        // sleep_ms(1);
        return;
    }

    dispOLED();
}
