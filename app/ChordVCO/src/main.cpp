/*!
 * Chord VCO
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
#include "Oscillator.hpp"
#include "EepromData.h"
#include "SmoothRandomCV.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
// #define PWM_RESO 4096         // 12bit
#define PWM_RESO 1024         // 10bit
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
static EdgeChecker gate;
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
static Oscillator osc[4];
static float max_coarse_freq = VCO_MAX_COARSE_FREQ;
static int8_t arpStep = 0;

static uint8_t addRootScale[7][4] = 
{
    {0, 3, 7, 10}, // Im7
    {0, 3, 6, 10}, // IIdim
    {0, 4, 7, 11}, // IIIM7
    {0, 3, 7, 10}, // IVm7
    {0, 3, 7, 10}, // Vm7
    {0, 4, 7, 11}, // VIM7
    {0, 4, 7, 10}, // VII7
};

static uint8_t rootScaleIndexFromSemitone[] = 
{
    0,0,1,2,0,3,0,4,5,0,0,6
};

// static uint8_t addRootScale[7][4] = 
// {
//     {0, 3, 7, 11}, // ImM7
//     {0, 3, 6, 10}, // IIdim
//     {0, 4, 8, 11}, // IIIM7#5
//     {0, 3, 7, 10}, // IVm7
//     {0, 4, 7, 10}, // V7
//     {0, 4, 7, 11}, // VIM7
//     {0, 3, 6,  9}, // VIIdim
// };

// static uint8_t rootScaleIndexFromSemitone[] = 
// {
//     0,0,1,2,0,3,0,4,5,0,6,0
// };

// 画面周り
#define MENU_MAX (1 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

const char selCV[][5] = {"---", "CV1", "CV2"};
const char selOct[][5] = {"---", "12", "24"};
const char selArp[][5] = {"---", "UP", "DOWN", "RAND"};
static SmoothRandomCV smoothRand(ADC_RESO);

SettingItem16 commonSettings[] =
{
    SettingItem16(0, 2, 1, &userConfig.rootMinus, "ROOT: %s", selOct, 3),
    SettingItem16(0, 2, 1, &userConfig.seventhMinus, "7TH: %s", selOct, 3),
    SettingItem16(0, 2, 1, &userConfig.oscAParaCV, "VCO A Para: %s", selCV, 3),
    SettingItem16(0, 3, 1, &userConfig.arpMode, "ARP Mode: %s", selArp, 4),
    SettingItem16(-200, 200, 1, &userConfig.voctTune, "Init  VOCT:%4d", NULL, 0)
};

static MenuSection16 menu[] = {
    {"CHORD VCO", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

static char modeDisp[1][4] = {"VCO"};
static char oscNames[1][2] = {"A"};
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

    if (osc[oscIndex].getWave() == Oscillator::Wave::SAW || osc[oscIndex].getWave() == Oscillator::Wave::MUL_TRI)
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
        drawOSC(0, 0);
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

    uint16_t sum = 0;
    uint16_t values[4] = {0};
    for (int i = 0; i < 4; ++i)
    {
        values[i] = osc[i].getWaveValue();
        sum += values[i];
    }

    // 平均してSQU以外は少しゲインを持ち上げる
    int16_t value = sum * 0.25;
    value -= 512;
    if (osc[0].getWave() == Oscillator::Wave::SQU)
    {
        value *= 0.9;
    }
    else
    {
        value = value * 1.25;
    }
    value += 512;
    value = constrain(value, 0, 1024);

    pwm_set_gpio_level(OUT1, value);
    pwm_set_gpio_level(OUT2, values[arpStep]);
    // pwm_set_gpio_level(OUT3, osc[0].getRandom16(1024));
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
    gate.init(GATE);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);

    initEEPROM();
    loadUserConfig(&userConfig);

    max_coarse_freq = (float)VCO_MAX_COARSE_FREQ;

    for (int i = 0; i < 4; ++i)
    {
        osc[i].init(SAMPLE_FREQ);
        osc[i].setWave((Oscillator::Wave)userConfig.oscAWave);
        osc[i].setFrequency(userConfig.oscACoarse);
        osc[i].setFreqName(userConfig.oscACoarse);
        osc[i].addPhaseShift(userConfig.oscAPhaseShift);
        osc[i].addFolding(userConfig.oscAFolding);
    }

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
    osc[2].startFolding(true);
    osc[3].startFolding(true);
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
    int16_t cv2Value = cv2.analogReadDirectFast();

    static float coarseA = userConfig.oscACoarse;
    static uint16_t lastPotValue = potValue;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    // float powCv1 = (float)pow(2, map(cv1Value, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    // float powCv2 = (float)pow(2, map(cv2Value, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);

    static uint8_t rootIndex = 0;
    static uint8_t rootConfirmCount = 0;
    uint8_t courceIndex = osc[0].getNoteNameIndexFromFreq(coarseA);
    float freq = coarseA * powVOct;
    uint8_t rootTemp = osc[0].getNoteNameIndexFromFreq(freq);

    if (rootTemp != rootIndex)
    {
        rootConfirmCount++;
        if (rootConfirmCount == 2)
        {
            arpStep = 0;
            rootIndex = rootTemp;
            rootConfirmCount = 0;
        }
    }
    else {
        rootConfirmCount = 0;
    }

    uint8_t rootDiff = (rootIndex - courceIndex) % 12;
    uint8_t scaleIndex = rootScaleIndexFromSemitone[rootDiff];
    int8_t rootMinus = userConfig.rootMinus * -12;
    int8_t seventhMinus = userConfig.seventhMinus * 12;
    osc[0].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[scaleIndex][0] + rootMinus);
    osc[1].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[scaleIndex][1]);
    osc[2].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[scaleIndex][2]);
    osc[3].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[scaleIndex][3] + seventhMinus);

    // arpeggio
    if (gate.isEdgeHigh())
    {
        if (userConfig.arpMode == 0)
            arpStep = 0;
        else if (userConfig.arpMode == 1)
            arpStep = constrainCyclic(arpStep + 1, 0, 3);
        else if (userConfig.arpMode == 2)
            arpStep = constrainCyclic(arpStep - 1, 0, 3);
        else
            arpStep = osc[0].getRandom16(4);
    }

    userConfig.oscACoarse = coarseA;

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
        for (int i = 0; i < 4; ++i)
        {
            // OLED描画更新でノイズが乗るので必要時以外更新しない
            if (btn0 != 3)
            {
                // OLED描画更新でノイズが乗るので必要時以外更新しない
                requiresUpdate |= osc[i].setNoteNameFromFrequency(coarseA);
                requiresUpdate |= osc[i].setWave((Oscillator::Wave)
                                                    constrainCyclic((int)osc[i].getWave() + (int)encValue, 0, (int)Oscillator::Wave::MAX));
                requiresUpdate |= osc[i].setFreqName(coarseA);

                userConfig.oscAWave = osc[i].getWave();
            }
            else if (btn0 == 3)
            {
                if (userConfig.oscAWave == Oscillator::Wave::SAW || userConfig.oscAWave == Oscillator::Wave::MUL_TRI)
                {
                    osc[i].addPhaseShift((int)encValue);
                    // osc[i].setPhaseShift(osc[i].getPhaseShift() + osc[i].getRandom16(5));
                    requiresUpdate |= userConfig.oscAPhaseShift != osc[i].getPhaseShift();
                    userConfig.oscAPhaseShift = osc[i].getPhaseShift();
                }
                else if (userConfig.oscAWave == Oscillator::Wave::TRI || userConfig.oscAWave == Oscillator::Wave::SINE)
                {
                    osc[i].addFolding((int)encValue);
                    requiresUpdate |= userConfig.oscAFolding != osc[i].getFolding();
                    userConfig.oscAFolding = osc[i].getFolding();
                }
            }
        }
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

    // OSCA CVでパラメータ変更
    if (userConfig.oscAParaCV > 0)
    {
        int16_t shift = map(userConfig.oscAParaCV == 1 ? cv1Value : cv2Value, 0, 4096, 0, 50);
        requiresUpdate |= osc[0].setFolding(shift) & (menuIndex == 0);
        requiresUpdate |= osc[0].setPhaseShift(shift) & (menuIndex == 0);
    }

    // max_coarse_freq = (float)VCO_MAX_COARSE_FREQ;

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

    sleep_us(1000); // 10kHz
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
