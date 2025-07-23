/*!
 * Chord VCO
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
#include "lib/EdgeChecker.hpp"
#include "lib/RandomFast.hpp"
#include "lib/Quantizer.hpp"
#include "lib/ActiveGainControl.hpp"

#include "MultiWaveOscEx.hpp"
#include "UserConfig.h"

#undef SAMPLE_FREQ
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8) // 32470.703125khz

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
static int pwmOuts[] = {OUT1, OUT2, OUT3, OUT4, OUT5, OUT6};
static uint8_t pwmOutsCount = sizeof(pwmOuts) / sizeof(pwmOuts[0]);
static ADCErrorCorrection adcErrorCorrection(3.3);

// ユーザー設定
static EEPROMConfigIO<UserConfig> userConfig(0); 
static bool saveConfirm = false;

// VCO
#define VCO_MAX_ROOT_INDEX 96 // noteNameのC7
// #define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO - 1)))) - 1) / (exp(ratio) - 1)
#define OSCILLATOR_MAX 4
static MultiWaveOscEx osc[OSCILLATOR_MAX];
static int8_t arpStep = 0;
static Quantizer quantizer(PWM_RESO);
static ActiveGainControl agc;
static uint16_t bias = PWM_RESO >> 1;
static RandomFast randFast;
static const float voctIndexRatio = 60.0f / (float)(ADC_RESO - 1);

// コードを構成する音の12音階上での位置
static uint8_t addRootScale[][7][OSCILLATOR_MAX] =
{
    // natulal minor chord
    {
        {0, 3, 7, 10}, // Im7
        {0, 3, 6, 10}, // IIdim
        {0, 4, 7, 11}, // IIIM7
        {0, 3, 7, 10}, // IVm7
        {0, 3, 7, 10}, // Vm7
        {0, 4, 7, 11}, // VIM7
        {0, 4, 7, 10}, // VII7
    },
    // major chord
    {
        {0, 4, 7, 11}, // IM7
        {0, 3, 7, 10}, // IIm7
        {0, 3, 7, 10}, // IIIm7
        {0, 4, 7, 11}, // IVM7
        {0, 4, 7, 10}, // V7
        {0, 3, 7, 10}, // VIm7
        {0, 3, 6, 10}, // VIIdim7
    }
};

// ルート起点としてノートの位置による構成コード（addRootScale）の指定
// ※キーからはずれたノートはとりあえず0にしている
static uint8_t rootScaleIndexFromSemitone[][12] =
{
    // natulal minor chord
    {0, 0, 1, 2, 0, 3, 0, 4, 5, 0, 6, 0},
    // major chord
    {0, 0, 1, 0, 2, 3, 0, 4, 0, 5, 0, 6}
};


// 画面周り
#define MENU_MAX (1 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

const char selCV[][5] = {"---", "CV1", "CV2"};
const char selOct[][5] = {"---", "12", "24"};
const char selArp[][5] = {"---", "UP", "DOWN", "RAND"};
const char selHold[][5] = {"FREE", "GATE"};
const char scales[][5] = {"MIN", "MAJ"};

SettingItem16 commonSettings[] =
    {
        SettingItem16(0, 2, 1, &userConfig.Config.rootMinus, "ROOT: %s", selOct, 3),
        SettingItem16(0, 2, 1, &userConfig.Config.seventhMinus, "7TH: %s", selOct, 3),
        SettingItem16(0, 2, 1, &userConfig.Config.oscAParaCV, "ParamCV: %s", selCV, 3),
        SettingItem16(0, 3, 1, &userConfig.Config.arpMode, "OUT2 ARP: %s", selArp, 4),
        SettingItem16(0, 1, 1, &userConfig.Config.voctHold, "HOLD MODE: %s", selHold, 2),
        SettingItem16(0, 1, 1, &userConfig.Config.scale, "SCALE: %s", scales, 2),
        SettingItem16(-200, 200, 1, &userConfig.Config.voctTune, "INIT VOCT:%4d", NULL, 0)};

SettingItem16 quantizerSettings[] =
    {
        SettingItem16(0, 2, 1, &userConfig.Config.quantizeCV, "INPUT: %s", selCV, 3),
        SettingItem16(0, quantizer.MaxScales, 1, &userConfig.Config.quantizeScale, "SCALE: %s", quantizer.ScaleNames, quantizer.MaxScales),
        SettingItem16(1, 5, 1, &userConfig.Config.quantizeOct, "OCT:%2d", NULL, 0),
        SettingItem16(0, 1, 1, &userConfig.Config.quantizeHold, "HOLD MODE: %s", selHold, 2),
};

static MenuSection16 menu[] = {
    {"CHORD VCO", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
    {"QUANTIZER", quantizerSettings, sizeof(quantizerSettings) / sizeof(quantizerSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

void drawOSC()
{
    static char disp_buf[33] = {0};
    u8g2.setFont(u8g2_font_7x14B_tf);

    sprintf(disp_buf, "Chord VCO: %s", scales[userConfig.Config.scale]);
    u8g2.drawStr(0, 0, disp_buf);

    if (osc[0].getWave() == MultiWaveOscEx::Wave::SAW || osc[0].getWave() == MultiWaveOscEx::Wave::MUL_TRI)
    {
        if (userConfig.Config.oscAParaCV > 0)
            sprintf(disp_buf, "Root:%s Phs:cv%d", osc[0].getNoteName(), userConfig.Config.oscAParaCV);
        else
            sprintf(disp_buf, "Root:%s Phs:%02d", osc[0].getNoteName(), userConfig.Config.oscAPhaseShift);
    }
    else if (osc[0].getWave() == MultiWaveOscEx::Wave::TRI ||
             osc[0].getWave() == MultiWaveOscEx::Wave::SINE)
    {
        if (userConfig.Config.oscAParaCV > 0)
            sprintf(disp_buf, "Root:%s Fld:cv%d", osc[0].getNoteName(), userConfig.Config.oscAParaCV);
        else
            sprintf(disp_buf, "Root:%s Fld:%02d", osc[0].getNoteName(), userConfig.Config.oscAFolding);
    }
    else
    {
        sprintf(disp_buf, "Root:%s", osc[0].getNoteName());
    }
    u8g2.drawStr(0, 48, disp_buf);
    u8g2.setFont(u8g2_font_logisoso26_tf);
    sprintf(disp_buf, "%s", osc[0].getWaveName());
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
        drawOSC();
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

    int16_t sum = 0;
    int16_t values[OSCILLATOR_MAX] = {0};
    for (int i = 0; i < OSCILLATOR_MAX; ++i)
    {
        values[i] = osc[i].getWaveValue();
        sum += values[i] - bias;
    }

    agc.setCurrentLevel(sum);
    pwm_set_gpio_level(OUT1, agc.getProcessedLevel(sum));
    pwm_set_gpio_level(OUT2, values[arpStep]);
    // pwm_set_gpio_level(OUT6, randFast.getRandom16(PWM_RESO));
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
    buttons[0].init(BTN1, false);
    buttons[0].setHoldTime(250);
    buttons[1].init(BTN2, false);
    buttons[1].setHoldTime(250);
    buttons[2].init(BTN3, false);
    gate.init(GATE);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    agc.init(PWM_RESO, OSCILLATOR_MAX, 0.35);

    userConfig.initEEPROM();
    userConfig.loadUserConfig();

    for (int i = 0; i < OSCILLATOR_MAX; ++i)
    {
        osc[i].init(SAMPLE_FREQ);
        osc[i].setWave((MultiWaveOscEx::Wave)userConfig.Config.oscAWave);
        osc[i].setFrequencyFromNoteNameIndex(userConfig.Config.oscACoarseIndex);
        osc[i].setCourceFromNoteNameIndex(userConfig.Config.oscACoarseIndex);
        osc[i].setPhaseShift(userConfig.Config.oscAPhaseShift);
        osc[i].setFolding(userConfig.Config.oscAFolding);
        osc[i].startFolding(true);
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
    for (int i = 0; i < pwmOutsCount; ++i)
    {
        slice |= 0x01 << pwm_gpio_to_slice_num(pwmOuts[i]);
    }
    pwm_set_mask_enabled(slice);

    adcErrorCorrection.init(3.3, 20.0);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    pot.analogRead(true);
    enc.getDirection();
    int16_t voct = vOct.analogRead(false);
    int16_t cv1Value = cv1.analogReadDirectFast();
    int16_t cv2Value = cv2.analogReadDirectFast();

    agc.update(3);

    static int16_t voctIndex = 0;
    int16_t tempVoctIndex = (int16_t)(adcErrorCorrection.correctedAdc(voct) * voctIndexRatio + 0.5f);

    if (tempVoctIndex != voctIndex)
    {
        arpStep = 0;
        voctIndex = tempVoctIndex;
    }

    uint8_t rootDiff = (userConfig.Config.oscACoarseIndex + (int)voctIndex) % 12;
    uint8_t scaleIndex = rootScaleIndexFromSemitone[userConfig.Config.scale][rootDiff];
    int8_t rootMinus = userConfig.Config.rootMinus * -12;
    int8_t seventhMinus = userConfig.Config.seventhMinus * 12;

    // arpeggio
    if (gate.isEdgeHigh())
    {
        if (userConfig.Config.arpMode == 0)
            arpStep = 0;
        else if (userConfig.Config.arpMode == 1)
            arpStep = constrainCyclic(arpStep + 1, 0, 3);
        else if (userConfig.Config.arpMode == 2)
            arpStep = constrainCyclic(arpStep - 1, 0, 3);
        else
            arpStep = randFast.getRandom16(4);
    }

    if (userConfig.Config.voctHold == 0 || (userConfig.Config.voctHold == 1 && gate.getValue()))
    {
        osc[0].setFreqFromNoteIndex(userConfig.Config.oscACoarseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][0] + rootMinus);
        osc[1].setFreqFromNoteIndex(userConfig.Config.oscACoarseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][1]);
        osc[2].setFreqFromNoteIndex(userConfig.Config.oscACoarseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][2]);
        osc[3].setFreqFromNoteIndex(userConfig.Config.oscACoarseIndex + voctIndex + addRootScale[userConfig.Config.scale][scaleIndex][3] + seventhMinus);
    }

    if (userConfig.Config.quantizeHold == 0 || (userConfig.Config.quantizeHold == 1 && gate.getValue()))
    {
        if (userConfig.Config.quantizeCV > 0)
        {
            quantizer.setScale(userConfig.Config.quantizeScale);
            uint16_t semi = map(userConfig.Config.quantizeCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, 0, (7 * userConfig.Config.quantizeOct));
            pwm_set_gpio_level(OUT5, constrain(quantizer.Quantize(semi) - PWMCVDCOutputErrorLUT[semi], 0, PWM_RESO - 1));
        }
    }

    // OSCA CVでパラメータ変更
    if (userConfig.Config.oscAParaCV > 0)
    {
        int16_t shift = map(userConfig.Config.oscAParaCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, 0, 50);
        requiresUpdate |= osc[0].setFolding(shift) & (menuIndex == 0);
        requiresUpdate |= osc[0].setPhaseShift(shift) & (menuIndex == 0);
    }

    sleep_us(250);
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
    // uint16_t voct = vOct.getValue();
    // int16_t cv1Value = cv1.getValue();
    // int16_t cv2Value = cv2.getValue();

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
            userConfig.Config.oscACoarseIndex = map(potValue, 0, ADC_RESO - 1, 0, VCO_MAX_ROOT_INDEX);
        }
        for (int i = 0; i < OSCILLATOR_MAX; ++i)
        {
            // OLED描画更新でノイズが乗るので必要時以外更新しない
            if (btn0 != 3)
            {
                // OLED描画更新でノイズが乗るので必要時以外更新しない
                requiresUpdate |= osc[i].setCourceFromNoteNameIndex(userConfig.Config.oscACoarseIndex);
                requiresUpdate |= osc[i].setWave((MultiWaveOscEx::Wave)
                                                     constrainCyclic((int)osc[i].getWave() + (int)encValue, 0, (int)MultiWaveOscEx::Wave::MAX));

                userConfig.Config.oscAWave = osc[i].getWave();
                pwm_set_gpio_level(LED1, 0);
            }
            else if (btn0 == 3)
            {
                if (userConfig.Config.oscAWave == MultiWaveOscEx::Wave::SAW || userConfig.Config.oscAWave == MultiWaveOscEx::Wave::MUL_TRI)
                {
                    osc[i].addPhaseShift((int)encValue);
                    // osc[i].setPhaseShift(osc[i].getPhaseShift() + osc[i].getRandom16(5));
                    requiresUpdate |= userConfig.Config.oscAPhaseShift != osc[i].getPhaseShift();
                    userConfig.Config.oscAPhaseShift = osc[i].getPhaseShift();
                }
                else if (userConfig.Config.oscAWave == MultiWaveOscEx::Wave::TRI || userConfig.Config.oscAWave == MultiWaveOscEx::Wave::SINE)
                {
                    osc[i].addFolding((int)encValue);
                    requiresUpdate |= userConfig.Config.oscAFolding != osc[i].getFolding();
                    userConfig.Config.oscAFolding = osc[i].getFolding();
                }
                pwm_set_gpio_level(LED1, PWM_RESO -1);
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
        sleep_ms(1);
        return;
    }

    dispOLED();
}
