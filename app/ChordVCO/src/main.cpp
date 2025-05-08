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
#include "../../commonlib/common/ActiveGainControl.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "../../commonlib/common/Quantizer.hpp"
#include "Oscillator.hpp"
#include "EepromData.h"
#include "SmoothRandomCV.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
// #define PWM_RESO 4096         // 12bit
#define PWM_RESO 2048 // 11bit
// #define PWM_RESO 1024         // 10bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
// #define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 4) // 64941.40625khz
// #define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 6) // 43294.27khz
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
static int pwmOuts[6] = {OUT1, OUT2, OUT3, OUT4, OUT5, OUT6};

// ユーザー設定
static UserConfig userConfig;
static bool saveConfirm = false;

// triple vco
#define VCO_MAX_ROOT_INDEX 96 // noteNameのC7
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO - 1)))) - 1) / (exp(ratio) - 1)
#define OSCILLATOR_MAX 4
static Oscillator osc[OSCILLATOR_MAX];
static int8_t arpStep = 0;
static Quantizer quantizer(PWM_RESO);
static ActiveGainControl agc;
static uint16_t bias = PWM_RESO >> 1;

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
// static SmoothRandomCV smoothRand(ADC_RESO);

SettingItem16 commonSettings[] =
    {
        SettingItem16(0, 2, 1, &userConfig.rootMinus, "ROOT: %s", selOct, 3),
        SettingItem16(0, 2, 1, &userConfig.seventhMinus, "7TH: %s", selOct, 3),
        SettingItem16(0, 2, 1, &userConfig.oscAParaCV, "ParamCV: %s", selCV, 3),
        SettingItem16(0, 3, 1, &userConfig.arpMode, "OUT2 ARP: %s", selArp, 4),
        SettingItem16(0, 1, 1, &userConfig.voctHold, "HOLD MODE: %s", selHold, 2),
        SettingItem16(0, 1, 1, &userConfig.scale, "SCALE: %s", scales, 2),
        SettingItem16(-200, 200, 1, &userConfig.voctTune, "INIT VOCT:%4d", NULL, 0)};

SettingItem16 quantizerSettings[] =
    {
        SettingItem16(0, 2, 1, &userConfig.quantizeCV, "INPUT: %s", selCV, 3),
        SettingItem16(0, quantizer.MaxScales, 1, &userConfig.quantizeScale, "SCALE: %s", quantizer.ScaleNames, quantizer.MaxScales),
        SettingItem16(1, 5, 1, &userConfig.quantizeOct, "OCT:%2d", NULL, 0),
        SettingItem16(0, 1, 1, &userConfig.quantizeHold, "HOLD MODE: %s", selHold, 2),
};

static MenuSection16 menu[] = {
    {"CHORD VCO", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
    {"QUANTIZER", quantizerSettings, sizeof(quantizerSettings) / sizeof(quantizerSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

void drawOSC()
{
    static char disp_buf[33] = {0};
    // u8g2.setFont(u8g2_font_VCR_OSD_tf);
    u8g2.setFont(u8g2_font_7x14B_tf);

    sprintf(disp_buf, "Chord VCO: %s", scales[userConfig.scale]);
    u8g2.drawStr(0, 0, disp_buf);

    if (osc[0].getWave() == Oscillator::Wave::SAW || osc[0].getWave() == Oscillator::Wave::MUL_TRI)
    {
        if (userConfig.oscAParaCV > 0)
            sprintf(disp_buf, "Root:%s Phase:cv%d", osc[0].getNoteName(), userConfig.oscAParaCV);
        else
            sprintf(disp_buf, "Root:%s Phase:%02d", osc[0].getNoteName(), osc[0].getPhaseShift());
    }
    else if (osc[0].getWave() == Oscillator::Wave::TRI ||
             osc[0].getWave() == Oscillator::Wave::SINE)
    {
        if (userConfig.oscAParaCV > 0)
            sprintf(disp_buf, "Root:%s Fold:cv%d", osc[0].getNoteName(), userConfig.oscAParaCV);
        else
            sprintf(disp_buf, "Root:%s Fold:%02d", osc[0].getNoteName(), osc[0].getFolding());
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
    // gpio_put(LED1, HIGH);

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
    pwm_set_gpio_level(OUT6, osc[0].getRandom16(PWM_RESO));
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
    buttons[0].init(BTN1, false);
    buttons[1].init(BTN2, false);
    buttons[2].init(BTN3, false);
    gate.init(GATE);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    agc.init(PWM_RESO, OSCILLATOR_MAX, 0.35);

    initEEPROM();
    loadUserConfig(&userConfig);

    for (int i = 0; i < OSCILLATOR_MAX; ++i)
    {
        osc[i].init(SAMPLE_FREQ);
        osc[i].setWave((Oscillator::Wave)userConfig.oscAWave);
        osc[i].setFrequencyFromNoteNameIndex(userConfig.oscACoarseIndex);
        osc[i].setCourceFromNoteNameIndex(userConfig.oscACoarseIndex);
        osc[i].addPhaseShift(userConfig.oscAPhaseShift);
        osc[i].addFolding(userConfig.oscAFolding);
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
    for (int i = 0; i < 6; ++i)
    {
        slice |= 0x01 << pwm_gpio_to_slice_num(pwmOuts[i]);
    }
    pwm_set_mask_enabled(slice);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    uint16_t potValue = pot.analogRead(true, true);
    int8_t encValue = enc.getDirection();
    uint16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    int16_t cv2Value = cv2.analogReadDirectFast();

    agc.update(3);

    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - 1 - userConfig.voctTune, 0, DAC_MAX_MILLVOLT - 1) * 0.001);

    static uint8_t rootIndex = 0;
    static uint8_t rootConfirmCount = 0;
    float freq = osc[0].getCource() * powVOct;
    uint8_t lastRootIndex = osc[0].getNoteNameIndexFromFreq(freq);

    if (lastRootIndex != rootIndex)
    {
        rootConfirmCount++;
        if (rootConfirmCount == 2)
        {
            arpStep = 0;
            rootIndex = lastRootIndex;
            rootConfirmCount = 0;
        }
    }
    else
    {
        rootConfirmCount = 0;
    }

    uint8_t rootDiff = (rootIndex - userConfig.oscACoarseIndex) % 12;
    uint8_t scaleIndex = rootScaleIndexFromSemitone[userConfig.scale][rootDiff];
    int8_t rootMinus = userConfig.rootMinus * -12;
    int8_t seventhMinus = userConfig.seventhMinus * 12;

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

    if (userConfig.voctHold == 0 || (userConfig.voctHold == 1 && gate.getEdge()))
    {
        osc[0].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[userConfig.scale][scaleIndex][0] + rootMinus);
        osc[1].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[userConfig.scale][scaleIndex][1]);
        osc[2].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[userConfig.scale][scaleIndex][2]);
        osc[3].setFrequencyFromNoteNameIndex(rootIndex + addRootScale[userConfig.scale][scaleIndex][3] + seventhMinus);
    }

    if (userConfig.quantizeHold == 0 || (userConfig.quantizeHold == 1 && gate.getEdge()))
    {
        if (userConfig.quantizeCV > 0)
        {
            quantizer.setScale(userConfig.quantizeScale);
            uint16_t cv = map(userConfig.quantizeCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, 0, (7 * userConfig.quantizeOct));
            pwm_set_gpio_level(OUT5, quantizer.Quantize(cv));
        }
    }

    // OSCA CVでパラメータ変更
    if (userConfig.oscAParaCV > 0)
    {
        int16_t shift = map(userConfig.oscAParaCV == 1 ? cv1Value : cv2Value, 0, ADC_RESO - 1, 0, 50);
        requiresUpdate |= osc[0].setFolding(shift) & (menuIndex == 0);
        requiresUpdate |= osc[0].setPhaseShift(shift) & (menuIndex == 0);
    }

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(userConfig.oscACoarseIndex);
    //     Serial.print(", ");
    //     Serial.print(rootIndex);
    //     Serial.print(", ");
    //     Serial.print(lastRootIndex);
    //     Serial.print(", ");
    //     Serial.print(rootConfirmCount);
    //     Serial.print(", ");
    //     Serial.print(freq);
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
    uint16_t potValue = pot.getValue();
    int8_t encValue = enc.getValue();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();
    uint16_t voct = vOct.getValue();
    int16_t cv1Value = cv1.getValue();
    int16_t cv2Value = cv2.getValue();

    static uint16_t lastPotValue = potValue;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    // pwm_set_gpio_level(LED1, cv1Value);
    // pwm_set_gpio_level(LED2, gate.getValue() ? 0 : PWM_RESO - 1);

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
            userConfig.oscACoarseIndex = map(potValue, 0, ADC_RESO - 1, 0, VCO_MAX_ROOT_INDEX);
        }
        for (int i = 0; i < OSCILLATOR_MAX; ++i)
        {
            // OLED描画更新でノイズが乗るので必要時以外更新しない
            if (btn0 != 3)
            {
                // OLED描画更新でノイズが乗るので必要時以外更新しない
                requiresUpdate |= osc[i].setCourceFromNoteNameIndex(userConfig.oscACoarseIndex);
                requiresUpdate |= osc[i].setWave((Oscillator::Wave)
                                                     constrainCyclic((int)osc[i].getWave() + (int)encValue, 0, (int)Oscillator::Wave::MAX));

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

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
    // sleep_ms(1);
}
