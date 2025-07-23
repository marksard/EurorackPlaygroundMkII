/*!
 * Rhythm Box
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
#include "lib/ActiveGainControl.hpp"

#include "PatternSeq.hpp"
#include "SingleShotWave.hpp"
#include "UserConfig.h"

// #include "wavetable/909_01/909_BD_Norm.h"
#include "wavetable/808_01/808_BD_02.h"
#include "wavetable/909_01/909_LT.h"
// #include "wavetable/909_01/909_OH.h"
#include "wavetable/808_01/808_OHH_01.h"
#include "wavetable/909_01/909_RC.h"
// #include "wavetable/909_01/909_RS.h"
#include "wavetable/808_01/808_Rim_01.h"
// #include "wavetable/909_01/909_SD_Snap.h"
#include "wavetable/808_01/808_SD_01.h"

SingleShotWave RC(buf_909_RC, buf_size_909_RC);
// SingleShotWave OH(buf_909_OH, buf_size_909_OH);
SingleShotWave OH(buf_808_OHH_01, buf_size_808_OHH_01);
SingleShotWave LT(buf_909_LT, buf_size_909_LT);
// SingleShotWave RS(buf_909_RS, buf_size_909_RS);
// SingleShotWave SD(buf_909_SD_Snap, buf_size_909_SD_Snap);
// SingleShotWave BD(buf_909_BD_Norm, buf_size_909_BD_Norm);
SingleShotWave RS(buf_808_Rim_01, buf_size_808_Rim_01);
SingleShotWave SD(buf_808_SD_01, buf_size_808_SD_01);
SingleShotWave BD(buf_808_BD_02, buf_size_808_BD_02);
SingleShotWave<int16_t> *pKit[SEQUENCER_TOTAL];
bool isSDFill = false;

#undef SAMPLE_FREQ
#define SAMPLE_FREQ 44100 // 再生ファイルにあわせる
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static EdgeChecker vOct;
static EdgeChecker cv1;
static EdgeChecker cv2;
static int pwmOuts[6] = { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6 };

// ユーザー設定
static EEPROMConfigIO<UserConfig> userConfig(0); 
static bool saveConfirm = false;

// 画面周り
#define MENU_MAX (SEQUENCER_TOTAL + 1)
static int16_t menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

typedef struct
{
    char title[16];
    SettingItemF items[24];
} SettingMenu;

const char selMute[][5] = {"----", "MUTE"};
const char selPan[][5] = {"L2", "L1", "C", "R1", "R2"};
const char selTrigger[][5] = {"INT", "VOCT", "CV1", "CV2"};
const char selIndication[][5] = {"OFF", "ON"};

SettingItemF settings[] =
{
    SettingItemF(0.0, 1.0, 1.00, &userConfig.Config.isUpdateStepIndicator, "STEP: %s", selIndication, 2),
    SettingItemF(0.0, 1.5, 0.02, &userConfig.Config.agcMaxGain, "GAIN: %4.2f", NULL, 0),
};

SettingItemF muteSettings[] =
{
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[0], "RC: %s", selMute, 2),
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[1], "HH: %s", selMute, 2),
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[2], "LT: %s", selMute, 2),
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[3], "RM: %s", selMute, 2),
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[4], "SD: %s", selMute, 2),
    SettingItemF(0.0, 1.0, 1.0, &userConfig.Config.mutes[5], "BD: %s", selMute, 2),
};

SettingItemF volumeSettings[] =
{
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[0], "RC: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[1], "HH: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[2], "LT: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[3], "RM: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[4], "SD: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &userConfig.Config.volumes[5], "BD: %4.2f", NULL, 0),
};

SettingItemF panSettings[] =
{
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[0], "RC: %s", selPan, 5),
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[1], "HH: %s", selPan, 5),
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[2], "LT: %s", selPan, 5),
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[3], "RM: %s", selPan, 5),
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[4], "SD: %s", selPan, 5),
    SettingItemF(0.0, 4.0, 1.0, &userConfig.Config.pans[5], "BD: %s", selPan, 5),
};

SettingItemF decaySettings[] =
{
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[0], "RC %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[1], "HH %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[2], "LT %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[3], "RM %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[4], "SD %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &userConfig.Config.decays[5], "BD %4.2f", NULL, 0),
};

SettingItemF pitchSettings[] =
{
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[0], "RC: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[1], "HH: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[2], "LT: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[3], "RM: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[4], "SD: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &userConfig.Config.pitches[5], "BD: %4.2f", NULL, 0),
};

SettingItemF trigSettings[] =
{
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[0], "RC: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[1], "HH: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[2], "LT: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[3], "RM: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[4], "SD: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &userConfig.Config.triggers[5], "BD: %s", selTrigger, 4),
};

static MenuSectionF menu[] = {
    {"SETTINGS", settings, sizeof(settings) / sizeof(settings[0])},
    {"MUTE", muteSettings, sizeof(muteSettings) / sizeof(muteSettings[0])},
    {"VOLUME", volumeSettings, sizeof(volumeSettings) / sizeof(volumeSettings[0])},
    {"PAN", panSettings, sizeof(panSettings) / sizeof(panSettings[0])},
    {"DECAY", decaySettings, sizeof(decaySettings) / sizeof(decaySettings[0])},
    {"PITCH", pitchSettings, sizeof(pitchSettings) / sizeof(pitchSettings[0])},
    {"TRIGGER", trigSettings, sizeof(trigSettings) / sizeof(trigSettings[0])},
};

static MenuControlF menuControl(menu, sizeof(menu) / sizeof(menu[0]));


static PatternSeq seq;
static EdgeChecker clockEdge;
static bool clockAlive = false;
static int16_t clockCount = 0;
// 1小節4打の16ステップを4つ=16小節
static int16_t resetCount = STEP_TOTAL;
static ActiveGainControl agc;

void initOLED()
{
    u8g2.setBusClock(400000);
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
}

void saveDisp()
{
    static char disp_buf[33] = {0};
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
}

void dispOLED()
{
    if (menuIndex < 6) {
        seq.updateDisplay(&u8g2, clockCount & (STEP_MAX - 1), menuIndex, encMode, 
        requiresUpdate, userConfig.Config.isUpdateStepIndicator > 0.1 ? true : false);
        if (saveConfirm)
        {
            saveDisp();
            u8g2.sendBuffer();
        }
        requiresUpdate = 0;
        return;
    }

    if (!requiresUpdate)
    {
        return;
    }

    u8g2.clearBuffer();
    requiresUpdate = 0;
    menuControl.draw(&u8g2, encMode);
    saveDisp();
    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    int16_t levelL = 0;
    int16_t levelR = 0;
    for (int i = 0; i < SEQUENCER_TOTAL; ++i)
    {
        int16_t level = pKit[i]->updateWave();
        uint16_t pan = (uint16_t)userConfig.Config.pans[i];
        uint16_t panL = pan <= 2 ? 0 : pan - 2;
        uint16_t panR = pan >= 2 ? 0 : 2 - pan;
        levelL += level >> panL;
        levelR += level >> panR;
    }
    agc.setCurrentLevel(levelL, levelR);
    agc.update(1);

    levelL = agc.getProcessedLevel(levelL);
    levelR = agc.getProcessedLevel(levelR);

    pwm_set_gpio_level(OUT1, levelL);
    pwm_set_gpio_level(OUT2, levelR);
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
    buttons[1].init(BTN2);
    buttons[1].setHoldTime(5);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    clockEdge.init(GATE);
    // gain最大値の設定は全部同時に鳴ることはほぼないため大きめにしている
    agc.init(PWM_RESO, SEQUENCER_TOTAL, 0.9);

    userConfig.initEEPROM();
    userConfig.loadUserConfig();

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    pKit[0] = &RC;
    pKit[1] = &OH;
    pKit[2] = &LT;
    pKit[3] = &RS;
    pKit[4] = &SD;
    pKit[5] = &BD;
    seq.setPatternName(0, "RC");
    seq.setPatternName(1, "HH");
    seq.setPatternName(2, "LT");
    seq.setPatternName(3, "RM");
    seq.setPatternName(4, "SD");
    seq.setPatternName(5, "BD");
    seq.setPattern(0, (uint8_t)userConfig.Config.pattern[0]);
    seq.setPattern(1, (uint8_t)userConfig.Config.pattern[1]);
    seq.setPattern(2, (uint8_t)userConfig.Config.pattern[2]);
    seq.setPattern(3, (uint8_t)userConfig.Config.pattern[3]);
    seq.setPattern(4, (uint8_t)userConfig.Config.pattern[4]);
    seq.setPattern(5, (uint8_t)userConfig.Config.pattern[5]);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    bool acc = encMode && menuIndex >= SEQUENCER_TOTAL ? true : false;
    enc.getDirection(acc);
    bool voctValue = vOct.isEdgeHigh();
    bool cv1Value = cv1.isEdgeHigh();
    bool cv2Value = cv2.isEdgeHigh();
    bool trig = clockEdge.isEdgeHigh();

    agc.setGainMax(userConfig.Config.agcMaxGain);

    if (trig)
    {
        if (clockAlive)
        {
            clockCount++;
            clockCount = clockCount % resetCount;
        }
        clockAlive = true;
    }

    for (int i = 0; i < SEQUENCER_TOTAL; ++i)
    {
        pKit[i]->setSpeed(userConfig.Config.pitches[i]);
        pKit[i]->setVolume(userConfig.Config.volumes[i]);
        pKit[i]->updateDecay(userConfig.Config.decays[i]);
        if (userConfig.Config.triggers[i] == 1)
        {
            pKit[i]->play(voctValue);
            continue;
        }
        else if (userConfig.Config.triggers[i] == 2)
        {
            pKit[i]->play(cv1Value);
            continue;
        }
        else if (userConfig.Config.triggers[i] == 3)
        {
            pKit[i]->play(cv2Value);
            continue;
        }

        int state = trig;
        int beat = seq.getBeat(i, clockCount);
        if (i == 4 && isSDFill)
        {
            beat = 0xF;
        }
        beat = (beat >> (clockCount >> 4)) & 1;
        state = state & beat;
        pKit[i]->play(state);
    }
    
    if (!saveConfirm)
    {
        buttons[1].setHoldTime(5);
        uint8_t btn1 = buttons[1].getState();
        if (btn1 == 1 || btn1 == 3)
        {
            pKit[4]->setMute(false);
            isSDFill = true;
        }
        else
        {
            isSDFill = false;
        }
    }

    if (!clockEdge.isAlive())
    {
        clockCount = 0;
        clockAlive = false;
    }

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     // Serial.print(voct);
    //     // Serial.print(", ");
    //     // Serial.print(cv1Value);
    //     // Serial.print(", ");
    //     // Serial.print(cv2Value);
    //     // Serial.print(", ");
    //     // Serial.print(gateValue);
    //     // Serial.println();
    //     // agc.print();
    // }

    tight_loop_contents();
    sleep_us(100);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    uint16_t potValue = pot.analogReadDropLow4bit();
    int8_t encValue = enc.getValue();
    uint8_t btn0 = buttons[0].getState();
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

    // ec長押しで設定保存
    if (btn2 == 4)
    {
        saveConfirm = true;
        requiresUpdate |= 1;
    }
    if (saveConfirm)
    {
        buttons[1].setHoldTime(500);
        uint8_t btn1 = buttons[1].getState();
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
    else if (btn0 == 1)
    {
        clockCount = 0;
    }

    if (menuIndex < SEQUENCER_TOTAL) {
        if (encMode)
        {
            requiresUpdate |= seq.addSelectPattern(menuIndex, encValue);
            userConfig.Config.pattern[menuIndex] = seq.getPattern(menuIndex);
        }
    }
    else
    {
        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
    }

    int8_t muteIndex = map(potValue, 0, 3800, 0, SEQUENCER_TOTAL);
    int8_t mutes[] = {-1, 0, 2, 3, 4, 1, 5};
    for (int i = 0; i < 7; ++i)
    {
        int8_t mute = mutes[i];
        if (mute == -1)continue;
        if (mute == 4 && isSDFill)continue;
        if (i <= muteIndex)
        {
            pKit[mute]->setMute(true);
        }
        else
        {
            pKit[mute]->setMute(userConfig.Config.mutes[mute] > 0.8 ? true : false);
        }
    }

    if (!updateOLED.ready())
    {
        tight_loop_contents();
        // sleep_ms(1);
        return;
    }

    // カウンタリセット時点灯
    // int16_t div16 = (clockCount % STEP_MAX) < (STEP_MAX >> 1) ? 1 : 0;
    int16_t div4 = (clockCount % 4) < (4 >> 1) ? 1 : 0;
    gpio_put(LED1, div4 ? HIGH : LOW);
    gpio_put(LED2, clockCount == 0 ? HIGH : LOW);

    dispOLED();
    // sleep_ms(1);

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    // Serial.print(potValue);
    // Serial.print(", ");
    // Serial.print(encValue);
    // Serial.print(", ");
    // Serial.print(btn0);
    // Serial.print(", ");
    // Serial.print(btn1);
    // Serial.print(", ");
    // Serial.print(btn2);
    // Serial.print(", ");
    // Serial.print(menuIndex);
    // Serial.println();
    // }
}
