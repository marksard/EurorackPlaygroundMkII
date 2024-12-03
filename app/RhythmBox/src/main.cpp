/*!
 * Rhythm Box
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
#include "../../commonlib/common/TriggerOut.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "PatternSeq.hpp"
#include "SingleShotWave.hpp"

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
int pwmOuts[SEQUENCER_TOTAL] = { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6 };
bool isSDFill = false;

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 256
#define PWM_RESO 2048         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define SAMPLE_FREQ 44100 // 再生ファイルにあわせる
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// setting values
float pitches[SEQUENCER_TOTAL] = {1.0,1.0,1.0,1.0,1.0,1.0};
float decays[SEQUENCER_TOTAL] = {1.0,0.5,1.0,1.0,1.0,1.0};
float volumes[SEQUENCER_TOTAL] = {0.7,1.0,1.0,0.8,1.0,1.0};
float triggers[SEQUENCER_TOTAL] = {0, 0, 0, 0, 0, 0};
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

const char selTrigger[][5] = {"INT", "VOCT", "CV1", "CV2"};

SettingItemF trigSettings[] =
{
    SettingItemF(0.0, 3.0, 1.0, &triggers[0], "RC: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &triggers[1], "HH: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &triggers[2], "LT: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &triggers[3], "RM: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &triggers[4], "SD: %s", selTrigger, 4),
    SettingItemF(0.0, 3.0, 1.0, &triggers[5], "BD: %s", selTrigger, 4),
};

SettingItemF volumeSettings[] =
{
    SettingItemF(0.1, 1.0, 0.05, &volumes[0], "RC: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &volumes[1], "HH: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &volumes[2], "LT: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &volumes[3], "RM: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &volumes[4], "SD: %4.2f", NULL, 0),
    SettingItemF(0.1, 1.0, 0.05, &volumes[5], "BD: %4.2f", NULL, 0),
};

SettingItemF decaySettings[] =
{
    SettingItemF(0.01, 1.0, 0.01, &decays[0], "RC %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &decays[1], "HH %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &decays[2], "LT %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &decays[3], "RM %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &decays[4], "SD %4.2f", NULL, 0),
    SettingItemF(0.01, 1.0, 0.01, &decays[5], "BD %4.2f", NULL, 0),
};

SettingItemF pitchSettings[] =
{
    SettingItemF(0.1, 2.0, 0.01, &pitches[0], "RC: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &pitches[1], "HH: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &pitches[2], "LT: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &pitches[3], "RM: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &pitches[4], "SD: %4.2f", NULL, 0),
    SettingItemF(0.1, 2.0, 0.01, &pitches[5], "BD: %4.2f", NULL, 0),
};

static MenuSectionF menu[] = {
    {"TRIGGER", trigSettings, sizeof(trigSettings) / sizeof(trigSettings[0])},
    {"VOLUME", volumeSettings, sizeof(volumeSettings) / sizeof(volumeSettings[0])},
    {"DECAY", decaySettings, sizeof(decaySettings) / sizeof(decaySettings[0])},
    {"PITCH", pitchSettings, sizeof(pitchSettings) / sizeof(pitchSettings[0])}
};

static MenuControlF menuControl(menu, sizeof(menu) / sizeof(menu[0]));


static PatternSeq seq;
static EdgeChecker clockEdge;
static bool clockAlive = false;
static int16_t clockCount = 0;
// 1小節4打の16ステップを4つ=16小節
static int16_t resetCount = STEP_TOTAL;

void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
}

void dispOLED()
{
    if (menuIndex < 6) {
        seq.updateDisplay(&u8g2, clockCount & (STEP_MAX - 1), menuIndex, encMode, requiresUpdate);
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
    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    for (int i = 0; i < SEQUENCER_TOTAL; ++i)
    {
        pwm_set_gpio_level(pwmOuts[i], pKit[i]->updateWave());
    }

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
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[1].setHoldTime(5);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    clockEdge.init(GATE);

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
    seq.setPattern(0, 3);
    seq.setPattern(1, 16);
    seq.setPattern(2, 19);
    seq.setPattern(3, 26);
    seq.setPattern(4, 25);
    seq.setPattern(5, 32);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    bool acc = encMode && menuIndex >= SEQUENCER_TOTAL ? true : false;
    enc.getDirection(acc);
    uint16_t voct = vOct.analogReadDirectFast();
    int16_t cv1Value = cv1.analogReadDirectFast();
    uint16_t cv2Value = cv2.analogReadDirectFast();
    int8_t triggerVOct = voct > 2048;
    int8_t triggerCV1 = cv1Value > 2048;
    int8_t triggerCV2 = cv2Value > 2048;

    bool trig = clockEdge.isEdgeHigh();

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
        pKit[i]->setSpeed(pitches[i]);
        pKit[i]->updateDecay(decays[i]);
        pKit[i]->setVolume(volumes[i]);
        if (triggers[i] == 1)
        {
            pKit[i]->play(triggerVOct);
            continue;
        }
        else if (triggers[i] == 2)
        {
            pKit[i]->play(triggerCV1);
            continue;
        }
        else if (triggers[i] == 3)
        {
            pKit[i]->play(triggerCV2);
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

    if (!clockEdge.isAlive())
    {
        clockCount = 0;
        clockAlive = false;
    }

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    // Serial.print(voct);
    // Serial.print(", ");
    // Serial.print(cv1Value);
    // Serial.print(", ");
    // Serial.print(cv2Value);
    // Serial.print(", ");
    // Serial.print(gateValue);
    // Serial.println();
    // }

    sleep_us(100);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(60); // = 60sec / (250bpm * 4ppq)
    updateOLED.start();
}

void loop1()
{
    uint16_t potValue = pot.analogRead();
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

    if (btn0 == 1)
    {
        clockCount = 0;
    }

    if (menuIndex < SEQUENCER_TOTAL) {
        if (encMode)
        {
            requiresUpdate |= seq.addSelectPattern((menuIndex % SEQUENCER_TOTAL), encValue);
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
        if (i <= muteIndex)
        {
            if (mute == 4 && isSDFill)
            {
                continue;
            }

            pKit[mute]->setMute(true);
        }
        else
        {
            pKit[mute]->setMute(false);
        }
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    // カウンタリセット時点灯
    gpio_put(LED1, (clockCount & (STEP_MAX - 1)) == 0 ? HIGH : LOW);
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
