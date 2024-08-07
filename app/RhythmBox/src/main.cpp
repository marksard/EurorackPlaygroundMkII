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
#include "SingleShotWave.hpp"

#include "wavetable/909_01/909_BD_Norm.h"
#include "wavetable/909_01/909_LT.h"
#include "wavetable/909_01/909_OH.h"
#include "wavetable/909_01/909_RC.h"
#include "wavetable/909_01/909_RS.h"
#include "wavetable/909_01/909_SD_Snap.h"

SingleShotWave RC(buf_909_RC, buf_size_909_RC);
SingleShotWave OH(buf_909_OH, buf_size_909_OH);
SingleShotWave LT(buf_909_LT, buf_size_909_LT);
SingleShotWave RS(buf_909_RS, buf_size_909_RS);
SingleShotWave SD(buf_909_SD_Snap, buf_size_909_SD_Snap);
SingleShotWave BD(buf_909_BD_Norm, buf_size_909_BD_Norm);
#define VOICE_MAX 6
SingleShotWave<int16_t> *pKit[VOICE_MAX] = { &RC, &OH, &LT, &RS, &SD, &BD };
int pwmOuts[VOICE_MAX] = { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6 };
bool isSDFill = false;

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
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
float pitches[VOICE_MAX] = {1.0,1.0,1.0,1.0,1.0,1.0};
float decays[VOICE_MAX] = {1.0,1.0,1.0,1.0,1.0,1.0};
float volumes[VOICE_MAX] = {1.0,1.0,1.0,1.0,1.0,1.0};

// 画面周り
#define MENU_MAX (18+6)
static int16_t menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

typedef struct
{
    char title[16];
    SettingItemF items[18];
} SettingMenu;

SettingMenu set = {
    "SETTINGS",
    {
        SettingItemF(0.1, 2.0, 0.01, &pitches[0], "RC Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[0], "RC Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[0], "RC Volume: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &pitches[1], "HH Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[1], "HH Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[1], "HH Volume: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &pitches[2], "LT Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[2], "LT Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[2], "LT Volume: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &pitches[3], "RM Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[3], "RM Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[3], "RM Volume: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &pitches[4], "SD Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[4], "SD Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[4], "SD Volume: %4.2f", NULL, 0),
        SettingItemF(0.1, 2.0, 0.01, &pitches[5], "BD Pitch: %4.2f", NULL, 0),
        SettingItemF(0.01, 1.0, 0.01, &decays[5], "BD Decay: %4.2f", NULL, 0),
        SettingItemF(0.1, 1.0, 0.05, &volumes[5], "BD Volume: %4.2f", NULL, 0),
    }};

static EdgeChecker clockEdge;
static uint8_t clockGate = 0;
static int16_t clockCount = 0;
// 1小節4打の16ステップを4つ=16小節
#define STEP_MAX 16
static int16_t resetCount = STEP_MAX * 4;
#define BEATS_TOTAL 41
static int16_t beatsSelects[VOICE_MAX] = {1,16,29,27,25,32};
// 0:休 1:打
// 1以降:4小節単位での休打切り替え。例：0b1010(10)→8小節目、16小節目
static uint8_t beats[BEATS_TOTAL][STEP_MAX]
{
    {1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1}, // 0
    {1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0},
    {1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0, 0},
    {0, 0, 1, 0,  0, 0, 1, 0,  0, 0, 1, 0,  0, 0, 1, 0},
    {0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0},
    {0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 1, 0, 1,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 1,  0, 1, 0, 0,  0, 0, 0, 0},
    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 1, 0, 1},

    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 10
    {1, 0, 1, 0,  1, 0, 1,10,  1,10, 1, 0,  1, 0, 1, 0},
    {1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 8,  1, 8, 1, 8},
    {0, 0, 1, 0,  0, 0, 1, 0,  0,10, 1, 0,  0, 8, 1, 0},
    {0, 8, 1, 0,  0, 0, 1, 0,  0, 0, 1, 0,  2, 0, 1, 0},
    {0, 0, 1, 0,  0, 1, 0, 8,  0, 0, 1, 0,  0, 8, 1, 0},
    {0, 0, 1, 0,  0, 0, 1, 0,  0, 0, 1, 0,  0, 0, 1,10},
    {1, 0, 1,10,  1, 0,10, 1,  0, 1, 0, 0,  1, 8, 0, 1},
    {0, 1, 0, 1,  0, 1, 0, 1,  0, 1, 0, 1,  0, 1,10, 1},
    {0, 1, 0, 1,  1,10, 0, 1,  0, 1, 1, 0,  1, 8, 0, 1},

    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 20
    {0, 0, 0, 0,  1, 0, 0, 1,  0, 1, 0, 0,  1, 0, 0, 0},
    {0, 0, 0, 0,  1, 0, 0, 1,  0, 8, 0, 0,  1, 0, 0, 0},
    {0, 0, 0, 0,  1, 0, 0, 1,  0, 8, 0, 0,  1, 0, 0,10},
    {0, 8, 0, 0,  1, 0, 0,10,  0, 8, 0, 0,  1, 0, 0, 0},
    {0, 0, 0, 0,  1, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0,10},
    {0, 8, 0, 0,  0, 0, 8, 0,  0, 8, 0, 0,  0, 0,10, 0},
    {0, 0,10, 0,  0, 8,10, 0,  0, 8, 0,10,  2, 8, 0,10},
    {0, 0, 0, 0,  0, 0, 0,10,  0,10,10, 0,  0, 0, 0,10},
    {0, 8, 0, 0,  0, 0, 0,10,  8,10, 0, 0,  0, 0, 8,10},

    {0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 30
    {1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0,10},
    {1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0, 0,  1, 0, 8, 0},
    {1, 0, 1, 0,  0, 0, 1, 0,  0, 0, 0, 1,  0, 0,10, 0},
    {1, 0, 8, 0,  0, 0, 0, 0,  0, 0, 1, 0,  0,10, 0, 0},
    {1, 0,10, 0,  0, 0, 0, 0,  0, 0, 1, 8,  0, 0, 0, 0},
    {1, 0, 0, 1,  0, 8, 0, 0,  0, 0, 1, 0,  0,10, 0, 8},
    {1, 0, 0, 1,  0, 0, 1, 0,  0, 0, 1, 0,  0, 0,10, 0},
    {1, 0, 0, 1,  1, 0, 1, 0,  1, 0, 1, 0,  0, 0,10, 8},
    {1, 0, 0, 1,  0, 0, 1, 0,  0, 1, 0, 0,  1, 0, 0, 8},

    {1, 0, 1, 0,  0, 1, 0, 0,  1, 0, 0, 1,  0, 0, 1, 8}, // 40
};

void updateDisplay(U8G2 *pU8g2, int8_t currentStep, int8_t selector, int8_t mode)
{
    static char disp_name[VOICE_MAX][3] = { "RC", "HH", "LT", "RC", "SD", "BD"};
    uint8_t origin_x = 15;
    uint8_t origin_y = 15;
    uint8_t seqXStep = 7;
    uint8_t seqYStep = 8;

    pU8g2->setFont(u8g2_font_7x14B_tf);
    pU8g2->drawStr(0, 0, "PATTERN SEQ");
    pU8g2->setFont(u8g2_font_5x8_tf);

    for (int8_t i = 0; i < 6 + 1; ++i)
    {
        pU8g2->drawStr(0, origin_y + (seqYStep * i), disp_name[i]);
        pU8g2->drawHLine(origin_x, origin_y + (seqYStep * i), 127);
    }

    for (int8_t i = 0; i < STEP_MAX + 1; ++i)
    {
        pU8g2->drawVLine(origin_x + (seqXStep * i), origin_y, 63);
    }

    for (int8_t x = 0; x < STEP_MAX; ++x)
    {
        for (int8_t y = 0; y < 6; ++y)
        {
            uint8_t beat = beats[beatsSelects[y]][x];
            if((beat >> 1) & 1) beat = 2;
            else if((beat >> 2) & 1) beat = 3;
            else if((beat >> 3) & 1) beat = 4;
            int8_t size = 6 - beat;
            int8_t offset = 1;
            if (size < 6)
                pU8g2->drawBox((origin_x + offset) + (seqXStep * x), 
                            (origin_y + offset) + (seqYStep * y), 
                            size, size);
        }
    }

    pU8g2->drawBox(origin_x + (seqXStep * currentStep), origin_y - 2, seqXStep, 4);
    
    if (mode == 0)
        pU8g2->drawBox(10, (origin_y + 1) + (seqYStep * (selector % 6)), 4, 6);
    else
        pU8g2->drawStr(10, (origin_y + 1) + (seqYStep * (selector % 6)), ">");
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
    u8g2.clearBuffer();

    if (menuIndex < 6) {
        updateDisplay(&u8g2, clockCount & 0xF, menuIndex, encMode);
        u8g2.sendBuffer();
        return;
    }

    if (!requiresUpdate)
    {
        return;
    }

    requiresUpdate = 0;
    drawSetting(&u8g2, set.title, set.items, menuIndex - 6, MENU_MAX - 6, encMode);
    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    bool trig = clockEdge.isEdgeHigh();
    clockGate = clockEdge.getValue();
    if (trig)
    {
        clockCount++;
        clockCount = clockCount % resetCount;
    }

    for (int i = 0; i < VOICE_MAX; ++i)
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

    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
    initPWM(OUT3, PWM_RESO);
    initPWM(OUT4, PWM_RESO);
    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void loop()
{
    pot.analogReadDropLow4bit();
    enc.getDirection(true);
    uint16_t voct = vOct.analogReadDirect();
    int16_t cv1Value = cv1.analogReadDirect();
    uint16_t cv2Value = cv2.analogReadDirect();

    // static uint8_t probabilities[4] = {0};
    // static uint8_t states[VOICE_MAX] = {0};
    // if ((clockCount & 0xF) == 0)
    // {
    //     randomSeed(voct);
    //     for (int i = 0; i < 4; ++i)
    //     {
    //         probabilities[i] = random(4);
    //     }
    // }

    for (int i = 0; i < VOICE_MAX; ++i)
    {
        pKit[i]->setSpeed(pitches[i]);
        pKit[i]->updateDecay(decays[i]);
        pKit[i]->setVolume(volumes[i]);
        int state = clockGate;
        int beat = beats[beatsSelects[i]][clockCount & 0xF];
        if (i == 4 && isSDFill)
        {
            beat = 1;
        }
        beat = beat <= 1 ? beat : ((beat >> (clockCount >> 4)) & 1);
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

    // カウンタリセット時点灯
    gpio_put(LED1, (clockCount & 0xF) == 0 ? HIGH : LOW);
    gpio_put(LED2, clockCount == 0 ? HIGH : LOW);

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

    sleep_ms(1);
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
    uint8_t btn2 = buttons[2].getState();

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

    if (btn0 == 1)
    {
        clockCount = resetCount - 1;
    }

    if (menuIndex < 6) {
        if (encMode)
        {
            beatsSelects[(menuIndex % 6)] = constrain(beatsSelects[(menuIndex % 6)] + encValue, 0, BEATS_TOTAL - 1);
        }
    }
    else
    {
        requiresUpdate |= set.items[menuIndex - 6].add(encValue);
    }

    int8_t muteIndex = map(potValue, 0, 4095, 0, VOICE_MAX);
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
