/*!
 * Step Sequencer
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
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/Euclidean.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"
#include "StepSeqPlayControl.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 4096         // 12bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 20)
static uint interruptSliceNum;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// step seq
static StepSeqPlayControl sspc(&u8g2);
static int16_t extSync;
static int16_t octUnder;
static int16_t octUpper;
static int16_t gateMin;
static int16_t gateMax;
static int16_t gateInitial;
static int16_t bpm;
static int16_t scale;
static int16_t ppq;

// quantizer
const int16_t halfReso = (ADC_RESO >> 1);
// static float voltPerTone = 4095.0 / 12.0 / 5.0;
static int16_t quantizeOut = 0;
static Euclidean euclid;
static TriggerOut euclidTrig;
static int16_t euclidOnsets;
static int16_t euclidStepSize;
static int16_t holdTrigger;

// 画面周り
#define MENU_MAX (8+12)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
PollingTimeEvent updateOLED;

typedef struct
{
    char title[12];
    SettingItem16 items[12];
} SettingMenu;

static const char scaleNames[][5] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"};
static const char onoff[][5] = {"OFF", "ON"};
static const char holdTriggers[][5] = {"CLK", "EUC"};
SettingMenu set[] = {
    {"SETTINGS", {
        SettingItem16(1, 16, 1, &euclidOnsets, "EUCRID ONSETS: %d", NULL, 0),
        SettingItem16(1, 16, 1, &euclidStepSize, "EUCRID STEP: %d", NULL, 0),
        SettingItem16(0, 1, 1, &holdTrigger, "HOLD TIGGER: %s", holdTriggers, 2),
        SettingItem16(-1, 4, 1, &octUnder, "GEN OCT UNDER: %d", NULL, 0),
        SettingItem16(-1, 4, 1, &octUpper, "GEN OCT UPPER: %d", NULL, 0),
        SettingItem16(0, StepSeqModel::Gate::L, 1, &gateMin, "GEN GATE MIN: %d", NULL, 0),
        SettingItem16(1, StepSeqModel::Gate::Max, 1, &gateMax, "GEN GATE MAX: %d", NULL, 0),
        SettingItem16(0, StepSeqModel::Gate::H, 1, &gateInitial, "GEN GATE INI: %d", NULL, 0),
        SettingItem16(0, 256, 1, &bpm, "BPM: %d", NULL, 0),
        SettingItem16(0, 10, 1, &scale, "SCALE: %s", scaleNames, 10),
        SettingItem16(0, 4, 1, &ppq, "PPQ: %d", NULL, 0),
        SettingItem16(0, 1, 1, &extSync, "SYNC IN: %d", onoff, 0),
    }}};

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

    // if (!requiresUpdate)
    //     return;
    // requiresUpdate = 0;

    sspc.updateLED();

    if (requiresUpdate)
        u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_7x14B_tf);
    switch (menuIndex)
    {
    case 0:
        u8g2.drawStr(0, 0, "GENERATE :RST<>GEN");
        break;
    case 1:
        sprintf(disp_buf,  "GATE SHFT: %1d", sspc.getGateLen());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 2:
        sprintf(disp_buf,  "OCT  SHFT: %1d", sspc.getOctave());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 3:
        u8g2.drawStr(0, 0, "ROTATE   : L <> R");
        break;
    case 4:
        u8g2.drawStr(0, 0, "STEP LEN :1 to 16");
        break;
    case 5:
        u8g2.drawStr(0, 0, "EDIT NOTE:00to 50");
        break;
    case 6:
        u8g2.drawStr(0, 0, "EDIT GATE: - to G");
        break;
    case 7:
        u8g2.drawStr(0, 0, "EDIT ACC : _ or *");
        break;
    default:
        drawSetting(&u8g2, set[0].title, set[0].items, menuIndex - 8, MENU_MAX - 8, encMode);
        u8g2.sendBuffer();
        return;
    }

    if (encMode == 0)
        u8g2.drawBox(0, 13, 63, 2);
    else
        u8g2.drawBox(68, 13, 60, 2);

    u8g2.setFont(u8g2_font_5x8_tf);
    sspc.updateDisplay();

    if (requiresUpdate)
        u8g2.sendBuffer();
    else
        u8g2.updateDisplay();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);

    int8_t ready = sspc.updateProcedure();

    // quantizer
    if (ready)
    {
        int8_t trig = euclid.getNext();
        euclidTrig.update(trig);
        if ((holdTrigger == 0 && ready) ||
            (holdTrigger == 1 && trig))
        {
            pwm_set_gpio_level(OUT6, quantizeOut);
        }
    }
    else
    {
        euclidTrig.update(LOW);
    }

    requiresUpdate |= ready;

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
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);
    pinMode(GATE, INPUT);

    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    sspc.setClockMode(StepSeqPlayControl::CLOCK::INT);
    sspc.requestResetAllSequence();
    sspc.setBPM(133, 48);
    sspc.start();
    bpm = sspc.getBPM();
    scale = sspc.getScale();
    ppq = sspc.getPPQ();
    octUnder = sspc.getOctUnder();
    octUpper = sspc.getOctUpper();
    gateMin = sspc.getGateMin();
    gateMax = sspc.getGateMax();
    gateInitial = sspc.getGateInitial();

    euclidOnsets = 5;
    euclidStepSize = 16;
    holdTrigger = 0;

    euclidTrig.init(OUT4);
    euclid.generate(euclidOnsets, euclidStepSize);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    pot.analogReadDropLow4bit();
    enc.getDirection(true);
    uint16_t voct = vOct.analogReadDirect();
    int16_t cv1Value = cv1.analogReadDirect();
    uint16_t cv2Value = cv2.analogReadDirect();

    // quantizer
    int8_t cv = map(cv1Value, 0, 4096, 0, 36);
    int8_t oct = cv / 7;
    int8_t semi = sspc.getScaleKey(sspc.getScale(), cv % 7);
    quantizeOut = ((oct * 12) + semi) * voltPerTone;

    int length = sspc.getStepDulation();
    euclidTrig.setDuration(length >> 1);

    // Serial.print(cv1Value);
    // Serial.print(", ");
    // Serial.print(cv2Value);
    // Serial.println();

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
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    // requiresUpdate |= updateMenuIndex(btn0, btn1);
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

    uint8_t step = map(potValue, 0, 4095, 0, 15);
    sspc.setSettingPos(step);

    if (btn0 == 1)
    {
        if (sspc.isStart())
        {
            sspc.stop();
            sspc.reset();
        }
        else
        {
            sspc.start();
        }
    }

    switch (menuIndex)
    {
    case 0:
        if (encValue == 1)
        {
            sspc.requestGenerateSequence();
        }
        else if (encValue == -1)
        {
            sspc.resetAllSequence();
        }
        break;
    case 1:
        sspc.addGateLen(encValue);
        break;
    case 2:
        sspc.addOctave(encValue);
        break;
    case 3:
        sspc.moveSeq(encValue);
        break;
    case 4:
        sspc.addGateKeyEnd(encValue, encValue);
        break;
    case 5:
        sspc.addNote(encValue);
        break;
    case 6:
        sspc.addGate(encValue);
        break;
    case 7:
        if (encValue == 1)
        {
            sspc.toggleAcc();
        }
        break;
    default:
        requiresUpdate |= set[0].items[menuIndex - 8].add(encValue);
        sspc.setClockMode(extSync ? StepSeqPlayControl::CLOCK::EXT : StepSeqPlayControl::CLOCK::INT);
        sspc.setOctUnder(octUnder);
        sspc.setOctUpper(octUpper);
        sspc.setGateMin(gateMin);
        sspc.setGateMax(gateMax);
        sspc.setGateInitial(gateInitial);
        sspc.setBPM(bpm, 48);
        sspc.setScale(scale);
        sspc.setPPQ(ppq);

        euclidOnsets = constrain(euclidOnsets, 0, euclidStepSize);
        euclidStepSize = constrain(euclidStepSize, euclidOnsets, euclid.EUCLID_MAX_STEPS);
        euclid.generate(euclidOnsets, euclidStepSize);
        break;
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
