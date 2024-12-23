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
#include "../../commonlib/ui_common/EuclideanDisp.hpp"
#include "../../commonlib/common/Euclidean.hpp"
#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"
#include "StepSeqPlayControl.hpp"
#include "Oscillator.hpp"
#include "EepromData.h"

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
// static SmoothAnalogRead vOct;
static EdgeChecker vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// ユーザー設定
static UserConfig userConfig;
static bool saveConfirm = false;

static StepSeqPlayControl sspc(&u8g2);
static Euclidean euclid;
static EuclideanDisp euclidDisp;
static TriggerOut euclidTrig;
static Oscillator intLFO;
static int16_t quantizeOut = 0;

// 画面周り
#define MENU_MAX (8+1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
static PollingTimeEvent updateOLED;

static const char scaleNames[][5] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"};
static const char seqSyncModes[][5] = {"INT", "GATE"};
static const char shTriggers[][5] = {"CLK", "EUC"};
static const char shSources[][5] = {"INT", "CV1"};

static const char euclidSyncDivsStr[][5] = {"1", "2", "3", "4", "8", "16"};
static const char euclidSyncDivs[] = {1, 2, 3, 4, 8, 16};

SettingItem16 commonSettings[] =
{
    SettingItem16(0, 1, 1, &userConfig.seqSyncMode, "SYNC MODE: %s", seqSyncModes, 2),
    SettingItem16(0, 256, 1, &userConfig.bpm, "BPM: %d", NULL, 0),
    SettingItem16(0, 10, 1, &userConfig.scale, "SCALE: %s", scaleNames, 10),
    SettingItem16(0, 3, 1, &userConfig.swing, "SWING: %d", NULL, 0),
};

SettingItem16 sequenceSettings[] =
{
    SettingItem16(-1, 4, 1, &userConfig.octUnder, "OCT UNDER: %d", NULL, 0),
    SettingItem16(-1, 4, 1, &userConfig.octUpper, "OCT UPPER: %d", NULL, 0),
    SettingItem16(0, StepSeqModel::Gate::Max, 1, &userConfig.gateMin, "GATE MIN: %s", StepSeqModel::GateDisp, StepSeqModel::Gate::Max),
    SettingItem16(1, StepSeqModel::Gate::Max, 1, &userConfig.gateMax, "GATE MAX: %s", StepSeqModel::GateDisp, StepSeqModel::Gate::Max),
    SettingItem16(0, StepSeqModel::Gate::Max, 1, &userConfig.gateInitial, "GATE INI: %s", StepSeqModel::GateDisp, StepSeqModel::Gate::Max),
};

SettingItem16 euclidSettings[] =
{
    SettingItem16(0, 5, 1, &userConfig.euclidSyncDiv, "DIV:/%s", euclidSyncDivsStr, 6),
    SettingItem16(-16, 16, 1, &userConfig.euclidPos, "POS:%d", NULL, 0),
    SettingItem16(0, 16, 1, &userConfig.euclidOnsets, " ON:%2d", NULL, 0),
    SettingItem16(1, 16, 1, &userConfig.euclidStepSize, "STP:%2d", NULL, 0),
};

SettingItem16 shettings[] =
{
    SettingItem16(0, 1, 1, &userConfig.shTrigger, "TRG:%s", shTriggers, 2),
    SettingItem16(0, 1, 1, &userConfig.shSource, "SRC:%s", shSources, 2),
    SettingItem16(1, 5, 1, &userConfig.shIntOctMax, "OCT:%2d", NULL, 0),
    SettingItem16(1, 99, 1, &userConfig.shIntSpeed, "SPD:%2d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"COMMON", commonSettings, sizeof(commonSettings) / sizeof(commonSettings[0])},
    {"PATTERNSEQ", sequenceSettings, sizeof(sequenceSettings) / sizeof(sequenceSettings[0])},
};

static MenuSection16 menu2[] = {
    {"EUCLIDTRIG", euclidSettings, sizeof(euclidSettings) / sizeof(euclidSettings[0])},
    {"SAMPL&HOLD", shettings, sizeof(shettings) / sizeof(shettings[0])}
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));
static MenuControl16 euclidMenuControl(menu2, sizeof(menu2) / sizeof(menu2[0]));

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

    sspc.updateLED();

    if (requiresUpdate)
        u8g2.clearBuffer();

    u8g2.setDrawColor(2);
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
    case 8:
        u8g2.setDrawColor(1);
        euclidDisp.drawCircle(&u8g2, euclid.getStepSize(), euclid.getStartPos(), euclid.getCurrent(), euclid.getSteps());
        euclidMenuControl.draw(&u8g2, encMode, false, true);
        u8g2.sendBuffer();
        return;
    default:
        menuControl.draw(&u8g2, encMode);
        u8g2.sendBuffer();
        return;
    }

    if (encMode == 0)
        u8g2.drawBox(0, 13, 63, 2);
    else
        u8g2.drawBox(68, 13, 60, 2);

    u8g2.setFont(u8g2_font_5x8_tf);
    sspc.updateDisplay();

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

    // euclid
    if (ready)
    {
        if (sspc.getGatePos() % euclidSyncDivs[userConfig.euclidSyncDiv] == 0)
        {
            int8_t trig = euclid.getNext();
            euclidTrig.update(trig);
            if ((userConfig.shTrigger == 0 && ready) ||
                (userConfig.shTrigger == 1 && trig))
            {
                pwm_set_gpio_level(OUT6, quantizeOut);
            }
        }
        else
        {
            euclidTrig.update(LOW);
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
    // pinMode(GATE, INPUT);

    initPWM(OUT5, PWM_RESO);
    initPWM(OUT6, PWM_RESO);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    initEEPROM();
    loadUserConfig(&userConfig);

    sspc.setClockMode((StepSeqPlayControl::CLOCK)userConfig.seqSyncMode);
    sspc.setOctUnder(userConfig.octUnder);
    sspc.setOctUpper(userConfig.octUpper);
    sspc.setGateMin(userConfig.gateMin);
    sspc.setGateMax(userConfig.gateMax);
    sspc.setGateInitial(userConfig.gateInitial);
    sspc.setSwingIndex(userConfig.swing);
    sspc.setBPM(userConfig.bpm, 48);
    sspc.setScale(userConfig.scale);
    sspc.requestResetAllSequence();
    sspc.start();

    euclidTrig.init(OUT4);
    euclid.generate(userConfig.euclidOnsets, userConfig.euclidStepSize);
    euclidDisp.init(28, 42, 21);
    euclidDisp.generateCircle(userConfig.euclidStepSize);

    intLFO.init(SAMPLE_FREQ);
    intLFO.setWave(Oscillator::Wave::TRI);

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
}

void loop()
{
    pot.analogReadDropLow4bit();
    enc.getDirection();
    // uint16_t voct = vOct.analogReadDirect();
    int16_t cv1Value = cv1.analogReadDirect();
    uint16_t cv2Value = cv2.analogReadDirect();

    intLFO.setFrequency(userConfig.shIntSpeed);
    uint16_t internalLFOValue = intLFO.getWaveValue();

    // quantizer
    int16_t cv = 0;
    if (userConfig.shSource)
    {
        cv = map(cv1Value, 0, 4096, 0, 36);
    }
    else {
        cv = map(internalLFOValue, 0, 4096, 0, (7 * userConfig.shIntOctMax) + 1);
    }
    int8_t oct = cv / 7;
    int8_t semi = sspc.getScaleKey(sspc.getScale(), cv % 7);
    quantizeOut = ((oct * 12) + semi) * sspc.VoltPerTone;

    int length = sspc.getStepDulation();
    euclidTrig.setDuration(length >> 1);

    // Serial.print(cv1Value);
    // Serial.print(", ");
    // Serial.print(cv2Value);
    // Serial.println();

    sleep_us(1000);
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
    bool voct = vOct.isEdgeHigh();

    if (voct)
    {
        sspc.requestGenerateSequence();
    }

    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        if (menuIndex == 8)
        {
            requiresUpdate |= euclidMenuControl.select(encValue);
            if (euclidMenuControl.isUnder()) 
            {
                menuIndex--;
                requiresUpdate = true;
            }
            else if (euclidMenuControl.isOver())
            {
                menuIndex++;
                requiresUpdate = true;
            }
        }
        else if (menuIndex == 9)
        {
            requiresUpdate |= menuControl.select(encValue);
            if (menuControl.isUnder()) 
            {
                menuIndex--;
                requiresUpdate = true;
            }
            // if (menuControl.isOver())
            // {
            //     menuIndex++;
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

    uint8_t step = map(potValue, 0, 4095, 0, 15);
    sspc.setSettingPos(step);

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
    else if (btn0 == 1)
    {
        if (sspc.isStart())
        {
            sspc.stop();
            sspc.reset();
            euclid.resetCurrent();
        }
        else
        {
            sspc.start();
        }
    }

    if (menuIndex < 8 && btn1 == 1)
    {
        sspc.reset();
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
    case 8:
        if (btn1 == 1)
        {
            euclid.resetCurrent();
        }

        requiresUpdate |= euclidMenuControl.addValue2CurrentSetting(encValue);
        userConfig.euclidOnsets = constrain(userConfig.euclidOnsets, 0, euclid.EUCLID_MAX_STEPS);
        userConfig.euclidStepSize = constrain(userConfig.euclidStepSize, userConfig.euclidOnsets, euclid.EUCLID_MAX_STEPS);
        euclid.setStartPos(userConfig.euclidPos);
        if (euclid.generate(userConfig.euclidOnsets, userConfig.euclidStepSize))
        {
            euclidDisp.generateCircle(euclid.getStepSize());
        }

        userConfig.euclidPos = euclid.getStartPos();
        break;
    default:
        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        sspc.setClockMode((StepSeqPlayControl::CLOCK)userConfig.seqSyncMode);
        sspc.setOctUnder(userConfig.octUnder);
        sspc.setOctUpper(userConfig.octUpper);
        sspc.setGateMin(userConfig.gateMin);
        sspc.setGateMax(userConfig.gateMax);
        sspc.setGateInitial(userConfig.gateInitial);
        sspc.setSwingIndex(userConfig.swing);
        sspc.setBPM(userConfig.bpm, 48);
        sspc.setScale(userConfig.scale);
        break;
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
