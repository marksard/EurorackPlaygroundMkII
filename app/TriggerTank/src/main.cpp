/*!
 * Clock Divider
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"

#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/TriggerOut.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 2048         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 10)
static uint interruptSliceNum;

// 標準インターフェース
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;

// Trigger Tank
#define OUT_COUNT 5
static int outPins[OUT_COUNT] = {OUT1, OUT2, OUT3, OUT4, OUT5};
static uint8_t mainMode = 0;
static EdgeChecker clockEdge;
static EdgeChecker resetEdge;

static int16_t trigDurationMode = 0;
static TriggerOut triggerOuts[OUT_COUNT];
static uint8_t trigDurations[] = {0, 2, 8, 16, 32, 64, 80}; // 0は通常のゲート出力、その他はトリガーパルス幅（%）
static uint8_t trigDurationsSize = sizeof(trigDurations) / sizeof(trigDurations[0]);

static uint8_t clockGate = 0;
static int16_t clockCount = 0;
static int16_t resetCount = 64;
static int16_t divMode = 0;
static int16_t divCount[][OUT_COUNT] = {
    {2, 4, 8, 16, 32},
    {3, 5, 7, 8, 12}
};
static int16_t divCountSize = sizeof(divCount) / sizeof(divCount[0]);

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

void initTriggerOuts()
{

    for (int i = 0; i < OUT_COUNT; ++i)
    {
        triggerOuts[i].init(outPins[i]);
        triggerOuts[i].setDuration(trigDurations[trigDurationMode]);
    }
}

uint8_t updateMainMode(int8_t encValue)
{
    if (encValue > 0)
    {
        mainMode = (mainMode + 1) % 4;
    }
    else if (encValue < 0)
    {
        mainMode = (mainMode + 3) % 4;
    }

    digitalWrite(LED2, mainMode & 0x01);
    digitalWrite(LED1, (mainMode >> 1) & 0x01);

    return mainMode;
}

void initClockDivider()
{
    initTriggerOuts();
}

void updateClockDividerProcedure()
{
    if (resetEdge.isEdgeHigh())
    {
        clockCount = 0;
    }

    int duration = clockEdge.getDurationMills();
    duration = map(trigDurations[trigDurationMode], 0, 100, 0, duration);
    for (int i = 0; i < OUT_COUNT; ++i)
    {
        triggerOuts[i].setDuration(duration);
        int16_t divCountAdd = divCount[divMode][i];
        // クロックカウンタの値をdivCountAddで割った余りが、divCountAddの半分より小さい場合にトリガーを出力する
        bool div = (clockCount % divCountAdd) < (divCountAdd >> 1);
        bool out = triggerOuts[i].getTriggerGate(div, trigDurationMode == 0 ? 0 : 1);
        triggerOuts[i].set(out);
    }
}

void updateClockDividerUI(uint8_t btn0, uint8_t btn1, uint8_t btn2, int8_t encValue)
{
    if (btn0 == 2)
    {
        clockCount = resetCount - 1; // スタートポジションを合わせる
    }
    else if (btn1 == 2)
    {
        divMode = constrainCyclic(divMode + 1, 0, divCountSize - 1);
    }
    else if (btn0 == 3)
    {
        trigDurationMode = constrain(trigDurationMode + encValue, 0, trigDurationsSize - 1);
    }
    else if (btn1 == 3)
    {
        clockCount = (clockCount + encValue) % resetCount;
    }
    else if (btn2 == 3)
    {
        updateMainMode(encValue);
    }
}

// void interruptPWM()
// {
//     pwm_clear_irq(interruptSliceNum);
// }

void clockEdgeCallback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        clockEdge.updateEdge(1);
        clockCount = (clockCount + 1) % resetCount;
    }
    else if (events & GPIO_IRQ_LEVEL_HIGH)
    {
        clockGate = 1;
        clockEdge.updateEdge(0);
    }
    else
    {
        clockGate = 0;
        clockEdge.updateEdge(0);
    }
}

void setup()
{
    analogReadResolution(12);

    pot.init(POT1);
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    // vOct.init(VOCT);
    cv1.init(CV1);
    cv2.init(CV2);

    // initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    clockEdge.init(GATE, clockEdgeCallback);
    resetEdge.init(VOCT);

    initTriggerOuts();
}

void loop()
{
    int8_t encValue = enc.getDirection();
    // uint16_t voct = vOct.analogReadDirect();
    int16_t cv1Value = cv1.analogReadDirect();
    uint16_t cv2Value = cv2.analogReadDirect();

    
    updateClockDividerProcedure();
    
    sleep_us(50);
}

void setup1()
{
}

void loop1()
{
    int8_t encValue = enc.getValue();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    updateClockDividerUI(btn0, btn1, btn2, encValue);

    sleep_us(1000);
}
