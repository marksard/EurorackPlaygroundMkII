/*!
 * Trigger Happy Prototype
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"

#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/TriggerOut.hpp"
#include "../../commonlib/common/Quantizer.hpp"
#include "../../commonlib/common/Euclidean.hpp"

#include "StepSeqModel.hpp"
#include "BlinkLED.hpp"

#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_RESO 2048         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8)
static uint interruptSliceNum;

// 標準インターフェース
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;
static BlinkLED led1;
static BlinkLED led2;

// common
#define OUT_COUNT 5
static uint8_t outPins[OUT_COUNT] = {OUT1, OUT2, OUT3, OUT4, OUT5};
static uint8_t mainMode = 0;
static EdgeChecker clockEdge;
static uint64_t clockCount = 0;
static uint64_t resetCount = 64;
// static uint64_t resetCount = 144403552893600; // 1-32の最小公倍数

static TriggerOut triggerOuts[OUT_COUNT];
// トリガーパルス幅（%）、100は通常のゲート出力とする
static uint8_t trigDurations[] = {2, 8, 16, 32, 64, 80, 100};
static uint8_t trigDurationsSize = sizeof(trigDurations) / sizeof(trigDurations[0]);
static uint8_t trigDurationMode = trigDurationsSize - 1;

// ClockDivider
volatile bool clockEdgeLatch = false;
volatile bool dataEdgeLatch = false;
volatile bool clockGate = false;
static uint8_t divisionsIndex = 3;
static uint8_t divisions[][OUT_COUNT] = {
    {1, 2, 5, 8, 12},
    {2, 3, 4, 5, 6},
    {2, 4, 8, 16, 32},
    {3, 5, 7, 9, 11},
};
static uint8_t divisionsSize = sizeof(divisions) / sizeof(divisions[0]);

// ShiftRegister
#define SHIFT_REGISTER_SIZE 8
static uint8_t shiftRegisterValues[SHIFT_REGISTER_SIZE] = {0};
static uint8_t shiftRegisterBits[OUT_COUNT] = {1, 2, 3, 4, 5};
static uint8_t r2rScale = 1;
static int8_t r2rScaleIndex[] = {0, 5, 7};
static uint8_t r2rScaleIndexSize = sizeof(r2rScaleIndex) / sizeof(r2rScaleIndex[0]);
static uint8_t r2rOctMax = 2;
static Quantizer quantizer(PWM_RESO);
static bool shiftResisterUseInternalData = true;
static uint8_t srOnsets = 10;
static uint8_t srStepSize = 16;
static Euclidean srInternalData;

// Euclidean
static uint8_t euclidOnsetsIndex = 1;
static uint8_t euclidOnsets[][OUT_COUNT] = 
{
    {2, 4, 6, 7, 9},
    {3, 5, 7, 8, 7},
    {4, 6, 8, 9, 5},
    {5, 7, 9, 6, 3},
    {6, 8, 5, 3, 2},
};
static uint8_t euclidOnsetsSize = sizeof(euclidOnsets) / sizeof(euclidOnsets[0]);
static uint8_t euclidStep = 16;
static Euclidean euclid[OUT_COUNT];

// StepSeq
static StepSeqModel stepSeqModel;
static bool requestGenerateSequence = false;

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

void initLED()
{
    initPWM(LED1, PWM_RESO);
    led1.init(SAMPLE_FREQ);
    led1.setWave(BlinkLED::Wave::SQU);
    led1.setFrequency(100);
    led1.setLevel(0, 0);
    initPWM(LED2, PWM_RESO);
    led2.init(SAMPLE_FREQ);
    led2.setWave(BlinkLED::Wave::SQU);
    led2.setFrequency(100);
    led2.setLevel(0, 0);
}

void setLED(uint8_t led, uint8_t level, uint8_t max, int8_t freq = 10)
{
    if (led == 1)
    {
        led1.setFrequency(freq);
        led1.setLevel(level, max);
    }
    else if (led == 2)
    {
        led2.setFrequency(freq);
        led2.setLevel(level, max);
    }
}

void offLED(uint8_t led)
{
    if (led == 1)
    {
        led1.setLevel(0, 0);
    }
    else if (led == 2)
    {
        led2.setLevel(0, 0);
    }
}

void setLevelIndicationDoubleLED(uint8_t level, uint8_t max, int8_t freq)
{
    uint8_t maxHalf = max >> 1;
    if (level <= maxHalf)
    {
        setLED(1, level, maxHalf, freq);
        setLED(2, 0, freq);
    }
    else
    {
        setLED(1, 1, 1, freq);
        setLED(2, level - maxHalf, maxHalf, freq);
    }
}

uint8_t updateMainMode(int8_t encValue)
{
    mainMode = constrainCyclic(mainMode + encValue, 0, 3);

    setLED(1, (((mainMode + 1) >> 1) & 0x01) ? 1 : 0, 1, 10);
    setLED(2, ((mainMode + 1) & 0x01) ? 1 : 0, 1, 10);

    return mainMode;
}

void generateSequence()
{
    stepSeqModel.generateSequence(-1, 2,
                                  StepSeqModel::Gate::S, StepSeqModel::Gate::G, StepSeqModel::Gate::_);

    stepSeqModel._scaleIndex.set(5);
    stepSeqModel.keyStep.setMode(Step::Mode::Forward);
    stepSeqModel.gateStep.setMode(Step::Mode::Forward);
    stepSeqModel.keyStep.resetPlayStep();
    stepSeqModel.gateStep.resetPlayStep();
    // stepSeqModel.printSeq();
}

void updateSequenceProcedure()
{
    if (stepSeqModel.gateStep.pos.getMin() == stepSeqModel.gateStep.pos.get())
    {
        if (requestGenerateSequence)
        {
            requestGenerateSequence = false;
            generateSequence();
        }
    }

    uint16_t voct = stepSeqModel.getPlayNote() * quantizer.VoltPerTone;
    pwm_set_gpio_level(OUT6, voct);
    stepSeqModel.keyStep.nextPlayStep();
}

void initClockDivider()
{
    initTriggerOuts();
}

void updateClockDividerProcedure()
{
    int duration = clockEdge.getDurationMills();
    duration = map(trigDurations[trigDurationMode], 0, 100, 0, duration);
    for (int i = 0; i < OUT_COUNT; ++i)
    {
        triggerOuts[i].setDuration(duration);
        int16_t div = divisions[divisionsIndex][i];
        // クロックカウンタの値をdivで割った余りが、divの半分より小さい場合にトリガーを出力する
        bool trig = (clockCount % div) < (div >> 1);
        bool out = triggerOuts[i].getTriggerGate(div == 1 ? clockGate : trig, trigDurationMode == trigDurationsSize - 1 ? 0 : 1);
        triggerOuts[i].set(out);
    }

    if (clockEdgeLatch)
    {
        updateSequenceProcedure();
        clockEdgeLatch = false;
    }
}

void updateClockDividerUI(uint8_t btn0, uint8_t btn1, uint8_t btn2, int8_t encValue)
{
    // A
    if (btn0 == 2)
    {
        clockCount = resetCount - 1; // スタートポジションを合わせる
        stepSeqModel.keyStep.setMode(Step::Mode::Reverse);
        stepSeqModel.keyStep.resetPlayStep();
        stepSeqModel.keyStep.setMode(Step::Mode::Forward);
        stepSeqModel.gateStep.setMode(Step::Mode::Reverse);
        stepSeqModel.gateStep.resetPlayStep();
        stepSeqModel.gateStep.setMode(Step::Mode::Forward);
    }
    // B
    else if (btn1 == 2)
    {
        clockCount = (clockCount + 1) % resetCount;
    }
    // M
    else if (btn2 == 2)
    {
        updateMainMode(1);
    }
    // A+B
    else if (btn0 == 3 && btn1 == 3)
    {
        requestGenerateSequence = true;
        setLED(1, 1, 1, 20);
        setLED(2, 1, 1, 20);
    }
    // A+Enc
    else if (btn0 == 3)
    {
    }
    // B+Enc
    else if (btn1 == 3)
    {
        divisionsIndex = constrain(divisionsIndex + encValue, 0, divisionsSize - 1);
        setLevelIndicationDoubleLED(divisionsIndex, divisionsSize - 1, 100);
    }
    // M+Enc
    else if (btn2 == 3)
    {
        trigDurationMode = constrain(trigDurationMode + encValue, 0, trigDurationsSize - 1);
        setLevelIndicationDoubleLED(trigDurationMode, trigDurationsSize - 1, 100);
    }
    // Enc only
    else if (btn0 == 0 && btn1 == 0 && btn2 == 0)
    {
        stepSeqModel.keyStep.pos.setLimit(stepSeqModel.keyStep.pos.getMin(),
                                          stepSeqModel.keyStep.pos.getMax() + encValue);
        // setLED(1, clockCount, resetCount, 100);
        setLED(2, stepSeqModel.keyStep.pos.get() <= (stepSeqModel.keyStep.pos.getMax() >> 1), 1, 100);
    }
}

void initShiftRegister()
{
    initTriggerOuts();

    for (int i = 0; i < SHIFT_REGISTER_SIZE; ++i)
    {
        shiftRegisterValues[i] = 0;
    }

    srInternalData.generate(srOnsets, srStepSize);
}

void updateShiftRegisterProcedure()
{
    int duration = clockEdge.getDurationMills();
    duration = map(trigDurations[trigDurationMode], 0, 100, 0, duration);

    if (clockEdgeLatch)
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            triggerOuts[i].setDuration(duration);
            bool trig = shiftRegisterValues[shiftRegisterBits[i] - 1] == 1;
            bool out = triggerOuts[i].getTriggerGate(trig, trigDurationMode == trigDurationsSize - 1 ? 0 : 1);
            triggerOuts[i].set(out);
        }

        uint16_t r2rOut = 1;
        for (int i = 0; i < 8; ++i)
        {
            r2rOut += (shiftRegisterValues[i] ? 1 : 0) << i;
        }

        // quantizer
        quantizer.setScale(r2rScaleIndex[r2rScale]);
        uint16_t cv = map(r2rOut, 0, 255, 0, (7 * r2rOctMax));
        int16_t r2rCV = r2rScaleIndex[r2rScale] == -1 ? cv : quantizer.Quantize(cv);
        pwm_set_gpio_level(OUT6, r2rCV);

        clockEdgeLatch = false;
        if (dataEdgeLatch)
        {
            dataEdgeLatch = false;
        }
    }
    else
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            triggerOuts[i].update(0);
        }
    }
}

void updateShiftRegisterUI(uint8_t btn0, uint8_t btn1, uint8_t btn2, int8_t encValue)
{
    // A
    if (btn0 == 2)
    {
        srInternalData.resetCurrent();
    }
    // B
    if (btn1 == 2)
    {
        shiftResisterUseInternalData = shiftResisterUseInternalData ? false : true;
        offLED(1);
        offLED(2);
    }
    // M
    else if (btn2 == 2)
    {
        updateMainMode(1);
    }
    // A+Enc
    else if (btn0 == 3)
    {
        if (shiftResisterUseInternalData)
        {
            srOnsets = constrain(srOnsets + encValue, 1, Euclidean::EUCLID_MAX_STEPS);
            srStepSize = constrain(srStepSize, srOnsets, Euclidean::EUCLID_MAX_STEPS);
            srInternalData.generate(srOnsets, srStepSize);
            setLevelIndicationDoubleLED(srOnsets, Euclidean::EUCLID_MAX_STEPS, 100);
        }
    }
    // B+Enc
    else if (btn1 == 3)
    {
        if (shiftResisterUseInternalData)
        {
            srStepSize = constrain(srStepSize + encValue, srOnsets, Euclidean::EUCLID_MAX_STEPS);
            srInternalData.generate(srOnsets, srStepSize);
            setLevelIndicationDoubleLED(srStepSize, Euclidean::EUCLID_MAX_STEPS, 100);
        }
    }
    // M+Enc
    else if (btn2 == 3)
    {
        trigDurationMode = constrain(trigDurationMode + encValue, 0, trigDurationsSize - 1);
        setLevelIndicationDoubleLED(trigDurationMode, trigDurationsSize - 1, 100);
    }
    // Enc Only
    else if (btn0 == 0 && btn1 == 0 && btn2 == 0)
    {
        r2rOctMax = constrain(r2rOctMax + encValue, 1, 5);
        if (shiftResisterUseInternalData)
        {
            setLED(1, srOnsets, Euclidean::EUCLID_MAX_STEPS, 100);
            setLED(2, srInternalData.getCurrent(), Euclidean::EUCLID_MAX_STEPS, 100);
        }
    }
}

void initEuclidean()
{
    initTriggerOuts();
    for (int i = 0; i < OUT_COUNT; ++i)
    {
        euclid[i].generate(euclidOnsets[euclidOnsetsIndex][i], euclidStep);
    }
}

void updateEuclideanProcedure()
{
    int duration = clockEdge.getDurationMills();
    duration = map(trigDurations[trigDurationMode], 0, 100, 0, duration);

    if (clockEdgeLatch)
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            int8_t trig = euclid[i].getNext();
            triggerOuts[i].setDuration(duration);
            bool out = triggerOuts[i].getTriggerGate(trig, trigDurationMode == trigDurationsSize - 1 ? 0 : 1);
            triggerOuts[i].set(out);
        }

        updateSequenceProcedure();

        clockEdgeLatch = false;
    }
    else
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            triggerOuts[i].update(0);
        }
    }
}

void generateEuclideanSequence()
{
    for (int i = 0; i < OUT_COUNT; ++i)
    {
        uint8_t step = euclidStep;
        uint8_t onset = euclidOnsets[euclidOnsetsIndex][i];
        // Serial.print("euclid[");
        // Serial.print(i);
        // Serial.print("] = ");
        // Serial.print(onset);
        // Serial.print(",");
        // Serial.print(step);
        // Serial.println();
        onset = (onset >= step - 1) ? constrain(step - (Euclidean::EUCLID_MAX_STEPS - onset), 1, step) : onset;
        // Serial.print("changed = ");
        // Serial.print(onset);
        // Serial.println();
        euclid[i].generate(onset, step);
    }
}

void updateEuclideanUI(uint8_t btn0, uint8_t btn1, uint8_t btn2, int8_t encValue)
{
    // A
    if (btn0 == 2)
    {
        for (int i = 0; i < OUT_COUNT; ++i)
        {
            euclid[i].resetCurrent();
        }    
    }
    // B
    if (btn1 == 2)
    {
    }
    // M
    else if (btn2 == 2)
    {
        updateMainMode(1);
    }
    // A+B
    else if (btn0 == 3 && btn1 == 3)
    {
        requestGenerateSequence = true;
        setLED(1, 1, 1, 20);
        setLED(2, 1, 1, 20);
    }
    // A+Enc
    else if (btn0 == 3)
    {
        euclidOnsetsIndex = constrain(euclidOnsetsIndex + encValue, 0, euclidOnsetsSize - 1);
        generateEuclideanSequence();
        setLevelIndicationDoubleLED(euclidOnsetsIndex, euclidOnsetsSize - 1, 100);
    }
    // B+Enc
    else if (btn1 == 3)
    {
        euclidStep = constrain(euclidStep + encValue, 4, Euclidean::EUCLID_MAX_STEPS);
        generateEuclideanSequence();
        setLevelIndicationDoubleLED(euclid[0].getStepSize(), Euclidean::EUCLID_MAX_STEPS, 100);
    }
    // M+Enc
    else if (btn2 == 3)
    {
        trigDurationMode = constrain(trigDurationMode + encValue, 0, trigDurationsSize - 1);
        setLevelIndicationDoubleLED(trigDurationMode, trigDurationsSize - 1, 100);
    }
    // Enc Only
    else if (btn0 == 0 && btn1 == 0 && btn2 == 0)
    {
        stepSeqModel.keyStep.pos.setLimit(stepSeqModel.keyStep.pos.getMin(),
                                          stepSeqModel.keyStep.pos.getMax() + encValue);
        setLED(1, euclid[0].getCurrent() < (euclid[0].getStepSize() >> 1), 1, 100);
        setLED(2, stepSeqModel.keyStep.pos.get() <= (stepSeqModel.keyStep.pos.getMax() >> 1), 1, 100);
    }
}

void initDefault()
{
    initTriggerOuts();
}

static uint16_t testToneVOct = 0;
void defaultProcedure()
{
    pwm_set_gpio_level(OUT6, testToneVOct * quantizer.VoltPerTone);
}

void defaultUI(uint8_t btn0, uint8_t btn1, uint8_t btn2, int8_t encValue)
{
    if (btn2 == 2)
    {
        updateMainMode(1);
    }
    // Enc only
    else if (btn0 == 0 && btn1 == 0 && btn2 == 0)
    {
        testToneVOct = constrain(testToneVOct + encValue, 0, 60);
        setLED(1, (testToneVOct % 12) == 0 ? 1 : 0, 1, 100);        
    }
}

void edgeCallback(uint gpio, uint32_t events)
{
    if (gpio == GATE)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            clockEdge.updateEdge(1);
            clockCount = (clockCount + 1) % resetCount;
            clockEdgeLatch = true;
            clockGate = true;

            for (int i = 7; i > 0; --i)
            {
                shiftRegisterValues[i] = shiftRegisterValues[i - 1];
            }

            if (shiftResisterUseInternalData)
            {
                shiftRegisterValues[0] = srInternalData.getNext() ? 1 : 0;
            }
            else
            {
                shiftRegisterValues[0] = dataEdgeLatch ? 1 : 0;
            }
        }
        else
        {
            clockEdge.updateEdge(0);
            clockGate = false;
        }
    }
    else if (gpio == VOCT)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            clockCount = 0;
            srInternalData.resetCurrent();
            for (int i = 0; i < OUT_COUNT; ++i)
            {
                euclid[i].resetCurrent();
            }    
        }
    }
    else if (gpio == CV1)
    {
        if (events & GPIO_IRQ_EDGE_RISE)
        {
            dataEdgeLatch = true;
        }
    }
}

// void gpioIRQHandler()
// {
//     for (int gpio = 29; gpio >= 0; gpio--)
//     {
//         uint32_t events = gpio_get_irq_event_mask(gpio);
//         if (events)
//         {
//             gpio_acknowledge_irq(gpio, events);
//             edgeCallback(gpio, events);
//         }
//     }
// }

// void interruptPWM()
// {
//     pwm_clear_irq(interruptSliceNum);
// }

void setup()
{
    // Serial.begin(9600);
    analogReadResolution(12);

    pot.init(POT1);
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    // vOct.init(VOCT);
    // cv1.init(CV1);
    cv2.init(CV2);

    initPWM(OUT6, PWM_RESO);

    initLED();
    initTriggerOuts();
    initClockDivider();
    initShiftRegister();
    initEuclidean();
    initDefault();
    stepSeqModel.resetSequence(StepSeqModel::Gate::S);
    generateSequence();

    clockEdge.init(GATE);
    pinMode(VOCT, INPUT);
    pinMode(CV1, INPUT);

    // initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);
    // irq_set_exclusive_handler(IO_IRQ_BANK0, gpioIRQHandler);

    gpio_set_irq_enabled(GATE, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(VOCT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CV1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_callback(edgeCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop()
{
    int8_t encValue = enc.getDirection();
    // uint16_t voct = vOct.analogReadDirect();
    // int16_t cv1Value = cv1.analogReadDirect();
    // uint16_t cv2Value = cv2.analogReadDirect();

    switch (mainMode)
    {
    case 0:
        updateClockDividerProcedure();
        break;
    case 1:
        updateShiftRegisterProcedure();
        break;
    case 2:
        updateEuclideanProcedure();
        break;
    default:
        defaultProcedure();
        break;
    }

    pwm_set_gpio_level(LED1, led1.getWaveValue());
    pwm_set_gpio_level(LED2, led2.getWaveValue());

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

    switch (mainMode)
    {
    case 0:
        updateClockDividerUI(btn0, btn1, btn2, encValue);
        break;
    case 1:
        updateShiftRegisterUI(btn0, btn1, btn2, encValue);
        break;
    case 2:
        updateEuclideanUI(btn0, btn1, btn2, encValue);
        break;
    default:
        defaultUI(btn0, btn1, btn2, encValue);
        break;
    }

    if (btn0 == 2 || btn0 == 4 ||
        btn1 == 2 || btn1 == 4 ||
        btn2 == 2 || btn2 == 4)
    {
        offLED(1);
        offLED(2);
    }

    sleep_us(1000);
}
