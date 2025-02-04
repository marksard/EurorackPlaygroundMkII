#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <hardware/irq.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/common/TriggerOut.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/pwm_wrapper.h"

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
static SmoothAnalogRead cv1;
static SmoothAnalogRead cv2;
static TriggerOut clockOut;

#define DATA_IN GATE
#define CLOCK_IN VOCT
uint8_t shiftRegister[8] = {0};
int16_t shiftRegisterIndex[5] = {1, 2, 3, 4, 5};
int16_t r2rScale = 5;
int16_t r2rOctMax = 2;
volatile bool dataInEdge = false;
volatile bool clockInEdge = false;

#define MAX_SCALE_KEY 7
#define MAX_SCALES 10
#define MAX_SCALES_M1 (MAX_SCALES - 1)

// スケール
static const uint8_t scales[MAX_SCALES][MAX_SCALE_KEY] =
    {
        {0, 2, 4, 5, 7, 9, 11}, // ionian / major
        {0, 2, 3, 5, 7, 9, 10}, // dorian
        {0, 1, 3, 5, 7, 8, 10}, // phrygian
        {0, 2, 4, 6, 7, 9, 11}, // lydian
        {0, 2, 4, 5, 7, 9, 10}, // mixolydian
        {0, 2, 3, 5, 7, 8, 10}, // aeolian / natural minor
        {0, 1, 3, 5, 6, 8, 10}, // locrian
        {0, 2, 3, 4, 7, 9,  0}, // m.blues
        {0, 1, 4, 5, 7, 8, 10}, // spanish
        {0, 2, 4, 7, 9, 0,  2}, // luoyin
};
const float VoltPerTone = 4095.0 / 12.0 / 5.0;
static const char scaleNames[][5] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"};

// 画面周り
#define MENU_MAX (SEQUENCER_TOTAL + 1)
static int16_t menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;

typedef struct
{
    char title[16];
    SettingItem16 items[24];
} SettingMenu;

SettingItem16 shiftResisterSettings[] =
{
    SettingItem16(1, 8, 1, &shiftRegisterIndex[0], "OUT1: %d", NULL, 0),
    SettingItem16(1, 8, 1, &shiftRegisterIndex[1], "OUT2: %d", NULL, 0),
    SettingItem16(1, 8, 1, &shiftRegisterIndex[2], "OUT3: %d", NULL, 0),
    SettingItem16(1, 8, 1, &shiftRegisterIndex[3], "OUT4: %d", NULL, 0),
    SettingItem16(1, 8, 1, &shiftRegisterIndex[4], "OUT5: %d", NULL, 0),
};

SettingItem16 r2rSettings[] =
{
    SettingItem16(0, 10, 1, &r2rScale, "SCALE: %s", scaleNames, 10),
    SettingItem16(1, 5, 1, &r2rOctMax, "OCT:%2d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"SHFT-REGIST", shiftResisterSettings, sizeof(shiftResisterSettings) / sizeof(shiftResisterSettings[0])},
    {"R2R OUT", r2rSettings, sizeof(r2rSettings) / sizeof(r2rSettings[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));

void dataInIntr() {
    dataInEdge = true;
}

void clockInIntr() {
    clockInEdge = true;
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
    if (!requiresUpdate)
    {
        return;
    }

    requiresUpdate = 0;
    u8g2.clearBuffer();
    menuControl.draw(&u8g2, encMode);
    u8g2.sendBuffer();
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
    buttons[0].init(BTN1);
    buttons[0].setHoldTime(5);
    buttons[1].init(BTN2);
    buttons[1].setHoldTime(5);
    buttons[2].init(BTN3);
    cv1.init(CV1);
    cv2.init(CV2);

    pinMode(DATA_IN, INPUT);
    pinMode(CLOCK_IN, INPUT);
    attachInterrupt(digitalPinToInterrupt(DATA_IN), dataInIntr, RISING);
    attachInterrupt(digitalPinToInterrupt(CLOCK_IN), clockInIntr, RISING);
    pinMode(LED2, OUTPUT);
    pinMode(OUT1, OUTPUT);
    pinMode(OUT2, OUTPUT);
    pinMode(OUT3, OUTPUT);
    pinMode(OUT4, OUTPUT);
    pinMode(OUT5, OUTPUT);

    initPWM(OUT6, PWM_RESO);
    clockOut.init(LED1);
    clockOut.setDuration(10);
}

void loop()
{
    uint8_t btn0 = buttons[0].getValue();
    uint8_t btn1 = buttons[1].getValue();
    clockOut.update(clockInEdge);
    if (clockInEdge)
    {
        clockOut.set(clockInEdge);
        bool data = dataInEdge | ((btn1 == 1 || btn1 == 3) ? true : false);
        gpio_put(LED2, data);

        for (int i = 7; i > 0; --i)
        {
            shiftRegister[i] = shiftRegister[i - 1];
        }
        shiftRegister[0] = (btn0 == 1 || btn0 == 3) ? false : data;

        gpio_put(OUT1, shiftRegister[shiftRegisterIndex[0] - 1]);
        gpio_put(OUT2, shiftRegister[shiftRegisterIndex[1] - 1]);
        gpio_put(OUT3, shiftRegister[shiftRegisterIndex[2] - 1]);
        gpio_put(OUT4, shiftRegister[shiftRegisterIndex[3] - 1]);
        gpio_put(OUT5, shiftRegister[shiftRegisterIndex[4] - 1]);
        gpio_put(OUT6, shiftRegister[shiftRegisterIndex[5] - 1]);

        if (dataInEdge) dataInEdge = false;
        if (clockInEdge) clockInEdge = false;

        uint16_t r2rOut = 1;
        for (int i = 0; i < 8; ++i)
        {
            r2rOut += (shiftRegister[i] ? 1 : 0) << i;
        }

        // quantizer
        uint16_t cv = 0;
        cv = map(r2rOut, 0, 255, 0, (7 * r2rOctMax) + 1);
        uint8_t oct = cv / 7;
        uint8_t semi = scales[r2rScale][cv % 7];
        r2rOut = ((oct * 12) + semi) * VoltPerTone;
        pwm_set_gpio_level(OUT6, r2rOut);
    }

    sleep_us(100);
}

void setup1()
{
    initOLED();
}

void loop1()
{
    uint16_t potValue = pot.getValue();
    int8_t encValue = enc.getDirection();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint8_t btn2 = buttons[2].getState();

    if (btn2 == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        requiresUpdate |= menuControl.select(encValue);
        encValue = 0;
    }

    requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);

    dispOLED();
    sleep_ms(1);

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
    // Serial.println();
    // }
}
