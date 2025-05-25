/*!
 * PatternSeq
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

#define SEQUENCER_TOTAL 6
#define STEP_MAX 16
#define STEP_TOTAL 64 // 16 * 4
#define BEATS_TOTAL 41

// 数値は4小節単位での休打切り替え。例：0b1010(10)→8小節目、16小節目ヒット
const static uint8_t beats[BEATS_TOTAL][STEP_MAX]
{
    {15,15,15,15, 15,15,15,15, 15,15,15,15, 15,15,15,15}, // 0
    {15, 0,15,15, 15, 0,15,15, 15, 0,15,15, 15, 0,15,15},
    {15,15,15, 0, 15,15,15, 0, 15,15,15, 0, 15,15,15, 0},
    {15, 0,15, 0, 15, 0,15, 0, 15, 0,15, 0, 15, 0,15, 0},
    {15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0},
    { 0, 0,15, 0,  0, 0,15, 0,  0, 0,15, 0,  0, 0,15, 0},
    { 0, 0, 0, 0, 15, 0, 0, 0,  0, 0, 0, 0, 15, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0, 15, 0, 0, 0},
    { 0, 0, 0, 0, 15, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0},
    { 0, 0, 0, 0,  0, 0, 0,15,  0,15, 0, 0,  0, 0, 0, 0},

    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 10
    { 8,15,15,15, 10,15,15,15,  8,15,15,15, 10,15,15,15},
    {15, 0,15, 0, 15, 0,15,10, 15,10,15, 0, 15, 0,15, 0},
    {15, 0,15, 0, 15, 0,15, 0, 15, 0,15,10, 15, 8,15, 8},
    { 0, 0,15, 0,  0, 0,15, 0,  0,10,15, 0,  0, 0,15, 8},
    { 0, 0,15,15,  0, 0,15,15,  0,15,15, 8,  0, 8,15, 0},
    { 0, 0,15, 0,  0, 0,15, 0,  0, 0,15, 0,  0, 0,15,10},
    {15, 0,15,10, 15, 0,10,15,  0,15, 0, 0, 15, 8, 0,15},
    { 0,15, 0,15,  0,15, 0,15,  0,15, 0,15,  0,15,10,15},
    { 0,15, 0,15, 15,10, 0,15,  0,15,15, 0, 15, 8, 0,15},

    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 20
    { 0, 0, 0, 0, 15, 0, 0,15,  0,15, 0, 0, 15, 0, 0, 0},
    { 0, 0, 0, 0, 15, 0, 0,15,  0, 8, 0, 0, 15, 0, 0, 0},
    { 0, 0, 0, 0, 15, 0, 0,15,  0, 8, 0, 0, 15, 0, 0,10},
    { 0, 8, 0, 0, 15, 0, 0,10,  0, 8, 0, 0, 15, 0, 0, 0},
    { 0, 0, 0, 0, 15, 0, 0, 0,  0, 0, 0, 0, 15, 8, 0,10},
    { 0,15,10,15,  0, 8,10,15,  0, 8,15,10,  2, 8,15,10},
    { 0,15, 0,15,  0,15,15, 8,  0,15,10,15,  0,15, 0,15},
    { 0,15, 0, 0, 15, 0, 0,15,  0, 0,15, 8,  0,15, 0, 0},
    { 0, 0, 0,15,  0, 0,15, 0,  0,15, 0, 8,  0, 0,15, 0},

    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 30
    {15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 8},
    {15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 8, 0},
    {15, 0,15, 0,  0, 0,15, 0,  0, 0, 0,15,  0, 0,10, 0},
    {15, 0, 8, 0,  0, 0, 0, 0,  0, 0,15, 0,  0,10, 0, 0},
    {15, 0,10, 0,  0, 0, 0, 0,  0, 0,15, 8,  0, 0, 0, 0},
    {15, 0, 0,15,  0, 8, 0, 0,  0, 0,15, 0,  0,10, 0, 8},
    {15, 0, 0,15,  0, 0,15, 0,  0, 0,15, 0,  0, 0,10, 0},
    {15, 0, 0,15, 15, 0,15, 0, 15, 0,15, 0,  0, 0,10, 8},
    {15, 0, 0,15,  0, 0,15, 0,  0,15, 0, 0, 15, 0, 0, 8},

    {15, 0,15, 0,  0,15, 0, 0, 15, 0, 0,15,  0, 0,15, 8}, // 40
};

struct PatternSetting
{
public:
    PatternSetting()
    {
    }

public:
    char disp_name[3] = {0};
    int8_t currentPattern;
};

class PatternSeq
{
public:
    PatternSeq()
    : _prevCurrentStep(15)
    {
    }

    void updateDisplay(U8G2 *pU8g2, int8_t currentStep, int8_t selector, int8_t mode, bool requiresUpdate = true, bool isUpdateStepIndicator = true)
    {
        uint8_t origin_x = 15;
        uint8_t origin_y = 15;
        uint8_t seqXStep = 7;
        uint8_t seqYStep = 8;

        // ステップインジケータ部分のみ更新
        if (requiresUpdate == false)
        {
            if (isUpdateStepIndicator == false)
            {
                return;
            }
            pU8g2->setDrawColor(0);
            pU8g2->drawBox(origin_x + (seqXStep * _prevCurrentStep), origin_y - 2, seqXStep, 3);
            pU8g2->setDrawColor(2);
            pU8g2->drawBox(origin_x + (seqXStep * currentStep), origin_y - 2, seqXStep, 3);
            pU8g2->updateDisplayArea(0, 1, 16, 1);
            _prevCurrentStep = currentStep;
            return;
        }

        _prevCurrentStep = currentStep;

        pU8g2->clearBuffer();
        if (isUpdateStepIndicator == true)
            pU8g2->drawBox(origin_x + (seqXStep * currentStep), origin_y - 2, seqXStep, 3);

        if (mode == 0)
            pU8g2->drawBox(10, (origin_y + 1) + (seqYStep * (selector % SEQUENCER_TOTAL)), 4, 6);
        else
            pU8g2->drawStr(10, (origin_y + 1) + (seqYStep * (selector % SEQUENCER_TOTAL)), ">");

        pU8g2->setFont(u8g2_font_7x14B_tf);
        pU8g2->drawStr(0, 0, "PATTERN SEQ");
        pU8g2->setFont(u8g2_font_5x8_tf);

        for (int8_t i = 0; i < SEQUENCER_TOTAL + 1; ++i)
        {
            pU8g2->drawStr(0, origin_y + (seqYStep * i), _patternSetting[i].disp_name);
        }

        for (int8_t i = 4; i < STEP_MAX; i += 4)
        {
            pU8g2->drawVLine(origin_x + (seqXStep * i), origin_y, 63);
        }

        for (int8_t x = 0; x < STEP_MAX; ++x)
        {
            for (int8_t y = 0; y < SEQUENCER_TOTAL; ++y)
            {
                uint8_t beat = beats[_patternSetting[y].currentPattern][x];
                int8_t offset = 1;
                uint8_t odd = (beat & (1 + 4));
                uint8_t even = (beat & (2 + 8));
                // 1小節：左上、2小節：右上、3小節：左下、4小節：右下に四角で表示
                if (odd > 0)
                {
                    pU8g2->drawBox((origin_x + offset) + (seqXStep * x),
                                   (origin_y + offset) + (seqYStep * y) + (odd == 4 ? 3 : 0),
                                   3, odd == 5 ? 6 : 3);
                }
                if (even > 0)
                {
                    pU8g2->drawBox((origin_x + offset) + (seqXStep * x) + 3,
                                   (origin_y + offset) + (seqYStep * y) + (even == 8 ? 3 : 0),
                                   3, even == 10 ? 6 : 3);
                }
            }
        }

        pU8g2->sendBuffer();
    }

    bool addSelectPattern(uint8_t seqIndex, int8_t value)
    {
        int16_t pattern = constrain(_patternSetting[seqIndex].currentPattern + value, 0, (BEATS_TOTAL - 1));
        bool result = false;
        if (_patternSetting[seqIndex].currentPattern != pattern)
        {
            result = true;
        }

        _patternSetting[seqIndex].currentPattern = pattern;

        return result;
    }

    uint8_t getBeat(uint8_t seqIndex, uint8_t step)
    {
        return beats[_patternSetting[seqIndex].currentPattern][step & (STEP_MAX - 1)];
    }

    void setPattern(uint8_t seqIndex, uint8_t value)
    {
        _patternSetting[seqIndex].currentPattern = value;
    }

    uint8_t getPattern(uint8_t seqIndex)
    {
        return _patternSetting[seqIndex].currentPattern;
    }

    void setPatternName(uint8_t seqIndex, const char *value)
    {
        strncpy(_patternSetting[seqIndex].disp_name, value, 2);
    }

private:
    PatternSetting _patternSetting[SEQUENCER_TOTAL];
    int8_t _prevCurrentStep;
};