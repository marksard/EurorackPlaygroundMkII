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
    {15, 0,15,15, 15, 0,15,15, 15,10,15,15, 15, 0,15,15},
    {15, 0,15, 0, 15, 0,15,10, 15,10,15, 0, 15, 0,15, 0},
    {15, 0,15, 0, 15, 0,15, 0, 15, 0,15, 8, 15, 8,15, 8},
    { 0, 0,15, 0,  0, 0,15, 0,  0,10,15, 0,  0, 8,15, 0},
    { 0, 0,15, 0,  0,15, 0, 8,  0, 0,15, 0,  0, 8,15, 0},
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
    { 0, 0,10, 0,  0, 8,10, 0,  0, 8, 0,10,  2, 8, 0,10},
    { 0, 0, 0, 0,  0, 0, 0,10,  0,10,10, 0,  0, 0, 0,10},
    { 0, 8, 0, 0,  0, 0, 0,10,  8,10, 0, 0,  0, 0, 8,10},
    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0,15, 0,15},

    { 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0}, // 30
    {15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0,10},
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
    int16_t currentPattern;
};

class PatternSeq
{
public:
    PatternSeq()
    {

    }

    void updateDisplay(U8G2 *pU8g2, int8_t currentStep, int8_t selector, int8_t mode)
    {
        uint8_t origin_x = 15;
        uint8_t origin_y = 15;
        uint8_t seqXStep = 7;
        uint8_t seqYStep = 8;

        pU8g2->setFont(u8g2_font_7x14B_tf);
        pU8g2->drawStr(0, 0, "PATTERN SEQ");
        pU8g2->setFont(u8g2_font_5x8_tf);

        for (int8_t i = 0; i < SEQUENCER_TOTAL + 1; ++i)
        {
            pU8g2->drawStr(0, origin_y + (seqYStep * i), patternSetting[i].disp_name);
            pU8g2->drawHLine(origin_x, origin_y + (seqYStep * i), 127);
        }

        for (int8_t i = 0; i < STEP_MAX + 1; ++i)
        {
            pU8g2->drawVLine(origin_x + (seqXStep * i), origin_y, 63);
        }

        for (int8_t x = 0; x < STEP_MAX; ++x)
        {
            for (int8_t y = 0; y < SEQUENCER_TOTAL; ++y)
            {
                uint8_t beat = beats[patternSetting[y].currentPattern][x];
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
            pU8g2->drawBox(10, (origin_y + 1) + (seqYStep * (selector % SEQUENCER_TOTAL)), 4, 6);
        else
            pU8g2->drawStr(10, (origin_y + 1) + (seqYStep * (selector % SEQUENCER_TOTAL)), ">");
    }

    void addSelectPattern(uint8_t seqIndex, int8_t value)
    {
        patternSetting[seqIndex].currentPattern = constrain(patternSetting[seqIndex].currentPattern + value, 0, (BEATS_TOTAL - 1));
    }

    uint8_t getBeat(uint8_t seqIndex, uint8_t step)
    {
        return beats[patternSetting[seqIndex].currentPattern][step & (STEP_MAX - 1)];
    }

    void setPattern(uint8_t seqIndex, uint8_t value) { patternSetting[seqIndex].currentPattern = value; }
    void setPatternName(uint8_t seqIndex, const char* value)
    {
        strncpy(patternSetting[seqIndex].disp_name, value, 2);
    }

private:
    PatternSetting patternSetting[SEQUENCER_TOTAL];
};