/*!
 * Quantizer
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

class Quantizer
{
public:
    Quantizer(uint16_t pwmReso)
    : VoltPerTone((float)(pwmReso) / 12.0 / 5.0)
    , _pwmReso(pwmReso)
    , _scaleIndex(0)
    {
    }

    uint16_t Quantize(uint16_t value)
    {
        uint8_t oct = value / 7;
        uint8_t semi = Scales[_scaleIndex][value % 7];
        return ((oct * 12) + semi) * VoltPerTone;
    }

    void setScale(uint8_t value)
    {
        _scaleIndex = constrain(value, 0, MaxScales - 1);
    }

    // const uint8_t *getScaleKeys(uint8_t index) { return Scales[index]; }
    // const char *getScaleName(uint8_t index) { return ScaleNames[index]; }
    float getVoltPerTone() { return VoltPerTone; }

public:
    const float VoltPerTone;
    const uint8_t Scales[10][7] =
    {
            {0, 2, 4, 5, 7, 9, 11}, // ionian / major
            {0, 2, 3, 5, 7, 9, 10}, // dorian
            {0, 1, 3, 5, 7, 8, 10}, // phrygian
            {0, 2, 4, 6, 7, 9, 11}, // lydian
            {0, 2, 4, 5, 7, 9, 10}, // mixolydian
            {0, 2, 3, 5, 7, 8, 10}, // aeolian / natural minor
            {0, 1, 3, 5, 6, 8, 10}, // locrian
            {0, 2, 3, 4, 7, 9, 12},  // m.blues
            {0, 1, 4, 5, 7, 8, 10}, // spanish
            {0, 2, 4, 7, 9,12, 14},  // luoyin
    };
    const uint16_t MaxScales = sizeof(Scales) / sizeof(Scales[0]);

    const char ScaleNames[10][5] = {
        "maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"
    };

private:
    uint16_t _pwmReso;
    uint8_t _scaleIndex;
};
