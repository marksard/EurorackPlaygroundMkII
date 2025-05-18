/*!
 * BlinkLED class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#define WAVE_LENGTH 4096
#define WAVE_LENGTH_BIT 12

#define bit_11
#ifdef bit_10
#define WAVE_INDEX_DIV_BIT 2 // WAVE_LENGTH_BIT - WAVE_HEIGHT
#define WAVE_HEIGHT 1024
#elif defined(bit_11)
#define WAVE_INDEX_DIV_BIT 1 // WAVE_LENGTH_BIT - WAVE_HEIGHT
#define WAVE_HEIGHT 2048
#else
#define WAVE_INDEX_DIV_BIT 0 // WAVE_LENGTH_BIT - WAVE_HEIGHT
#define WAVE_HEIGHT 4096
#endif

#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class BlinkLED
{
public:
    enum Wave
    {
        SQU,
        TRI,
        MAX = TRI,
    };
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    BlinkLED()
    {
    }

    void init(float clock)
    {
        _phaseAccum = 0;
        _tuningWordM = 0;
        _wave = Wave::SQU;
        _widthHalf = WAVE_LENGTH >> 1;
        _height = WAVE_HEIGHT - 1;
        _interruptClock = clock;
        _level = 10;
    }

    uint16_t getWaveValue()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        uint32_t indexHeight = (index >> WAVE_INDEX_DIV_BIT);
        uint16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < _widthHalf ? _height : 0;
            break;
        case Wave::TRI:
            value = index < _widthHalf ? (indexHeight << 1) : ((_height - indexHeight) << 1);
            break;
        default:
            value = 0;
            break;
        }
        // Serial.println(value);
        return value >> _level;
    }

    void setFrequency(float frequency)
    {
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * ((float)frequency / _interruptClock);
    }

    bool setWave(Wave value)
    {
        bool result = _wave != value;
        if (value < 0 && value > Wave::MAX)
            return false;
        _wave = value;
        return result;
    }

    void setLevel(uint8_t level, uint8_t max)
    {
        _level = map(level, 0, max, 6, 0);
    }

private:
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    Wave _wave;
    uint16_t _widthHalf;
    uint16_t _height;
    uint8_t _level;
    float _interruptClock;
};
