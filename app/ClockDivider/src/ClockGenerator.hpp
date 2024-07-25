/*!
 * ClockGenerator class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once
#include <Arduino.h>

#define WAVE_LENGTH 4096
#define WAVE_LENGTH_BIT 12

#define WAVE_INDEX_DIV_BIT 0
#define WAVE_HEIGHT 4096

#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class ClockGenerator
{
public:
    // 上位ビットをindex範囲にする
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    ClockGenerator()
    {
    }

    void init(float clock)
    {
        _frequency = 0.0;
        _phaseAccum = 0;
        _tuningWordM = 0;
        _widthHalf = WAVE_LENGTH >> 1;
        _widthM1 = WAVE_LENGTH -1;
        _heightHalf = WAVE_HEIGHT >> 1;
        _heightM1 = WAVE_HEIGHT -1;
        _interruptClock = clock;
        _value = 0;
    }

    uint16_t update()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        uint32_t indexHeight = (index >> WAVE_INDEX_DIV_BIT);
        _value = indexHeight < _widthHalf ? _heightM1 : 0;
        return _value;
    }

    void setFrequency(float frequency)
    {
        _frequency = frequency;
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * ((float)frequency / _interruptClock);
    }

    void reset()
    {
        _phaseAccum = 0;
        _tuningWordM = 0;
    }

    float getFrequency() { return _frequency; }
    uint16_t getValue() { return _value; }
    uint8_t getPulse() { return _value > 0 ? HIGH : LOW; }

private:
    float _frequency;
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    uint16_t _widthHalf;
    uint16_t _widthM1;
    uint32_t _heightHalf;
    uint32_t _heightM1;
    float _interruptClock;
    uint16_t _value;
};
