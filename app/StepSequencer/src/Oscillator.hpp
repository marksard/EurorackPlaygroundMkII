/*!
 * Oscillator class
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

#define WAVE_HEIGHT_BIT (WAVE_LENGTH_BIT - WAVE_INDEX_DIV_BIT)
#define FOLD_TRI_MAX ((WAVE_HEIGHT >> 1) - (WAVE_HEIGHT >> 4))

#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class Oscillator
{
public:
    enum Wave
    {
        TRI,
        NOISE,
        MAX = NOISE,
    };
    const char waveName[Wave::MAX + 1][9] = {"TRIANGLE", "W-NOISE "};
    // 上位12ビット(0~4095)をindex範囲にする
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    Oscillator()
    {
    }

    void init(float clock)
    {
        _phaseAccum = 0;
        _tuningWordM = 0;
        _wave = Wave::TRI;
        _widthHalf = WAVE_LENGTH >> 1;
        _widthM1 = WAVE_LENGTH - 1;
        _heightHalf = WAVE_HEIGHT >> 1;
        _heightM1 = WAVE_HEIGHT - 1;
        _interruptClock = clock;
        _lastValue = 0;
    }

    // value範囲＝DAC、PWM出力範囲：0-4095(12bit)
    // index範囲：0-4095(12bit)
    // とした。sine以外は単純な演算のみで済む
    uint16_t getWaveValue()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        uint32_t indexHeight = (index >> WAVE_INDEX_DIV_BIT);
        uint16_t value = 0;
        switch (_wave)
        {
        case Wave::TRI:
            value = getTriangle(index, indexHeight);
        case Wave::NOISE:
            value = getRandom16(WAVE_HEIGHT);
            break;
        default:
            value = 0;
            break;
        }

        // simplest linear interpolation
        value = (_lastValue + value) >> 1;
        _lastValue = value;
        return value;
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

    uint16_t getRandom16(uint16_t max) { return getRandomFast() % max; }

private:
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    Wave _wave;
    uint16_t _widthHalf;
    uint16_t _widthM1;
    uint32_t _heightHalf;
    uint32_t _heightM1;
    float _interruptClock;
    uint16_t _lastValue;
    uint32_t m_w = 1;
    uint32_t m_z = 2;

    inline uint16_t getTriangle(uint32_t index, uint32_t indexHeight)
    {
        return index < _widthHalf ? (indexHeight << 1) : ((_heightM1 - indexHeight) << 1);
    }

    inline uint32_t getRandomFast()
    {
        // Multiply-with-carry
        m_z = 36969L * (m_z & 65535L) + (m_z >> 16);
        m_w = 18000L * (m_w & 65535L) + (m_w >> 16);
        return (m_z << 16) + m_w;
    }
};
