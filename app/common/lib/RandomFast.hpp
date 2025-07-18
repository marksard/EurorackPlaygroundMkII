/*!
 * RandomFast class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

class RandomFast
{
public:
    RandomFast()
    {
        m_w = 1;
        m_z = 2;
    }

    inline void randomSeed(ulong seed)
    {
        m_w = seed;
        m_z = m_w + seed;
    }

    inline uint32_t getRandomFast()
    {
        // Multiply-with-carry
        m_z = 36969L * (m_z & 65535L) + (m_z >> 16);
        m_w = 18000L * (m_w & 65535L) + (m_w >> 16);
        return (m_z << 16) + m_w;
    }

    inline int16_t getRandom16(int16_t max) { return getRandomFast() % max; }
    inline int16_t getRandom16(int16_t min, int16_t max)
    {
        uint32_t result = getRandomFast();
        int16_t scaled_result = (int16_t)(result % (max - min)) + min;
        if (scaled_result < min)
        {
            scaled_result = min;
        }
        else if (scaled_result > max)
        {
            scaled_result = max;
        }
        // Serial.println(scaled_result);
        return scaled_result;
    }

private:
    uint32_t m_w;
    uint32_t m_z;
};