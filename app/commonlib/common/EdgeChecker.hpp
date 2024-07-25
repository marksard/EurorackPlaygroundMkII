/*!
 * EdgeChecker class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>

class EdgeChecker
{
public:
    EdgeChecker() {}
    EdgeChecker(uint8_t pin)
    {
        init(pin);
    }
    
    /// @brief ピン設定
    /// @param pin
    void init(uint8_t pin)
    {
        setPin(pin);
        // 空読み
        for(int i = 0; i < 8; ++i)
        {
            isEdgeHigh();
        }
    }

    /// @brief 立上がりエッジ検出
    /// @return 
    inline bool isEdgeHigh()
    {
        uint8_t value = readPin();
        uint8_t edge = 0;
        if (value == 1 && _lastValue == 0)
        {
            edge = 1;
            _duration = micros() - _lastMicros;
            _lastMicros = micros();
        }
        _lastValue = value;
        return edge;
    }

    /// @brief 立下がりエッジ検出
    /// @return 
    inline bool isEdgeLow()
    {
        uint8_t value = readPin();
        uint8_t edge = value == 0 && _lastValue == 1 ? 1 : 0;
        _lastValue = value;
        return edge;
    }

    inline uint16_t getBPM(byte bpmReso = 4)
    {
        return (60.0 * 1000000.0) / (_duration * bpmReso);
    }

    inline int getDurationMills()
    {
        return _duration / 1000;
    }

    inline int getDurationMicros()
    {
        return _duration;
    }

    inline bool getValue() { return _lastValue; }

    inline void setPin(byte pin)
    {
        _pin = pin;
        pinMode(pin, INPUT);
    }

protected:
    uint8_t _pin;
    uint8_t _lastValue;
    long _lastMicros;
    int _duration;

    /// @brief ピン値読込
    /// @return
    virtual uint8_t readPin()
    {
        return gpio_get(_pin);
        // return digitalRead(_pin);
    }
};
