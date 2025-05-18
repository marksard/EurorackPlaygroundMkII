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
    EdgeChecker(uint8_t pin, ulong aliveTimeMillis = 1000)
    {
        init(pin, aliveTimeMillis);
    }

    /// @brief ピン設定
    /// @param pin
    void init(uint8_t pin, ulong aliveTimeMillis = 1000)
    {
        setPin(pin);
        _aliveTimeMicros = aliveTimeMillis * 1000;
        // 空読み
        for(int i = 0; i < 8; ++i)
        {
            isEdgeHigh();
        }
    }

    // /// @brief ピン設定
    // /// @param pin
    // void init(uint8_t pin, gpio_irq_callback_t callback, ulong aliveTimeMillis = 1000)
    // {
    //     setPin(pin);
    //     _aliveTimeMicros = aliveTimeMillis * 1000;
    //     gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_EDGE_FALL, true, callback);
    // }

    /// @brief エッジ状態更新
    /// @param value エッジ検出値
    /// @return エッジ判定（立ち上がりON）
    /// readPinを使わず割り込みなどで更新する
    inline bool updateEdge(uint8_t value)
    {
        bool edge = 0;
        if (value != 0 && _lastValue == 0)
        {
            ulong now = micros();
            edge = true;
            _duration = now - _lastMicros;
            _lastMicros = now;
        }
        _lastValue = value;
        _lastEdge = edge;
        return edge;
    }

    /// @brief 立上がりエッジ検出
    /// @return 
    inline bool isEdgeHigh()
    {
        uint8_t value = readPin();
        return updateEdge(value);
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
    inline bool getEdge() { return _lastEdge; }

    inline void setPin(byte pin)
    {
        _pin = pin;
        pinMode(pin, INPUT);
    }

    inline bool isAlive()
    {
        // return micros() < (_lastMicros + _aliveTimeMicros);
        return (micros() - _lastMicros) < _aliveTimeMicros;
    }

protected:
    uint8_t _pin;
    uint8_t _lastValue;
    bool _lastEdge;
    ulong _lastMicros;
    int _duration;
    ulong _aliveTimeMicros;

    /// @brief ピン値読込
    /// @return
    virtual uint8_t readPin()
    {
        return gpio_get(_pin);
    }
};
