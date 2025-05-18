/*!
 * Button class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>

class Button
{
public:
    Button() {}
    Button(uint8_t pin)
    {
        init(pin);
    }
    
    /// @brief ピン設定
    /// @param pin
    void init(uint8_t pin, bool needWait = true, bool pullup = true, bool invert = false)
    {
        _pin = pin;
        _pinState = 0;
        _holdStage = 0;
        _holdTime = 500*1000;
        _lastMicros = 0;
        _leadLastMicros = 0;
        _lastResult = 0;
        _invert = invert;

        pinMode(pin, pullup ? INPUT_PULLUP : INPUT);

        // 空読み
        for(int i = 0; i < 8; ++i)
        {
            uint8_t value = readPin();
            _pinState = (_pinState << 1) | value;
        }

        _needWait = needWait;
    }

    /// @brief ボタン状態を取得
    /// @return 0:None 1:Button down 2:Button up 3:Holding 4:Holded
    inline uint8_t getState()
    {
        uint8_t value = readPin();
        ulong micro = micros();

        if (_needWait && (micros() - _leadLastMicros) < 1000)
        {
            return _lastResult;
        }

        _leadLastMicros = micros();
        _lastResult = 0;

        // 簡単チャタ取り
        _pinState = (_pinState << 1) | value;

        // Holding
        if (_holdStage == 2)
        {
            // Holded
            if (_pinState == 0x0F)
            {
                _holdStage = 0;
                _lastResult = 4;
            }
            else if (_pinState == 0x00)
            {
                _lastResult = 3;
            }

            return _lastResult;
        }

        // Button down
        if (_pinState == 0xF0)
        {
            _lastResult = 1;
            _holdStage = 0;
        }
        // Button up
        else if (_pinState == 0x0F)
        {
            _lastResult = 2;
            _holdStage = 0;
        }
        // Hold check
        else if (_pinState == 0x00)
        {
            // Start hold check
            if (_holdStage == 0)
            {
                _holdStage = 1;
                _lastMicros = micros();
            }
            // Hold confirm
            else if ((micros() - _lastMicros) >= _holdTime)
            {
                _holdStage = 2;
            }
        }

        return _lastResult;
    }

    void setHoldTime(int16_t mills)
    {
        _holdTime = mills * 1000;
    }

    uint8_t getValue() { return _lastResult; }

protected:
    uint8_t _pin;
    uint8_t _pinState;
    uint8_t _holdStage;
    ulong _lastMicros;
    ulong _holdTime;
    ulong _leadLastMicros;
    uint8_t _lastResult;
    bool _needWait;
    bool _invert;

    /// @brief ピン値読込
    /// @return
    virtual uint8_t readPin()
    {
        bool result = gpio_get(_pin);
        return _invert ? !result : result;
    }
};
