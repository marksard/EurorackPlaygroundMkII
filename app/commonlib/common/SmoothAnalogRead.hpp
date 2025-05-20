/*!
 * AnalogRead class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>
#include <hardware/adc.h>

/// @brief 12bitADC専用
class SmoothAnalogRead
{
public:
    SmoothAnalogRead() {}
    SmoothAnalogRead(byte pin)
    {
        init(pin);
    }
    
    /// @brief ピン設定
    /// @param pin
    void init(byte pin)
    {
        _pin = pin;
        _value = 0;
        _valueOld = 65535;
        pinMode(pin, INPUT);
        adc_init();
    }

    uint16_t analogReadDirectFast()
    {
        _valueOld = _value;
        _value = readPinFast();
        // _value = (_valueOld + _value + 1) >> 1;
        return _value;
    }

    // uint16_t analogReadDirect()
    // {
    //     _value = readPin();
    //     return _value;
    // }

    uint16_t analogReadDropLow4bit()
    {
        _valueOld = _value;
        uint16_t value = readPinFast();
        if (value < 0x16)
        {
            _value = 0;
        }
        else if (value > 0xFFB)
        {
            _value = 0xFFF;
        }
        else
        {
            _value = value & 0xFFF0;
        }
        return _value;
    }

    uint16_t analogRead(bool smooth = true)
    {
        _valueOld = _value;
        // 平均＋ローパスフィルタ仕様
        int aval = 0;
        for (byte i = 0; i < 4; ++i)
        {
            aval += readPinFast();
        }
        aval = (_valueOld + (aval >> 2) + 1) >> 1;
        // さらにローパス(端数を4095に調整)
        if (smooth)
        {
            _value = (_value * 0.95) + (aval * 0.05024);
            return _value;
        }
        _value = aval;
        return _value;
    }

    uint16_t getValue()
    {
        return _value;
    }

    bool hasChanged()
    {
        return _valueOld != _value;
    }

protected:
    byte _pin;
    uint16_t _value;
    uint16_t _valueOld;

    /// @brief ピン値読込
    /// @return
    /// @note 必要最低限の処理のため、これを利用する場合すべてのADCは同一スレッドで読むこと
    virtual uint16_t readPinFast()
    {
        adc_select_input(_pin - A0);
        return adc_read();
    }

    /// @brief ピン値読込
    /// @return
    virtual uint16_t readPin()
    {
        return ::analogRead(_pin);
    }
};
