/*!
 * AnalogRead class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>
#include <hardware/adc.h>

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
        _value = readPinFast();
        return _value;
    }

    uint16_t analogReadDirect()
    {
        _value = readPin();
        return _value;
    }

    uint16_t analogReadDropLow4bit()
    {
        _valueOld = _value;
        uint16_t value = readPin();
        _value = map((((value + _value) >> 1) & 0xFFF0), 0, 4080, 0, 4095);
        // _value = ((value + _value + 2) >> 1) & 0xFFF0;
        return _value;
    }

    uint16_t analogRead(bool smooth = true, bool fast = true)
    {
        _valueOld = _value;
        // アナログ入力。平均＋ローパスフィルタ仕様
        int aval = 0;
        for (byte i = 0; i < 16; ++i)
        {
            aval += fast ? readPinFast() : readPin();
        }
        // 実測による調整
        // 10bit
        // aval = max(((aval >> 4) - 3), 0);
        // _value = (_value * 0.8) + (aval * 0.2014);
        // 12bit
        aval = max(((aval >> 4) - 16), 0);
        _value = (_value * 0.95) + (aval * 0.05044);
        // Serial.print(aval);
        // Serial.print(",");
        // Serial.println(_value);
        _value = smooth ? _value : aval;
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
