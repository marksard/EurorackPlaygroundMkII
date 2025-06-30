/*!
 * TriggerOut class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>

class TriggerOut
{
public:
    TriggerOut() {}
    TriggerOut(uint8_t pin)
    {
        init(pin);
    }
    
    /// @brief ピン設定
    /// @param pin
    void init(uint8_t pin)
    {
        _pin = pin;
        _duration = 10;

        pinMode(pin, OUTPUT);
    }

    /// @brief 出力ON/OFF更新
    ///        ON中ONせず、次回update時の時間経過によりOFF
    /// @param status 
    inline void update(int status)
    {
        // if (_status == 0 && status != 0)
        // {
        //     _status = 1;
        //     _lastMillis = millis();
        //     writePin(_status);
        // }
        // else if (_status == 1 && (millis() - _lastMillis) > _duration)
        // {
        //     _status = 0;
        //     writePin(_status);
        // }

        if (_status == 0 && status != 0)
        {
            _status = 1;
            _lastMillis = millis();
            writePin(1);
        }
        else if (_status == 1)
        {
            if ((millis() - _lastMillis) > _duration)
            {
                _status = 2;
                writePin(0);
            }
        }
        else if (_status == 2)
        {
            if (status == 0)
            {
                _status = 0;
                writePin(0);
            }
        }
    }

    /// @brief トリガーゲート取得
    /// @param status トリガー状態 (0 or 1)
    /// @param triggerMode トリガーモード (0：statusそのまま出力 1：トリガー出力)
    inline int getTriggerGate(int status, int triggerMode)
    {
        // Serial.print(status);
        // Serial.print(",");
        // Serial.println();
        if (triggerMode == 0)
        {
            return status;
        }

        if (_status == 0 && status != 0)
        {
            // Serial.println("edge on");
            _status = 1;
            _lastMillis = millis();
            return 1;
        }
        else if (_status == 1)
        {
            if ((millis() - _lastMillis) > _duration)
            {
                _status = 2;
                return 0;
            }

            return 1;
        }
        else if (_status == 2)
        {
            if (status == 0)
            {
                // Serial.println("edge off");
                _status = 0;
            }
        }

        return 0;
    }

    inline void set(int status)
    {
        writePin(status);
    }

    /// @brief トリガー長設定
    /// @param duration ms
    inline void setDuration(uint16_t duration)
    {
        _duration = duration;
    }

protected:
    uint8_t _pin;
    uint16_t _duration;
    int _status;
    unsigned long _lastMillis;

    /// @brief ピン値読込
    /// @return
    virtual void writePin(int status)
    {
        gpio_put(_pin, status);
    }
};
