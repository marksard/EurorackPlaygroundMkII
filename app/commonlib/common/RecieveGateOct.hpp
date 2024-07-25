/*!
 * Recieve OCT/CV class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "SmoothAnalogRead.hpp"

#define GATE_OFF 0
#define GATE_ON 1
#define GATE_THRU 2

class RecieveGateOct
{
public:
    RecieveGateOct()
    {
        _gatePrev = LOW;
        _gateOn = GATE_OFF;
    }

    RecieveGateOct(byte gatePin, byte vOctPin)
        : RecieveGateOct()
    {
        init(gatePin, vOctPin);
    }

    void init(byte gatePin, byte vOctPin)
    {
        _gatePin = gatePin;
        _vOctRead.init(vOctPin);
        pinMode(gatePin, INPUT_PULLUP);
    }

    bool ready()
    {
        _gateOn = isGateOn();
        // return _gateOn != GATE_THRU;
        return true;
    }

    /// @brief ノートオンしたかどうか
    /// @return true: note on
    bool isNoteOn()
    {
        if (_gateOn != GATE_ON)
            return false;

        // Serial.println("note on :");

        return true;
    }

    /// @brief ノートオフしたかどうか
    /// @return true: note off
    bool isNoteOff()
    {
        if (_gateOn != GATE_OFF)
            return false;

        // Serial.println("note off :");

        return true;
    }

    /// @brief VOct電圧値取得
    /// @return VOct電圧値
    uint16_t getVOct()
    {
        return _vOctRead.analogRead(false); // 平均を取得
    }

protected:
    byte _gatePin;
    byte _gatePrev;
    byte _gateOn;
    SmoothAnalogRead _vOctRead;

    /// @brief ピン値読込
    /// @return
    virtual uint16_t readGatePin()
    {
        return ::digitalRead(_gatePin);
    }

private:
    /// @brief ゲートピンのon/off判定
    /// @return 
    byte isGateOn()
    {
        byte gate = readGatePin();
        byte result = GATE_THRU;
        if (gate == HIGH && _gatePrev == LOW)
        {
            result = GATE_ON;
        }
        else if (gate == LOW && _gatePrev == HIGH)
        {
            result = GATE_OFF;
        } 

        // if (gate != _gatePrev)
        // {
        //     Serial.print("gate:");
        //     Serial.print(gate);
        //     Serial.print(",");
        //     Serial.println(_gatePrev);
        // }
        _gatePrev = gate;
        return result;
    }
};
