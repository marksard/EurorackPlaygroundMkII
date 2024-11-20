/*!
 * PollingTimeEvent
 * ポーリングベースのイベント検出クラス
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "TriggerInterface.hpp"

class PollingTimeEvent : public TriggerInterface
{
public:
    PollingTimeEvent()
    {
        _start = 0;
        setBPM(133, 4);
    }

    void start() override
    {
        if (_start)
            return;
        _lastMicros = micros();
        _start = 1;
    }

    void stop() override
    {
        _lastMicros = micros();
        _start = 0;
    }

    bool ready() override
    {
        if (!_start)
            return false;

        long now = micros();
        if ((now - _lastMicros) >= triggerTime)
        {
            // Serial.print(now - _lastMicros);
            // Serial.print(",");
            // Serial.println(triggerTime);
            _lastMicros = now;
            return true;
        }

        return false;
    }

    bool isStart() override
    {
        return _start ? true : false;
    }

    void setMills(int millSec) override
    {
        triggerTime = millSec * 1000;
    }

    int getMills() override { return triggerTime / 1000; }

    bool setBPM(byte bpm, byte bpmReso) override
    {
        if (_bpm == bpm && _bpmReso == bpmReso)
            return false;
        _bpm = bpm;
        _bpmReso = bpmReso;
        triggerTime = (long)((60.0 / (bpm * bpmReso)) * 1000000.0);
        // Serial.print(bpm);
        // Serial.print(",");
        // Serial.print(bpmReso);
        // Serial.print(",");
        // Serial.print(triggerTime);
        // Serial.print(",");
        // Serial.println();
        return true;
    }

    bool setBPM(byte bpm) override { return setBPM(bpm, _bpmReso); }
    byte getBPM() override { return _bpm; }
    byte getBPMReso() override { return _bpmReso; }

private:
    byte _start;
    byte _bpm;
    byte _bpmReso;
    long triggerTime;
    long _lastMicros;
};
