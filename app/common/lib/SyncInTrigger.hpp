/*!
 * TriggerInterface
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "TriggerInterface.hpp"
#include "EdgeChecker.hpp"

class SyncInTrigger : public TriggerInterface
{
public:
    SyncInTrigger(byte pin, ulong aliveTimeMillis = 2000)
        : SyncInTrigger()
    {
        _edge.init(pin, aliveTimeMillis);
        _bpmReso = 4;
    }

    SyncInTrigger()
    {
        _start = 0;
    }

    void start() override
    {
        _start = 1;
    }

    void stop() override
    {
        _start = 0;
    }

    bool ready() override
    {
        if (!_start)
            return false;

        return _edge.isEdgeHigh();
    }

    bool isStart() override
    {
        return _start ? true : false;
    }

    bool isAlive()
    {
        return _edge.isAlive();
    }

    void setMills(int millSec) override {}
    int getMills() override { return _edge.getDurationMills(); }

    bool setBPM(byte bpm, byte bpmReso) override
    {
        _bpmReso = bpmReso;
        return false;
    }
    bool setBPM(byte bpm) override { return true; }
    byte getBPM() override { return _edge.getBPM(_bpmReso); }
    byte getBPMReso() override { return _bpmReso; }

    void setPin(byte pin)
    {
        _edge.setPin(pin);
    }

protected:
    byte _start;
    byte _bpmReso;
    EdgeChecker _edge;
};
