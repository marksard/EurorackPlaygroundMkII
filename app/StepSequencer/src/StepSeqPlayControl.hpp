/*!
 * StepSeqPlayControl
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "../../common/lib/TriggerInterface.hpp"
#include "../../common/lib/PollingTimeEvent.hpp"
#include "../../common/lib/SyncInTrigger.hpp"
#include "../../common/lib/TriggerOut.hpp"
#include "../../common/basic_definition.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"

class StepSeqPlayControl
{
public:
    enum CLOCK
    {
        INT = 0,
        EXT,
        IGNORE,
    };

    const float VoltPerTone;

public:
    StepSeqPlayControl(U8G2 *pU8g2, uint16_t pwmReso)
        : VoltPerTone((float)(pwmReso) / 12.0 / 5.0)
        , _pwmReso(pwmReso)
        , _ssm(), _ssv(pU8g2, 0, 16)
        , _settingPos(DEF_MAX_STEP_M1, 0, DEF_MAX_STEP_M1)
        , _octUnder(0, 5, 0, 5)
        , _octUpper(0, 5, 0, 5)
        , _gateMin(StepSeqModel::Gate::_, StepSeqModel::Gate::Max, StepSeqModel::Gate::_, StepSeqModel::Gate::Max)
        , _gateMax(StepSeqModel::Gate::_, StepSeqModel::Gate::Max, StepSeqModel::Gate::_, StepSeqModel::Gate::Max)
        , _gateInitial(StepSeqModel::Gate::_, StepSeqModel::Gate::Max, StepSeqModel::Gate::_, StepSeqModel::Gate::Max)
        , _syncIn(GATE, 4000)
        , _syncOut(OUT1)
        , _accOut(OUT2)
        , _gateOut(OUT3)
    {
        _pTrigger = NULL;
        _clock = CLOCK::IGNORE;
        _seqReadyCount = 0;
        _seqReadyCountMax = 0;
        _requestGenerateSequence = false;
        _requestResetAllSequence = false;

        _octUnder.set(-1);
        _octUpper.set(2);
        _gateMin.set(StepSeqModel::Gate::H);
        _gateMax.set(StepSeqModel::Gate::Max);
        _gateInitial.set(StepSeqModel::Gate::H);

        _gateOut.setDuration(10);
        _accOut.setDuration(200);
        _syncOut.setDuration(10);

        setClockMode(CLOCK::INT);
    }

    bool isStart()
    {
        return _pTrigger->isStart();
    }

    void start()
    {
        _pTrigger->start();
    }

    void stop()
    {
        _pTrigger->stop();
    }

    void reset()
    {
        _seqReadyCount = 0;
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
    }

    void resetRange()
    {
        _ssm.keyStep.pos.setLimit(0, 16);
        _ssm.gateStep.pos.setLimit(0, 16);
    }

    void resetMode()
    {
        _ssm.keyStep.setMode(Step::Mode::Forward);
        _ssm.gateStep.setMode(Step::Mode::Forward);
    }

    void setClockMode(CLOCK clock)
    {
        if (_clock == clock)
            return;

        _clock = clock;
        if (clock == CLOCK::INT)
        {
            if (_pTrigger != NULL)
            {
                _polling.setBPM(_polling.getBPM(), _polling.getBPMReso());
                _seqReadyCountMax = _polling.getBPMReso() / 4;
                if (_pTrigger->isStart())
                {
                    _polling.start();
                }
            }
            _pTrigger = &_polling;
        }
        else if (clock == CLOCK::EXT)
        {
            if (_pTrigger != NULL)
            {
                _seqReadyCountMax = 4 / 4;
                if (_pTrigger->isStart())
                {
                    _syncIn.start();
                }
            }
            _pTrigger = &_syncIn;
        }
    }

    CLOCK getClockMode() { return _clock; }

    void addBPM(int8_t value)
    {
        if (_clock == CLOCK::EXT)
            return;
        if (value == 0)
            return;
        uint8_t bpm = constrain(_pTrigger->getBPM() + value, 0, 255);
        _pTrigger->setBPM(bpm);
    }

    void setBPM(byte bpm, byte bpmReso)
    {
        if (_clock == CLOCK::EXT)
            return;
        if (_pTrigger->setBPM(bpm, bpmReso))
        {
            _seqReadyCountMax = bpmReso / 4;
            // Serial.print(_seqReadyCountMax);
            // Serial.println();
        }
    }

    uint8_t getBPM() { return _pTrigger->getBPM(); }
    int8_t getScale() { return _ssm._scaleIndex.get(); }
    uint8_t getScaleKey(uint8_t scale, uint8_t key) { return _ssm.getScaleKey(scale, key); }

    void setSettingPos(int8_t value)
    {
        _settingPos.set(value);
    }

    void addGate(int8_t value)
    {
        uint8_t pos = _settingPos.get();
        uint8_t current = _ssm.getGate(pos);
        _ssm.setGate(pos,
                     (StepSeqModel::Gate)constrain((current + value),
                                                   StepSeqModel::Gate::_, StepSeqModel::Gate::G));
    }

    void addNote(int8_t value)
    {
        uint8_t pos = _settingPos.get();
        uint8_t currentOct = _ssm.getOctave(pos) * MAX_SCALE_KEY;
        uint8_t currentKey = _ssm.getKey(pos);
        int8_t note = constrain((int8_t)(currentOct + currentKey + value), 0, 35); // 7key*5oct
        _ssm.setOctave(pos, note / MAX_SCALE_KEY);
        _ssm.setKey(pos, note % MAX_SCALE_KEY);
    }

    void toggleAcc()
    {
        uint8_t pos = _settingPos.get();
        uint8_t current = _ssm.getAcc(pos);
        _ssm.setAcc(pos, (current + 1) & 1);
    }

    void addGateLimit(int8_t min, int8_t max)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin() + min,
                                   _ssm.gateStep.pos.getMax() + max);
    }

    void addKeyLimit(int8_t min, int8_t max)
    {
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin() + min,
                                  _ssm.keyStep.pos.getMax() + max);
    }

    void addGateKeyStart(int8_t gate, int8_t key)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin() + gate,
                                   _ssm.gateStep.pos.getMax());
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin() + key,
                                  _ssm.keyStep.pos.getMax());
    }

    void addGateKeyEnd(int8_t gate, int8_t key)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin(),
                                   _ssm.gateStep.pos.getMax() + gate);
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin(),
                                  _ssm.keyStep.pos.getMax() + key);
    }

    void addGateStepMode(int8_t value)
    {
        _ssm.gateStep.addMode(value);
    }

    void addKeyStepMode(int8_t value)
    {
        _ssm.keyStep.addMode(value);
    }

    void addScale(int8_t value)
    {
        _ssm._scaleIndex.add(value);
    }

    void setScale(int8_t value)
    {
        _ssm._scaleIndex.set(value);
    }

    void moveSeq(int8_t value)
    {
        if (value == 0)
            return;
        _ssm.moveSeq(value > 0 ? StepSeqModel::SeqMove::RIGHT : StepSeqModel::SeqMove::LEFT);
    }

    void addGateLen(int8_t value)
    {
        _ssm.gateLenAdder.add(value);
    }

    int8_t getGateLen()
    {
        return _ssm.gateLenAdder.get();
    }

    void addOctave(int8_t value)
    {
        _ssm.octaveAdder.add(value);
    }

    int8_t getOctave()
    {
        return _ssm.octaveAdder.get();
    }

    void setVOct(int16_t value)
    {
        pwm_set_gpio_level(OUT5, value);
    }

    bool getPlayGate() { return _ssm.getPlayGate() > 0; }

    void addOctUnder(int8_t value) { _octUnder.add(value); }
    void addOctUpper(int8_t value) { _octUpper.add(value); }
    void addGateMin(int8_t value) { _gateMin.add(value); }
    void addGateMax(int8_t value) { _gateMax.add(value); }
    void addGateInitial(int8_t value) { _gateInitial.add(value); }

    void setOctUnder(int8_t value) { _octUnder.set(value); }
    void setOctUpper(int8_t value) { _octUpper.set(value); }
    void setGateMin(int8_t value) { _gateMin.set(value); }
    void setGateMax(int8_t value) { _gateMax.set(value); }
    void setGateInitial(int8_t value) { _gateInitial.set(value); }
    int8_t getOctUnder() { return _octUnder.get(); }
    int8_t getOctUpper() { return _octUpper.get(); }
    int8_t getGateMin() { return _gateMin.get(); }
    int8_t getGateMax() { return _gateMax.get(); }
    int8_t getGateInitial() { return _gateInitial.get(); }

    int8_t getGatePos() { return _ssm.gateStep.pos.get(); }

    int getStepDulation()
    {
        int length = _pTrigger->getMills() * _seqReadyCountMax;
        return length;
    }
    
    bool isExternalSyncAlive()
    {
        if (_clock == CLOCK::EXT)
        {
            return _syncIn.isAlive();
        }

        return true;
    }

    void updateGateOut(bool updateDuration)
    {
        if (updateDuration)
        {
            int length = getStepDulation();
            int duration = _ssm.getGateDuration();
            // Serial.print(duration);
            // Serial.print(",");
            // Serial.print(length);
            // Serial.print(",");
            _syncOut.setDuration(length >> 3);
            _syncOut.update(1);
            length = map(duration, 0, 100, 0, length);
            // Serial.print(length);
            // Serial.print(",");
            // Serial.println();
            _gateOut.setDuration(length);
        }
        else
        {
            _accOut.update(0);
            _syncOut.update(0);
        }

        uint8_t playGate = _ssm.getPlayGate();
        if (playGate == StepSeqModel::Gate::_)
        {
            _gateOut.set(0);
        }
        else if (playGate == StepSeqModel::Gate::G)
        {
            _gateOut.set(1);
        }
        else
        {
            _gateOut.update(updateDuration == true);
        }

        _accOut.update(_ssm.getPlayAcc() != 0);
    }

    void setSwingIndex(int8_t value) { swingIndex = constrain(value, 0, 4); }
    int8_t getSwingIndex() { return swingIndex; }
    int8_t beat16Count = 0;
    int8_t swingIndex = 0;
    int8_t swingTimeSets[4][4] = {
        {  0,  0,  0,  0 },
        {  1, -1,  1, -1 },
        {  2, -2,  2, -2 },
        {  3, -3,  3, -3 }
    };

    int8_t updateProcedure()
    {
        int8_t result = 0;

        if (_clock == CLOCK::EXT)
        {
            if (!_syncIn.isAlive())
            {
                reset();
            }
        }

        if (!_pTrigger->ready())
        {
            updateGateOut(false);
            return result;
        }

        if (_seqReadyCount >= _seqReadyCountMax + swingTimeSets[swingIndex][beat16Count])
        {
            _seqReadyCount = 0;
            _ssm.keyStep.nextPlayStep();
            _ssm.gateStep.nextPlayStep();
            beat16Count = (beat16Count + 1) & 3;
            // Serial.print(_ssm.keyStep.pos.get());
            // Serial.print(",note: ");
            // Serial.print(_ssm.getPlayNote());
            // Serial.println();
        }

        if (_seqReadyCount == 0)
        {
            if (_ssm.gateStep.pos.getMin() == _ssm.gateStep.pos.get())
            {
                if (_requestGenerateSequence)
                {
                    _requestGenerateSequence = false;
                    generateSequence();
                }
                if (_requestResetAllSequence)
                {
                    _requestResetAllSequence = false;
                    resetAllSequence();
                }
            }

            // Serial.print("--> play");
            // Serial.print(_ssm.getPlayNote());
            // Serial.println();
            uint8_t semi = _ssm.getPlayNote();
            int16_t voct = semi * VoltPerTone;
            voct = constrain(voct - PWMCVDCOutputErrorLUT[semi], 0, _pwmReso - 1);
            setVOct(voct);
            updateGateOut(true);
            result = 1;
        }
        else
        {
            updateGateOut(false);
        }

        _seqReadyCount++;

        return result;
    }

    void updateDisplay()
    {
        uint8_t key = _ssm.keyStep.pos.get();
        uint8_t gate = _ssm.gateStep.pos.get();
        uint8_t keyStart = _ssm.keyStep.pos.getMin();
        uint8_t keyEnd = _ssm.keyStep.pos.getMax();
        uint8_t gateStart = _ssm.gateStep.pos.getMin();
        uint8_t gateEnd = _ssm.gateStep.pos.getMax();

        _ssv.dispSteps(keyStart, keyEnd, gateStart, gateEnd, _ssm._octaves, _ssm._keys, (uint8_t *)_ssm._gates, (uint8_t *)_ssm._accs, _ssm.gateLenAdder.get(), _ssm.octaveAdder.get());
        _ssv.dispKeyPos(key);
        _ssv.dispGatePos(gate);
        _ssv.dispSettingPos(_settingPos.get());
    }

    void updateLED()
    {
        uint8_t key = _ssm.keyStep.pos.get();
        uint8_t keyStart = _ssm.keyStep.pos.getMin();
        if (keyStart == key)
        {
            gpio_put(LED1, HIGH);
        }
        else
        {
            gpio_put(LED1, LOW);
        }        
    }

    void requestGenerateSequence()
    {
        _requestGenerateSequence = true;
    }

    void requestResetAllSequence()
    {
        _requestResetAllSequence = true;
    }

    void resetAllSequence()
    {
        _ssm.resetSequence(_gateInitial.get());
    }

    void generateSequence(bool resetSyncCount = true)
    {
        _seqReadyCount = 0;
        _ssm.generateSequence(_octUnder.get(), _octUpper.get(),
                           _gateMin.get(), _gateMax.get(), _gateInitial.get());

        if (resetSyncCount == false)
        {
            return;
        }
        
        _ssm.keyStep.setMode(Step::Mode::Forward);
        _ssm.gateStep.setMode(Step::Mode::Forward);
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
        // _ssm.printSeq();
    }

private:
    StepSeqModel _ssm;
    StepSeqView _ssv;
    U8G2 *_pU8g2;
    TriggerInterface *_pTrigger;
    PollingTimeEvent _polling;
    SyncInTrigger _syncIn;
    CLOCK _clock;
    uint16_t _pwmReso;

    uint8_t _seqReadyCount;
    uint8_t _seqReadyCountMax;
    LimitValue<int8_t> _settingPos;

    bool _requestGenerateSequence;
    bool _requestResetAllSequence;

    LimitValue<int8_t> _octUnder;
    LimitValue<int8_t> _octUpper;
    LimitValue<int8_t> _gateMin;
    LimitValue<int8_t> _gateMax;
    LimitValue<int8_t> _gateInitial;

    TriggerOut _gateOut;
    TriggerOut _accOut;
    TriggerOut _syncOut;
};
