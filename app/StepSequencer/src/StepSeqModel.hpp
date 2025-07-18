/*!
 * StepSeqModel
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once
#include <Arduino.h>
#include "../../common/lib/RandomFast.hpp"

#define DEF_MAX_STEP 16
#define DEF_MAX_STEP_M1 (DEF_MAX_STEP - 1)
#define MAX_SCALE_KEY 7
#define MAX_SCALES 10
#define MAX_SCALES_M1 (MAX_SCALES - 1)

#define MAX_GATE_TIMINGS 4
#define MAX_GATE_STEP 16


template <typename vs = uint8_t>
void initArray(vs *pArray, vs size)
{
    for (vs i = 0; i < size; ++i)
        pArray[i] = 0;
}

template <typename vs = uint8_t>
void printArray(const char* lavel, vs *pArray, vs size)
{
    Serial.print(lavel);

    for (vs i = 0; i < size; ++i)
    {
        Serial.print(pArray[i]);
        Serial.print(",");
    }

    Serial.println("");
}

template <typename vs = int8_t>
class LimitValue
{
public:
    LimitValue(vs limitMax)
    {
        _value = 0;
        _min = 0;
        _max = 0;
        _limitMin = 0;
        _limitMax = limitMax;
        setLimit(_min, _max);
    }

    LimitValue(vs limitMax, vs min, vs max)
    {
        _value = 0;
        _limitMin = 0;
        _limitMax = limitMax;
        setLimit(min, max);
    }

    LimitValue(vs limitMin, vs limitMax, vs min, vs max)
    {
        _value = 0;
        _limitMin = limitMin;
        _limitMax = limitMax;
        setLimit(min, max);
    }

    void set(vs value) { _value = constrain(value, _min, _max); }
    vs get() { return _value; }
    void add(vs value) { _value = constrain(_value + value, _min, _max); }

    void setLimit(vs min, vs max)
    {
        if (min == _min && max == _max) return;
        _min = MAX(min, _limitMin);
        _max = MAX(MIN(max, _limitMax), _min);
        set(_value);
    }
    vs getMin() { return _min; }
    vs getMax() { return _max; }
    vs getDiff() { return _max - _min; }

private:
    vs _value;
    vs _min;
    vs _max;
    vs _limitMin;
    vs _limitMax;
};

class Step
{
public:
    static const uint8_t MAX_STEP = DEF_MAX_STEP;
    static const uint8_t MAX_STEP_M1 = DEF_MAX_STEP_M1;
    enum Mode
    {
        Forward,
        Reverse,
        TurnBack,
        Max,
    };

public:
    Step()
        : pos(MAX_STEP_M1, 0, MAX_STEP_M1)
    {
    }

    void nextPlayStep()
    {
        if (_mode == Mode::Forward)
        {
            if (_playCount + 1 > pos.getMax())
                _playCount = pos.getMin();
            else
                _playCount++;
            pos.set(_playCount);
        }
        else if (_mode == Mode::Reverse)
        {
            if (_playCount - 1 < pos.getMin())
                _playCount = pos.getMax();
            else
                _playCount--;
            pos.set(_playCount);
        }
        else if (_mode == Mode::TurnBack)
        {
            if (_playCount + 1 > pos.getDiff() + 1)
                pos.add(-1);
            else
                pos.add(1);

            if (_playCount > pos.getDiff() * 2)
                _playCount = 0;
            else
                _playCount++;
        }
    }

    void resetPlayStep()
    {
        if (_mode == Mode::Forward)
        {
            _playCount = pos.getMin();
            pos.set(_playCount);
        }
        else if (_mode == Mode::Reverse)
        {
            _playCount = pos.getMax();
            pos.set(_playCount);
        }
        else if (_mode == Mode::TurnBack)
        {
            _playCount = 0;
            pos.set(_playCount);
        }
    }

    void addMode(int8_t value)
    {
        if (value == 0) return;
        _mode = (Mode)constrain((Mode)_mode + value, Mode::Forward, Mode::TurnBack);
    }

    void setMode(Mode mode) { _mode = mode; }
    uint8_t getMode() { return _mode; }
    uint8_t getPlayCount() { return _playCount; }

public:
    LimitValue<int8_t> pos;

private:
    Mode _mode;
    int16_t _playCount;
};

class StepSeqModel
{
public:
    static const uint8_t MAX_STEP = DEF_MAX_STEP;
    static const uint8_t MAX_STEP_M1 = DEF_MAX_STEP_M1;
    enum Gate
    {
        _, // None
        S, // Short
        H, // Half
        L, // Long
        G, // Glide
        Max,
    };
    
    static constexpr const char GateDisp[Gate::Max][5] = {"-", "S", "H", "L", "G"};

    const uint8_t GateDuration[Gate::Max] = {0, 25, 50, 75, 100};

    enum SeqMove
    {
        LEFT,
        RIGHT,
    };

public:
    StepSeqModel()
    : _scaleIndex(MAX_SCALES_M1, 0, MAX_SCALES_M1)
    , gateLenAdder((int8_t)Gate::G * -1, (int8_t)Gate::G, (int8_t)Gate::G * -1, (int8_t)Gate::G)
    , octaveAdder(-1, 2, -1, 2)
    {
        initArray(_keys, MAX_STEP);
        initArray(_octaves, MAX_STEP);
        initArray(_accs, MAX_STEP);
        initArray((uint8_t *)_gates, MAX_STEP);
        _scaleIndex.set(2);
    }

    void setKey(uint8_t step, uint8_t value) { _keys[constrain(step, 0, MAX_STEP)] = value; }
    void setOctave(uint8_t step, uint8_t value) { _octaves[constrain(step, 0, MAX_STEP)] = value; }
    void setAcc(uint8_t step, uint8_t value) { _accs[constrain(step, 0, MAX_STEP)] = value; }
    void setGate(uint8_t step, StepSeqModel::Gate value) { _gates[constrain(step, 0, MAX_STEP)] = value; }
    uint8_t getKey(uint8_t value) { return _keys[value]; }
    uint8_t getOctave(uint8_t value) { return _octaves[value]; }
    uint8_t getAcc(uint8_t value) { return _accs[value]; }
    uint8_t getGate(uint8_t value) { return _gates[value]; }

    uint8_t getPlayKey() { return _keys[keyStep.pos.get()]; }
    uint8_t getPlayOctave() { return _octaves[keyStep.pos.get()]; }
    uint8_t getPlayAcc() { return _accs[gateStep.pos.get()]; }
    uint8_t getPlayGate() { return constrain(_gates[gateStep.pos.get()] + gateLenAdder.get(), 0, (uint8_t)Gate::G); }

    uint8_t getPlayNote() { return (constrain(getPlayOctave() + octaveAdder.get(), 0, 5) * 12) + Scales[_scaleIndex.get()][getPlayKey()]; }
    uint8_t getScaleKey(uint8_t scale, uint8_t key) { return Scales[scale][key]; }

    uint8_t getGateDuration() { return GateDuration[getPlayGate()]; }

    void randomSeed(ulong seed) { _rand.randomSeed(seed); }
    int16_t rand(int16_t max) { return _rand.getRandom16(0, max); }
    int16_t rand(int16_t min, int16_t max) { return _rand.getRandom16(min, max); }

    void moveSeq(SeqMove move)
    {
        if (move == SeqMove::LEFT)
        {
            uint8_t backKey = _keys[0];
            uint8_t backOct = _octaves[0];
            uint8_t backAcc = _accs[0];
            Gate backGate = _gates[0];
            for (uint8_t i = 0; i < MAX_STEP; ++i)
            {
                uint8_t destIndex = constrainCyclic(i + 1, 0, (int)MAX_STEP_M1);
                _keys[i] = _keys[destIndex];
                _octaves[i] = _octaves[destIndex];
                _accs[i] = _accs[destIndex];
                _gates[i] = _gates[destIndex];
            }
            _keys[MAX_STEP_M1] = backKey;
            _octaves[MAX_STEP_M1] = backOct;
            _accs[MAX_STEP_M1] = backAcc;
            _gates[MAX_STEP_M1] = backGate;
        }
        else
        {
            uint8_t backKey = _keys[MAX_STEP_M1];
            uint8_t backOct = _octaves[MAX_STEP_M1];
            uint8_t backAcc = _accs[MAX_STEP_M1];
            Gate backGate = _gates[MAX_STEP_M1];
            for (uint8_t i = MAX_STEP_M1; i > 0; --i)
            {
                uint8_t destIndex = constrainCyclic(i - 1, 0, (int)MAX_STEP_M1);
                _keys[i] = _keys[destIndex];
                _octaves[i] = _octaves[destIndex];
                _accs[i] = _accs[destIndex];
                _gates[i] = _gates[destIndex];
            }
            _keys[0] = backKey;
            _octaves[0] = backOct;
            _accs[0] = backAcc;
            _gates[0] = backGate;
        }
    }

    void generateSequence(int8_t octUnder, int8_t octUpper, int8_t gateMin, int8_t gateMax, int8_t gateInitial, bool leaveVibes = true)
    {
        // Serial.println("generateSequence");
        // Serial.print("octUnder: ");
        // Serial.println(octUnder);
        // Serial.print("octUpper: ");
        // Serial.println(octUpper);
        // Serial.print("gateMin: ");
        // Serial.println(gateMin);
        // Serial.print("gateMax: ");
        // Serial.println(gateMax);
        this->randomSeed(micros());
        byte geteSelect = this->rand(MAX_GATE_TIMINGS);
    
        for (byte i = 0; i < StepSeqModel::MAX_STEP; ++i)
        {
            // タイミングマップにランダムでタイミングをorして足す
            StepSeqModel::Gate gate = GateMap[geteSelect][i] == 1 ? 
                (StepSeqModel::Gate)this->rand(gateMin, gateMax + 1) : 
                (StepSeqModel::Gate)(this->rand(2) ? getGate(i) : gateInitial);
            setGate(i, gate);
    
            // 変更前のメロディーラインをランダムに残して繋がりを持たせる
            if (leaveVibes && this->rand(2))
            {
                continue;
            }
    
            // 基音(C0) + 音階はスケールに従いつつランダムで + オクターブ上下移動をランダムで(-1 or 0 ~ 2 * 12)
            // 0 ~ 24 + スケール音
            setOctave(i, (this->rand(octUnder, octUpper)));
            setKey(i, this->rand(MAX_SCALE_KEY));
            setAcc(i, gate != StepSeqModel::Gate::_ && this->rand(0, 6) == 1 ? 1 : 0);
        }
    }
    
    void resetSequence(int8_t gateInitial)
    {
        // Serial.println("resetSequence\n");
    
        for (byte i = 0; i < StepSeqModel::MAX_STEP; ++i)
        {
            setGate(i, (StepSeqModel::Gate)gateInitial);
            setOctave(i, 0);
            setKey(i, 0);
            setAcc(i, 0);
         }
    }
    
    void printSeq()
    {
        Serial.println("--------");
        printArray("key: ", _keys, MAX_STEP);
        printArray("oct: ", _octaves, MAX_STEP);
        printArray("acc: ", _accs, MAX_STEP);
        printArray("gate:", (uint8_t *)_gates, MAX_STEP);
        // for (byte i = 0; i < StepSeqModel::MAX_STEP * 3; ++i)
        // {
        //     Serial.print("i:");
        //     Serial.print(i);
        //     Serial.print(", ");
        //     Serial.print("Key Step:");
        //     Serial.print(keyStep.pos.get());
        //     Serial.print(", ");
        //     Serial.print("Gate Step:");
        //     Serial.print(gateStep.pos.get());
        //     Serial.print(", ");
        //     Serial.print("Key:");
        //     Serial.print(getPlayKey());
        //     Serial.print(", ");
        //     Serial.print("Octave:");
        //     Serial.print(getPlayOctave());
        //     Serial.print(", ");
        //     Serial.print("Gate:");
        //     Serial.print(getPlayGate());
        //     Serial.print(", ");
        //     Serial.print("Note:");
        //     Serial.print(getPlayNote());
        //     Serial.print(", ");
        //     Serial.print("Acc:");
        //     Serial.print(getPlayAcc());
        //     Serial.print(", ");
        //     Serial.println();
        //     keyStep.nextPlayStep();
        //     gateStep.nextPlayStep();
        // }
    }

public:
    // スケール
    const uint8_t Scales[MAX_SCALES][MAX_SCALE_KEY] =
    {
        {0, 2, 4, 5, 7, 9, 11}, // ionian / major
        {0, 2, 3, 5, 7, 9, 10}, // dorian
        {0, 1, 3, 5, 7, 8, 10}, // phrygian
        {0, 2, 4, 6, 7, 9, 11}, // lydian
        {0, 2, 4, 5, 7, 9, 10}, // mixolydian
        {0, 2, 3, 5, 7, 8, 10}, // aeolian / natural minor
        {0, 1, 3, 5, 6, 8, 10}, // locrian
        {0, 2, 3, 4, 7, 9,  0}, // m.blues
        {0, 1, 4, 5, 7, 8, 10}, // spanish
        {0, 2, 4, 7, 9, 0,  2}, // luoyin
    };

    // メロディーを成立しやすくするための発音タイミングマップ
    // この上にランダムでタイミングを追加してランダムかつメロディーを成立しやすく
    const uint8_t GateMap[MAX_GATE_TIMINGS][MAX_GATE_STEP] = {
        {1, 0, 1, 1,
        1, 1, 0, 1,
        0, 1, 1, 0,
        1, 0, 0, 1,},
        {0, 1, 0, 1,
        1, 0, 1, 0,
        0, 1, 1, 0,
        0, 1, 0, 1,},
        {1, 1, 0, 1,
        1, 1, 1, 1,
        1, 1, 1, 1,
        1, 0, 1, 1,},
        {1, 0, 1, 0,
        1, 0, 0, 1,
        0, 1, 1, 0,
        1, 0, 0, 1}};

public:
    Step keyStep;
    Step gateStep;
    LimitValue<int8_t> gateLenAdder;
    LimitValue<int8_t> octaveAdder;

public:
    uint8_t _keys[MAX_STEP];
    uint8_t _octaves[MAX_STEP];
    uint8_t _accs[MAX_STEP];
    StepSeqModel::Gate _gates[MAX_STEP];
    LimitValue<int8_t> _scaleIndex;
    RandomFast _rand;

private:

    template <typename vs = int8_t>
    vs constrainCyclic(vs value, vs min, vs max)
    {
        if (value > max)
            return min;
        if (value < min)
            return max;
        return value;
    }
};
