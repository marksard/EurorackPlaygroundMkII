/*!
 * DDS Multi Wave Oscillator class
 * Copyright 2025 marksard
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include <Arduino.h>
#include "wavetable/sine_10bit_4096.h"
#include "wavetable/sine_11bit_4096.h"
#include "wavetable/sine_12bit_4096.h"
#include "RandomFast.hpp"

#define WAVE_LENGTH_BIT 12
#define WAVE_LENGTH 4096
#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class MultiWaveOsc
{
public:
    enum Wave
    {
        SQU,
        SAW,
        MUL_TRI,
        TRI,
        SINE,
        NOISE,
        MAX = NOISE,
    };
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    MultiWaveOsc()
    {
    }

    void init(float clock, uint8_t heightBit = 11)
    {
        setWaveHeightBit(heightBit);
        _phaseAccum = 0;
        _tuningWordM = 0;
        _wave = Wave::SQU;
        _interruptClock = clock;
        _rnd.randomSeed(OSC_WAVE_BIT32 - 1);
        _lastValue = 0;
        _isFolding = false;
    }

    int16_t getWaveValue()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        uint32_t indexHeight = (index >> _widthHeightDiffBit);
        uint32_t indexPhase = (indexHeight + _phaseShift) % _height;
        uint32_t indexPulse = (indexHeight + _pulseShift) % _height;
        int16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < indexPulse ? _heightM1 : 0;
            break;
        case Wave::SAW:
            value = applyPhaseShift(value, indexHeight, indexPhase);
            break;
        case Wave::MUL_TRI:
        {
            value = getTriangle(index, indexHeight);
            uint16_t value2 = getTriangle(index, indexPhase);
            value = ((value * value2) >> _heightBit) % _height;
        }
        break;
        case Wave::TRI:
            value = getTriangle(index, indexHeight);
            if (_isFolding)
                value = applyEzFolding(value);
            break;
        case Wave::SINE:
            value = _pSineTable[index];
            if (_isFolding)
                value = applyEzFolding(value);
            break;
        case Wave::NOISE:
            value = _rnd.getRandom16(_height);
            break;
        default:
            value = 0;
            break;
        }

        // simplest linear interpolation
        value = (_lastValue + value) >> 1;
        _lastValue = value;
        return value;
    }

    bool setWave(Wave value)
    {
        bool result = _wave != value;
        if (value < 0 || value > Wave::MAX)
            return false;
        _wave = value;
        return result;
    }

    Wave getWave() { return _wave; }

    void setFrequency(float frequency)
    {
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * (frequency / _interruptClock);
    }

    void setFreqFromNoteIndex(int8_t index)
    {
        index = constrain(index, 0, 127);
        setFrequency(noteFreq[index]);
    }

    static float getFreqFromNoteIndex(uint8_t index)
    {
        index = constrain(index, 0, 127);
        return noteFreq[index];
    }

    inline uint8_t getNoteNameFromFreq(float freq)
    {
        // 指定した周波数に最も近い12平均律ノートのインデックスを返す関数
        int nearestIndex = 0;
        float minDiff = fabs(noteFreq[0] - freq);
        for (int i = 1; i < sizeof(noteFreq) / sizeof(noteFreq[0]); ++i)
        {
            float diff = fabs(noteFreq[i] - freq);
            if (diff < minDiff)
            {
                minDiff = diff;
                nearestIndex = i;
            }
        }
        return nearestIndex;
    }

    void getFineTuneRangeFromNoteIndex(int8_t index, float &minFine, float &plusFine)
    {
        index = constrain(index, 0, 127);
        int8_t indexM1 = constrain(index - 1, 0, 127);
        int8_t indexP1 = constrain(index + 1, 0, 127);
        minFine = (noteFreq[indexM1] - noteFreq[index]) * 0.5f;
        plusFine = (noteFreq[indexP1] - noteFreq[index]) * 0.5f;
    }

    void reset()
    {
        _phaseAccum = 0;
        _tuningWordM = 0;
    }

    void addPhaseShift(int16_t value)
    {
        value = value << 6;
        _phaseShift = constrain(_phaseShift + value, 0, _heightHalf - 1);
    }

    bool setPhaseShift(int16_t value)
    {
        bool result = _phaseShift != value;
        _phaseShift = constrain(value, -_heightHalf, _heightHalf - 1);
        return result;
    }

    int16_t getPhaseShift() { return _phaseShift; }

    void addFolding(int16_t value)
    {
        value = value << 6;
        _folding = constrain(_folding + value, 0, _heightHalf - 1);
    }

    bool setFolding(int16_t value)
    {
        bool result = _folding != value;
        _folding = constrain(value, -_heightHalf, _heightHalf - 1);
        return result;
    }

    int16_t getFolding() { return _folding; }

    void addPulseShift(int16_t value)
    {
        value = value << 6;
        _pulseShift = constrain(_pulseShift + value, -_heightHalf, _heightHalf - 1);
    }

    bool setPulseWidth(int16_t value)
    {
        value = abs(value + _heightHalf) % _heightM1;
        bool result = _pulseShift != value;
        _pulseShift = value;
        return result;
    }

    int16_t getPulseWidth() { return _pulseShift; }

    void startFolding(int16_t value) { _isFolding = value != 0 ? true : false; }

protected:
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    Wave _wave;
    uint16_t _widthHalf;
    uint16_t _widthM1;
    uint16_t _heightHalf;
    uint16_t _heightM1;
    float _interruptClock;
    int16_t _phaseShift;
    int16_t _folding;
    int16_t _pulseShift;
    uint16_t _lastValue;
    bool _isFolding;
    RandomFast _rnd;
    uint8_t _heightBit;
    uint16_t _height;
    uint8_t _widthHeightDiffBit;
    const uint16_t *_pSineTable;

    void setWaveHeightBit(uint8_t heightBit)
    {
        _heightBit = heightBit;
        _height = 1 << _heightBit;
        _widthHeightDiffBit = WAVE_LENGTH_BIT - _heightBit;
        _heightM1 = _height - 1;
        _heightHalf = _height >> 1;
        _widthM1 = WAVE_LENGTH - 1;
        _widthHalf = WAVE_LENGTH >> 1;
        _phaseShift = 0;
        _folding = 0;
        _pulseShift = _heightHalf;

        if (heightBit == 10)
        {
            _pSineTable = sine_10bit;
        }
        else if (heightBit == 11)
        {
            _pSineTable = sine_11bit;
        }
        else if (heightBit == 12)
        {
            _pSineTable = sine_12bit;
        }
        else
        {
            _pSineTable = nullptr; // Invalid height bit
        }
    }

    inline uint16_t getTriangle(uint32_t index, uint32_t indexHeight)
    {
        return index < _widthHalf ? (indexHeight << 1) : ((_heightM1 - indexHeight) << 1);
    }

    inline uint16_t applyPhaseShift(uint16_t value, uint32_t indexHeight, uint32_t indexPhase)
    {
        // Phase shift with normalize. (Not exact, but light.)
        uint16_t sum = indexHeight + indexPhase;
        uint16_t diff = abs((long)indexHeight - (long)indexPhase) % _heightHalf;
        return map(sum,
                   diff,
                   _heightM1,
                   0,
                   _heightHalf - 1);
    }

    // Light-weight Wavefolder
    inline uint16_t applyEzFolding(uint16_t value)
    {
        // Wavefolding
        int16_t foldLower = abs(_folding);
        int16_t foldUpper = (_heightM1 - foldLower);
        if (value > foldUpper)
            value = foldUpper - (value - foldUpper);
        if (value < foldLower)
            value = foldLower + (foldLower - value);
        // Normalize
        value = map(value, foldLower, foldUpper, 0, _heightM1);
        return value;
    }

    static constexpr float noteFreq[128] = {
        8.175798916, //     "C-2",
        8.661957218, //     "C#-2",
        9.177023997, //     "D-2",
        9.722718241, //     "D#-2",
        10.30086115, //     "E-2",
        10.91338223, //     "F-2",
        11.56232571, //     "F#-2",
        12.24985737, //     "G-2",
        12.9782718,  //     "G#-2",
        13.75,       //     "A-2",
        14.56761755, //     "A#-2",
        15.43385316, //     "B-2",
        16.35159783, //     "C-1",
        17.32391444, //     "C#-1",
        18.35404799, //     "D-1",
        19.44543648, //     "D#-1",
        20.60172231, //     "E-1",
        21.82676446, //     "F-1",
        23.12465142, //     "F#-1",
        24.49971475, //     "G-1",
        25.9565436,  //     "G#-1",
        27.5,        //     "A-1",
        29.13523509, //     "A#-1",
        30.86770633, //     "B-1",
        32.70319566, //     "C0",
        34.64782887, //     "C#0",
        36.70809599, //     "D0",
        38.89087297, //     "D#0",
        41.20344461, //     "E0",
        43.65352893, //     "F0",
        46.24930284, //     "F#0",
        48.9994295,  //     "G0",
        51.9130872,  //     "G#0",
        55,          //     "A0",
        58.27047019, //     "A#0",
        61.73541266, //     "B0",
        65.40639133, //     "C1",
        69.29565774, //     "C#1",
        73.41619198, //     "D1",
        77.78174593, //     "D#1",
        82.40688923, //     "E1",
        87.30705786, //     "F1",
        92.49860568, //     "F#1",
        97.998859,   //     "G1",
        103.8261744, //     "G#1",
        110,         //     "A1",
        116.5409404, //     "A#1",
        123.4708253, //     "B1",
        130.8127827, //     "C2",
        138.5913155, //     "C#2",
        146.832384,  //     "D2",
        155.5634919, //     "D#2",
        164.8137785, //     "E2",
        174.6141157, //     "F2",
        184.9972114, //     "F#2",
        195.997718,  //     "G2",
        207.6523488, //     "G#2",
        220,         //     "A2",
        233.0818808, //     "A#2",
        246.9416506, //     "B2",
        261.6255653, //     "C3",
        277.182631,  //     "C#3",
        293.6647679, //     "D3",
        311.1269837, //     "D#3",
        329.6275569, //     "E3",
        349.2282314, //     "F3",
        369.9944227, //     "F#3",
        391.995436,  //     "G3",
        415.3046976, //     "G#3",
        440,         //     "A3",
        466.1637615, //     "A#3",
        493.8833013, //     "B3",
        523.2511306, //     "C4",
        554.365262,  //     "C#4",
        587.3295358, //     "D4",
        622.2539674, //     "D#4",
        659.2551138, //     "E4",
        698.4564629, //     "F4",
        739.9888454, //     "F#4",
        783.990872,  //     "G4",
        830.6093952, //     "G#4",
        880,         //     "A4",
        932.327523,  //     "A#4",
        987.7666025, //     "B4",
        1046.502261, //     "C5",
        1108.730524, //     "C#5",
        1174.659072, //     "D5",
        1244.507935, //     "D#5",
        1318.510228, //     "E5",
        1396.912926, //     "F5",
        1479.977691, //     "F#5",
        1567.981744, //     "G5",
        1661.21879,  //     "G#5",
        1760,        //     "A5",
        1864.655046, //     "A#5",
        1975.533205, //     "B5",
        2093.004522, //     "C6",
        2217.461048, //     "C#6",
        2349.318143, //     "D6",
        2489.01587,  //     "D#6",
        2637.020455, //     "E6",
        2793.825851, //     "F6",
        2959.955382, //     "F#6",
        3135.963488, //     "G6",
        3322.437581, //     "G#6",
        3520,        //     "A6",
        3729.310092, //     "A#6",
        3951.06641,  //     "B6",
        4186.009045, //     "C7",
        4434.922096, //     "C#7",
        4698.636287, //     "D7",
        4978.03174,  //     "D#7",
        5274.040911, //     "E7",
        5587.651703, //     "F7",
        5919.910763, //     "F#7",
        6271.926976, //     "G7",
        6644.875161, //     "G#7",
        7040,        //     "A7",
        7458.620184, //     "A#7",
        7902.13282,  //     "B7",
        8372.01809,  //     "C8",
        8869.844191, //     "C#8",
        9397.272573, //     "D8",
        9956.063479, //     "D#8",
        10548.08182, //     "E8",
        11175.30341, //     "F8",
        11839.82153, //     "F#8",
        12543.85395  //     "G8"
    };
};
