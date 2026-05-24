/*!
 * MiniOsc class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "RandomFast.hpp"

#define WAVE_LENGTH_BIT 12
#define WAVE_LENGTH 4096
#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class MiniOsc
{
public:
    enum Wave
    {
        SQU,
        SAW,
        TRI,
        NOISE,
        MAX = NOISE,
    };
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    MiniOsc()
    {
    }

    void init(float clock, uint8_t heightBit = 11, bool signedOut = false)
    {
        setWaveHeightBit(heightBit);
        _phaseAccum = 0;
        _tuningWordM = 0;
        _wave = Wave::SQU;
        _widthHalf = WAVE_LENGTH >> 1;
        _interruptClock = clock;
        _attenuate = 0;
        _frequency = 0;
        _start = true;
        _bpm = 0;
        _bpmReso = 4;
        _rnd.randomSeed(OSC_WAVE_BIT32 - 1);
        _signedOut = signedOut;
    }

    int16_t getWaveValue()
    {
        if (_start == false) return 0;

        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        uint32_t indexHeight = (index >> _widthHeightDiffBit);
        int16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < _widthHalf ? _heightM1 : 0;
            break;
        case Wave::SAW:
            value = indexHeight;
            break;
        case Wave::TRI:
            value = index < _widthHalf ? (indexHeight << 1) : ((_heightM1 - indexHeight) << 1);
            break;
        case Wave::NOISE:
            value = _rnd.getRandom16(_height);
            break;
        default:
            value = 0;
            break;
        }

        if (_signedOut)
        {
            value = value - (_height >> 1);
        }

        return value >> _attenuate;
    }

    void setFrequency(float frequency)
    {
        if (_frequency == frequency)
            return;
        // _bpm = (uint32_t)(frequency * 60.0) / _bpmReso;
        _frequency = frequency;
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * ((float)frequency / _interruptClock);
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

    void setLevel(uint8_t level)
    {
        _attenuate = constrain(_heightBit - level, 0, _heightBit);
    }

    static float getFreqFromNoteIndex(uint8_t value)
    {
        if (value > 127) return noteFreq[127];
        return noteFreq[value];
    }

    void setFreqFromBPM(byte bpm, byte bpmReso)
    {
        if (_bpm == bpm && _bpmReso == bpmReso) return;
        _bpm = bpm;
        _bpmReso = bpmReso;
        uint16_t pulsesPerMinute = bpm * bpmReso;
        float frequency = pulsesPerMinute / 60.0;
        _duration = (1.0 / frequency) * 1000000;
        setFrequency(frequency);
    }

    // setFreqFromBPM設定時のみ使用可
    byte getBPM() { return _bpm; }
    byte getBPMReso() { return _bpmReso; }
    int getDurationMills() { return _duration / 1000; }

    void start()
    {
        _phaseAccum = 0;
        _start = true;
    }

    void stop()
    {
        _phaseAccum = 0;
        _start = false;
    }

    bool isStart()
    {
        return _start;
    }

private:
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    Wave _wave;
    uint16_t _widthHalf;
    uint16_t _heightM1;
    uint8_t _attenuate;
    float _interruptClock;
    float _frequency;
    bool _start;
    byte _bpm;
    byte _bpmReso;
    ulong _duration;
    RandomFast _rnd;
    bool _signedOut;
    uint8_t _heightBit;
    uint16_t _height;
    uint8_t _widthHeightDiffBit;

    void setWaveHeightBit(uint8_t heightBit)
    {
        _heightBit = heightBit;
        _height = 1 << _heightBit;
        _widthHeightDiffBit = WAVE_LENGTH_BIT - _heightBit;
        _heightM1 = _height - 1;
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
