/*!
 * CheapRhythm88 class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "../../commonlib/common/MiniOsc.hpp"
#include "../../commonlib/common/StateVariableFilter.hpp"

class CheapRhythm88
{
public:
    enum Type
    {
        DRUM_KICK,
        DRUM_SNARE,
        DRUM_HIHAT,
        DRUM_TOM,
        DRUM_TYPE_MAX
    };

public:
    CheapRhythm88() {}

    void init(float sampleRate, int16_t resolution)
    {
        // Envelope演算上signed変換が必要
        bool signedOut = true;
        _osc.init(sampleRate, signedOut);
        _osc.setWave(MiniOsc::TRI);
        _osc.setLevel(11);
        _osc.start();
        _osc2.init(sampleRate, signedOut);
        _osc2.setWave(MiniOsc::NOISE);
        _osc2.setLevel(11);
        _osc2.start();
        _start = false;
        _tvaEnvIndex = 0.0;
        _tvaEnvValue = 0;
        _pitchEnvIndex = 0.0;
        _pitchEnvValue = 0;
        _resolution = resolution;
        _resoM1 = resolution - 1;
        _resoHalf = resolution >> 1;
        svfHiHat.init(resolution, signedOut);
        svfHiHat.setParameter(0.9, 0.5);
        svfSnare.init(resolution, signedOut);
        svfSnare.setParameter(0.05, 1.0);
    }

    inline float calcFrequencyModulation(int16_t base, int16_t adder, float mod)
    {
        return (base + adder) * mod;
    }

    inline float calcFrequencyAdder(int16_t base, int16_t adder, float mod)
    {
        return base + adder + (mod * 0.097);
    }

    void update(Type type, int16_t pitchMod, float decayMod)
    {
        _type = type;
        switch (type)
        {
        case DRUM_KICK:
            updateDecay(decayMod, 1);
            updatePitchDecay(0.1);
            _osc.setFrequency(calcFrequencyAdder(kickFreq, pitchMod >> 2, _cheapMode ? 0 : (_pitchEnvValue)));
            _osc.setWave(MiniOsc::TRI);
            _osc.setLevel(11);
            _osc2.setLevel(6);
            break;
        case DRUM_SNARE:
            updateDecay(decayMod, 0);
            updatePitchDecay(0.5);
            _osc.setFrequency(calcFrequencyModulation(300, pitchMod, _cheapMode ? 0 : (_pitchEnvValue / 4096.0)));
            _osc.setWave(MiniOsc::TRI);
            _osc.setLevel(10);
            _osc2.setLevel(10);
            break;
        case DRUM_HIHAT:
            updateDecay(decayMod, 0);
            _osc.setFrequency(4000 + pitchMod);
            _osc.setWave(MiniOsc::SAW);
            _osc.setLevel(10);
            _osc2.setLevel(9);
            break;
        case DRUM_TOM:
            updateDecay(decayMod, 1);
            updatePitchDecay(tomPitchDecay);
            _osc.setFrequency(calcFrequencyAdder(40, pitchMod, _cheapMode ? 0 : _pitchEnvValue / 1.5));
            _osc.setWave(MiniOsc::TRI);
            _osc.setLevel(11);
            _osc2.setLevel(7);
            break;
        default:
            break;
        }
    }

    uint16_t process()
    {
        if (!_start || _tvaEnvIndex >= decayCurveSize)
            return _resoHalf;

        // クリップさせて歪ませる
        int16_t value = constrain(_osc.getWaveValue() + (int16_t)_osc2.getWaveValue(), -(_resoHalf), (_resoHalf) - 1);
        if (!_cheapMode && _type == Type::DRUM_HIHAT)
        {
            svfHiHat.process(value);
            value = svfHiHat.bandPass();
        }
        if (!_cheapMode && _type == Type::DRUM_SNARE)
        {
            svfSnare.process(value);
            value = svfSnare.highPass();
        }
        value = ((value * _tvaEnvValue) >> 10) + (_resoHalf);
        value = constrain(value, 0, _resoM1);
        return value;
    }

    bool isStart() const { return _start; }
    void start()
    {
        _start = true;
        _tvaEnvIndex = 0.0;
        _pitchEnvIndex = 0.0;
    }

    Type getType() { return _type; }

    void setCheapMode(bool mode) { _cheapMode = mode; }
    bool getCheapMode() { return _cheapMode; }

    int8_t kickFreq = 12;
    int8_t kickPatamA = 2;
    void addKickParamA(int8_t value)
    {
        if (value == 0)
            return;
        kickPatamA = constrain(kickPatamA + value, 0, 5);
        kickFreq = 8 + (kickPatamA * 2);
    }

    int8_t snarePatamA = 5;
    void addSnareParamA(int8_t value)
    {
        if (value == 0)
            return;
        snarePatamA = constrain(snarePatamA + value, 0, 5);
        float cutoff = snarePatamA * 0.02 + 0.001;
        svfSnare.setCutoffFrequency(cutoff);
    }

    int8_t hihatPatamA = 5;
    void addHihatParamA(int8_t value)
    {
        if (value == 0)
            return;
        hihatPatamA = constrain(hihatPatamA + value, 0, 5);
        float resolution = hihatPatamA * 0.10 + 0.04;
        svfHiHat.setResonance(resolution);
    }

    float tomPitchDecay = 0.2;
    int8_t tomPatamA = 1;
    void addTomParamA(int8_t value)
    {
        if (value == 0)
            return;
        tomPatamA = constrain(tomPatamA + value, 0, 5);
        tomPitchDecay = tomPatamA * 0.2;
    }

private:
    void updateDecay(float value, int type)
    {
        if (_tvaEnvIndex >= decayCurveSize)
        {
            _start = false;
            return;
        }
        // 範囲を0.1～0.95に制限
        const float adcDecayRatio1 = 0.75 / ADC_RESO;
        const float adcDecayRatio2 = 0.85 / ADC_RESO;
        value = (type == 0 ? adcDecayRatio1 : adcDecayRatio2) * value + 0.1;
        float decay = (1.0 - value);
        _tvaEnvValue = decayCurve[(int)_tvaEnvIndex];
        _tvaEnvIndex += decay;
    }

    void updatePitchDecay(float value)
    {
        if (_pitchEnvIndex >= decayCurveSize)
            return;
        float decay = (1.0 - value);
        _pitchEnvValue = decayCurve[(int)_pitchEnvIndex];
        _pitchEnvIndex += decay;
    }

private:
    Type _type;
    bool _start;
    MiniOsc _osc;
    MiniOsc _osc2;
    float _tvaEnvIndex;
    int16_t _tvaEnvValue;
    float _pitchEnvIndex;
    int16_t _pitchEnvValue;
    StateVariableFilter svfHiHat;
    StateVariableFilter svfSnare;
    bool _cheapMode;
    int16_t _resolution;
    int16_t _resoM1;
    int16_t _resoHalf;

private:
    // pwm resolution 2048, bias 1023
    const uint16_t decayCurveSize = 1024;
    const uint16_t decayCurve[1024] = {
        1024, 1016, 1008, 1000, 992, 984, 977, 969, 961, 954, 947, 939, 932, 925, 917, 910,
        903, 896, 889, 882, 875, 869, 862, 855, 848, 842, 835, 829, 822, 816, 810, 803,
        797, 791, 785, 779, 772, 766, 760, 755, 749, 743, 737, 731, 726, 720, 714, 709,
        703, 698, 692, 687, 682, 676, 671, 666, 661, 655, 650, 645, 640, 635, 630, 625,
        621, 616, 611, 606, 601, 597, 592, 588, 583, 578, 574, 569, 565, 561, 556, 552,
        548, 543, 539, 535, 531, 527, 523, 518, 514, 510, 506, 502, 499, 495, 491, 487,
        483, 479, 476, 472, 468, 465, 461, 457, 454, 450, 447, 443, 440, 436, 433, 430,
        426, 423, 420, 416, 413, 410, 407, 404, 401, 397, 394, 391, 388, 385, 382, 379,
        376, 373, 370, 367, 365, 362, 359, 356, 353, 351, 348, 345, 342, 340, 337, 335,
        332, 329, 327, 324, 322, 319, 317, 314, 312, 309, 307, 305, 302, 300, 298, 295,
        293, 291, 288, 286, 284, 282, 279, 277, 275, 273, 271, 269, 267, 265, 262, 260,
        258, 256, 254, 252, 250, 248, 247, 245, 243, 241, 239, 237, 235, 233, 232, 230,
        228, 226, 224, 223, 221, 219, 218, 216, 214, 212, 211, 209, 208, 206, 204, 203,
        201, 200, 198, 196, 195, 193, 192, 190, 189, 187, 186, 185, 183, 182, 180, 179,
        177, 176, 175, 173, 172, 171, 169, 168, 167, 165, 164, 163, 162, 160, 159, 158,
        157, 155, 154, 153, 152, 151, 149, 148, 147, 146, 145, 144, 142, 141, 140, 139,
        138, 137, 136, 135, 134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123,
        122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 113, 112, 111, 110, 109, 108,
        107, 107, 106, 105, 104, 103, 102, 102, 101, 100, 99, 99, 98, 97, 96, 95,
        95, 94, 93, 93, 92, 91, 90, 90, 89, 88, 88, 87, 86, 86, 85, 84,
        84, 83, 82, 82, 81, 80, 80, 79, 78, 78, 77, 77, 76, 75, 75, 74,
        74, 73, 73, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 65,
        65, 64, 64, 63, 63, 62, 62, 61, 61, 61, 60, 60, 59, 59, 58, 58,
        57, 57, 56, 56, 55, 55, 55, 54, 54, 53, 53, 53, 52, 52, 51, 51,
        50, 50, 50, 49, 49, 49, 48, 48, 47, 47, 47, 46, 46, 46, 45, 45,
        44, 44, 44, 43, 43, 43, 42, 42, 42, 41, 41, 41, 40, 40, 40, 40,
        39, 39, 39, 38, 38, 38, 37, 37, 37, 37, 36, 36, 36, 35, 35, 35,
        35, 34, 34, 34, 33, 33, 33, 33, 32, 32, 32, 32, 31, 31, 31, 31,
        30, 30, 30, 30, 29, 29, 29, 29, 29, 28, 28, 28, 28, 27, 27, 27,
        27, 27, 26, 26, 26, 26, 26, 25, 25, 25, 25, 25, 24, 24, 24, 24,
        24, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21,
        21, 21, 20, 20, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 18,
        18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16,
        16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14,
        14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12,
        12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11,
        11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8,
        8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7,
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
        4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};
