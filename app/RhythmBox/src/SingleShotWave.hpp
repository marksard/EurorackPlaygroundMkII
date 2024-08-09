/*!
 * Oscillator class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

// pwm reso 2048, bias 1023
const uint16_t decayCurveSize = 1024; 
const uint16_t decayCurve[1024] = {
1024,1016,1008,1000,992,984,977,969,961,954,947,939,932,925,917,910,
903,896,889,882,875,869,862,855,848,842,835,829,822,816,810,803,
797,791,785,779,772,766,760,755,749,743,737,731,726,720,714,709,
703,698,692,687,682,676,671,666,661,655,650,645,640,635,630,625,
621,616,611,606,601,597,592,588,583,578,574,569,565,561,556,552,
548,543,539,535,531,527,523,518,514,510,506,502,499,495,491,487,
483,479,476,472,468,465,461,457,454,450,447,443,440,436,433,430,
426,423,420,416,413,410,407,404,401,397,394,391,388,385,382,379,
376,373,370,367,365,362,359,356,353,351,348,345,342,340,337,335,
332,329,327,324,322,319,317,314,312,309,307,305,302,300,298,295,
293,291,288,286,284,282,279,277,275,273,271,269,267,265,262,260,
258,256,254,252,250,248,247,245,243,241,239,237,235,233,232,230,
228,226,224,223,221,219,218,216,214,212,211,209,208,206,204,203,
201,200,198,196,195,193,192,190,189,187,186,185,183,182,180,179,
177,176,175,173,172,171,169,168,167,165,164,163,162,160,159,158,
157,155,154,153,152,151,149,148,147,146,145,144,142,141,140,139,
138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,
122,121,120,119,118,117,116,115,114,113,113,112,111,110,109,108,
107,107,106,105,104,103,102,102,101,100,99,99,98,97,96,95,
95,94,93,93,92,91,90,90,89,88,88,87,86,86,85,84,
84,83,82,82,81,80,80,79,78,78,77,77,76,75,75,74,
74,73,73,72,71,71,70,70,69,69,68,68,67,67,66,65,
65,64,64,63,63,62,62,61,61,61,60,60,59,59,58,58,
57,57,56,56,55,55,55,54,54,53,53,53,52,52,51,51,
50,50,50,49,49,49,48,48,47,47,47,46,46,46,45,45,
44,44,44,43,43,43,42,42,42,41,41,41,40,40,40,40,
39,39,39,38,38,38,37,37,37,37,36,36,36,35,35,35,
35,34,34,34,33,33,33,33,32,32,32,32,31,31,31,31,
30,30,30,30,29,29,29,29,29,28,28,28,28,27,27,27,
27,27,26,26,26,26,26,25,25,25,25,25,24,24,24,24,
24,23,23,23,23,23,22,22,22,22,22,22,21,21,21,21,
21,21,20,20,20,20,20,20,19,19,19,19,19,19,19,18,
18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,
16,16,16,16,16,15,15,15,15,15,15,15,15,14,14,14,
14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,12,
12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,
11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,
10,9,9,9,9,9,9,9,9,9,9,9,9,9,8,8,
8,8,8,8,8,8,8,8,8,8,8,8,8,8,7,7,
7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,6,
6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
6,6,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,
4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,
3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

template <typename vs = int16_t>
class SingleShotWave
{
public:
    SingleShotWave(const vs wave[], uint32_t waveLength)
    {
        init(wave, waveLength);
    }

    void init(const vs wave[], uint32_t waveLength)
    {
        _pWave = wave;
        _waveLength = waveLength;
        _start = false;
        _waveIndex = 0.0;
        _pitch = 1.0;
        _volume = 1.0;
        _bias = 1023;
        _decayIndex = 0.0;
        _decayValue = 0;
        _lastEdge = 0;
        // _lastValue = _bias;
        _mute = false;
    }

    uint16_t updateWave()
    {
        if (!_start)
            return _bias;

        int16_t value = _pWave[(int)_waveIndex];
        _waveIndex += _pitch;
        if (_waveIndex >= _waveLength)
        {
            _start = false;
            _waveIndex = 0.0;
            _decayIndex = 0.0;
            // _lastValue = _bias;
            return _bias;
        }

        if (_mute)
            return _bias;

        value = (uint16_t)((((int32_t)value * _decayValue) >> 10) + _bias);
        // simplest linear interpolation
        // value = (_lastValue + value) >> 1;
        // _lastValue = value;
        return value;
    }

    void play(int8_t edge)
    {
        if (_lastEdge == 0 && edge != 0)
        {
            _start = true;
            _waveIndex = 0.0;
            _decayIndex = 0.0;
        }
        _lastEdge = edge;
    }

    void updateDecay(float value)
    {
        float decay = (1.0 - value);
        _decayValue = decayCurve[(int)_decayIndex] * _volume;
        _decayIndex += decay;
        if (_decayIndex >= decayCurveSize)
        {
            _start = false;
        }
    }

    void setSpeed(float value) { _pitch = value; }

    void setBias(vs value) { _bias = value; }
    void setMute(bool value) { _mute = value; }
    void setVolume(float value) { _volume = value; }

private:
    const vs *_pWave;
    uint32_t _waveLength;
    bool _start;
    bool _mute;
    float _waveIndex;
    float _pitch;
    float _volume;
    vs _bias;
    // vs _lastValue;
    float _decayIndex;
    int16_t _decayValue;
    int8_t _lastEdge;
};
