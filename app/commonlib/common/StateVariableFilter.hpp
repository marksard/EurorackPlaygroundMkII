/*!
 * StateVariableFilter class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

class StateVariableFilter
{
public:
    StateVariableFilter(int16_t resolution = 2048)
        : lp(0.0f), bp(0.0f), hp(0.0f),
          _cutoff(1.0f),
          _resonance(0.5f),
          _resolution(resolution),
          _resoM1(resolution - 1),
          _resoHalf(resolution >> 1)
    {
    }

    void process(uint16_t input)
    {
        float in = (float)input;

        hp = in - lp - _resonance * bp;
        bp += _cutoff * hp;
        lp += _cutoff * bp;
    }

    int bandPass() const
    {
        float out = constrain((int)bp + _resoHalf, 0, _resoM1);
        return (int)out;
    }

    int highPass() const
    {
        float out = constrain((int)(hp + _resoHalf), 0, _resoM1);
        return (int)out;
    }

    int lowPass() const
    {
        float out = constrain((int)lp, 0, _resoM1);
        return (int)out;
    }

    void setCutoffFrequency(float cutoff) { _cutoff = cutoff; }
    void setResonance(float resonance) { _resonance = resonance; }
    void setParameter(float cutoff, float resonance)
    {
        setCutoffFrequency(cutoff);
        setResonance(resonance);
    }

    void reset()
    {
        lp = 0.0f;
        bp = 0.0f;
        hp = 0.0f;
    }

private:
    int16_t _resolution; // 解像度（PWM_RESO）
    int16_t _resoM1;
    int16_t _resoHalf;
    float lp;         // ローパス出力
    float bp;         // バンドパス出力
    float hp;         // ハイパス出力（内部に保持）
    float _cutoff;    // カットオフ（0.05〜0.3くらい）
    float _resonance; // 共振（Q）
};