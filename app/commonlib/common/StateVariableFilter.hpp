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
    StateVariableFilter(int16_t resolution = 2048, bool signedOut = false)
    {
        init(resolution, signedOut);
    }

    inline void init(int16_t resolution = 2048, bool signedOut = false)
    {
        _lp = 0.0f;
        _bp = 0.0f;
        _hp = 0.0f;
        _cutoff = 1.0f;
        _resonance = 0.5f;
        _resolution = resolution;
        _resoM1 = resolution - 1;
        _resoHalf = resolution >> 1;
        _signedOut = signedOut;
    }

    inline void process(uint16_t input)
    {
        float in = (float)input;

        _hp = in - _lp - _resonance * _bp;
        _bp += _cutoff * _hp;
        _lp += _cutoff * _bp;
    }

    inline int bandPass() const
    {
        return gainRestriction(_bp);
    }

    inline int highPass() const
    {
        return gainRestriction(_hp);
    }

    inline int lowPass() const
    {
        return gainRestriction(_lp);
    }

    inline void setCutoffFrequency(float cutoff) { _cutoff = cutoff; }
    inline void setResonance(float resonance) { _resonance = resonance; }
    inline void setParameter(float cutoff, float resonance)
    {
        setCutoffFrequency(cutoff);
        setResonance(resonance);
    }

    inline void reset()
    {
        _lp = 0.0f;
        _bp = 0.0f;
        _hp = 0.0f;
    }

private:
    inline int gainRestriction(float value) const
    {
        if (_signedOut)
        {
            return constrain((int)value, -_resoHalf, _resoHalf - 1);
        }

        return constrain((int)value + _resoHalf, 0, _resoM1);
    }

private:
    int16_t _resolution; // 解像度（PWM_RESO）
    int16_t _resoM1;
    int16_t _resoHalf;
    float _lp;        // ローパス出力
    float _bp;        // バンドパス出力
    float _hp;        // ハイパス出力（内部に保持）
    float _cutoff;    // カットオフ（0.05〜0.3くらい）
    float _resonance; // 共振（Q）
    bool _signedOut;
};