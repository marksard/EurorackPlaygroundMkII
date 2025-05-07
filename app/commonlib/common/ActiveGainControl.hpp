/*!
 * ActiveGainControl class
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>

/// @brief ActiveGainControl class
/// @details 音量を一定に保つためのクラス。音量が大きくなったらGainを下げる。音量が小さくなったらGainを上げる。
class ActiveGainControl
{
public:
    ActiveGainControl() : _reso(0), _bias(0), _gainMax(0), _divs(0), _peak(0), _gain(0), _level(0) {}

    /// @brief 初期化
    /// @param reso resoはPWMの分解能。12bitなら4096
    /// @param waveCount 波形の数。4つの波形を同時に再生する場合は4
    /// @param gainMax Gainの最大値。0.0-1.0の範囲で指定
    void init(int16_t reso, int16_t waveCount, float gainMax)
    {
        // 浮動小数点演算を避けるため、gain計算は10bitシフトして整数演算にする
        _reso = reso;
        _bias = reso >> 1;
        _gainMax = gainMax * (1 << 10);
        _divs = _bias << 10;

        _peak = waveCount * (_bias - 1);
        _gain = _gainMax;
        _level = 0;
    }

    inline void setCurrentLevel(int16_t levelL, int16_t levelR = 0)
    {
        _level = max(abs(levelL), abs(levelR));
    }

    inline int16_t getProcessedLevel(int16_t level)
    {
        return constrain(((level * _gain) >> 10) + _bias, 0, _reso - 1);
    }

    inline void update(uint8_t decaySpeed = 1)
    {
        _peak = (_level > _peak) ? _level : _peak - decaySpeed; // レベルが大きくなったら上書き。それ以外は徐々に減衰
        _gain = min(_divs / _peak, _gainMax);
    }

    void print()
    {
        Serial.print("gain:");
        Serial.print(_gain);
        Serial.print(" max:");
        Serial.print(_gainMax);
        Serial.print(" peak:");
        Serial.print(_peak);
        Serial.println();
    }

private:
    int16_t _reso;
    int16_t _bias;
    int32_t _gainMax;
    int32_t _divs;
    int16_t _peak;
    int32_t _gain;
    int16_t _level;
};
