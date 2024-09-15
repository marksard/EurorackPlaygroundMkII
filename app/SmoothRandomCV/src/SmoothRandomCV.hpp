/*!
 * SmoothRandomCV class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "../../commonlib/common/RandomFast.hpp"

class SmoothRandomCV
{
public:
    SmoothRandomCV(uint16_t adcReso)
    {
        _lastFreq = 0.0;
        _lastLevel = 0.0;
        _lastMillis = 0;
        _holdFreq = 0.0;
        _holdLevel = 0.0;
        _holdMillis = 0;

        _curve = 3.0;
        _maxLevel = 100;
        _maxFreq = 5.0;
        _maxFreqRatio = 0.0;
        _adcReso = adcReso;
        _adcReso2 = (float)adcReso / 100.0;
    }

    void setMaxLevel(uint8_t value) { _maxLevel = value; }
    void setMaxFreq(float value)
    {
        if (_maxFreq == value) return;
        _maxFreq = value;
        _maxFreqRatio = ((float)_adcReso / (float)_maxFreq);
    }
    void setCurve(float value) { _curve = value; }

    float getFreq() { return _lastFreq; }
    float getLevel() { return _lastLevel; }

    // isTrigger有効ならtriggerに従う。それ以外は内部で100-1000ms間で自動更新
    // 更新タイミングで次の目標levelとfreqをランダムで決め、updateコールごとに
    // 簡易ローパスフィルタによってその目標値に近づくようにしている
    // curveはフィルターの急峻さ
    // 出力は周波数値と12bitアナログ値です
    void update(bool trigger, bool isTrigger = true)
    {
        ulong tm = millis();
        bool on = false;
        if (isTrigger)
        {
            if (trigger) on = true;
        }
        else
        {
            if (tm - _lastMillis > _holdMillis) on = true;
        }
        if (on)
        {
            _holdMillis = _rnd.getRandom16(100, 1000);
            _holdLevel = _rnd.getRandom16(0, _adcReso2 * _maxLevel);
            _holdFreq = _holdLevel / _maxFreqRatio;
            _lastMillis = tm;
        }
        float curveRight = _curve / 10000.0;
        float curveLeft = 1.0 - curveRight;
        _lastFreq = (_lastFreq * curveLeft) + (_holdFreq * curveRight);
        _lastLevel = (_lastLevel * curveLeft) + (_holdLevel * curveRight);
    }

private:
    RandomFast _rnd;

    float _lastFreq;
    float _lastLevel;
    ulong _lastMillis;
    float _holdFreq;
    float _holdLevel;
    ulong _holdMillis;

    float _curve;
    uint8_t _maxLevel;
    float _maxFreq;
    float _maxFreqRatio;
    uint16_t _adcReso;
    float _adcReso2;
};
