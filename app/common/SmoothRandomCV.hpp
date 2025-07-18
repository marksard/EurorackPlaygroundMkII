/*!
 * SmoothRandomCV class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "lib/RandomFast.hpp"

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
        if (_maxFreq == value)
            return;
        _maxFreq = value;
        _maxFreqRatio = _maxFreq / 100.0;
    }
    void setCurve(float value) { _curve = value; }

    float getFreq() { return _lastFreq; }
    float getLevel() { return _lastLevel; }

    // isTrigger有効ならtriggerに従う。それ以外は内部タイマーで自動更新
    // 更新タイミングで次の目標levelとfreqをランダムで決め、updateコールごとに
    // 簡易ローパスフィルタによってその目標値に近づくようにしている
    // curveはフィルターの急峻さ
    // 出力は周波数と12bitアナログ値
    bool update(bool trigger, bool isTrigger = true)
    {
        bool on = isTrigger ? trigger : ready();
        if (on)
        {
            int16_t rndLevel = _rnd.getRandom16(0, _maxLevel);
            int16_t rndFreq = _rnd.getRandom16(0, 100);
            // levelとfreqをランダムに変化させる
            _holdLevel = _adcReso2 * rndLevel;
            _holdFreq = _maxFreqRatio * rndFreq;
        }
        float curveRight = _curve * 0.0001;
        float curveLeft = 1.0 - curveRight;
        _lastFreq = (_lastFreq * curveLeft) + (_holdFreq * curveRight);
        _lastLevel = (_lastLevel * curveLeft) + (_holdLevel * curveRight);

        return on;
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

    bool ready()
    {
        ulong tm = millis();
        bool result = (tm - _lastMillis) > _holdMillis ? true : false;
        if (result)
        {
            _holdMillis = _rnd.getRandom16(100, 1000);
            _lastMillis = tm;
        }

        return result;
    }
};
