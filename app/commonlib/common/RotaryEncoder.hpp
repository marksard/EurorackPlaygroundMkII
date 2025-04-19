/*!
 * RotaryEncoder class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#define RE_DELTA_THRESHOLD_COUNT 5
static const uint16_t _thresholds[RE_DELTA_THRESHOLD_COUNT] = {1000, 5000, 10000, 20000, 40000};
static const byte _deltas[RE_DELTA_THRESHOLD_COUNT] = {12, 6, 3, 2, 1};

class RotaryEncoder
{
public:
    RotaryEncoder() {}
    RotaryEncoder(int pin1, int pin2)
    {
        init(pin1, pin2);
    }

    /// @brief ピン設定
    /// @param pin1
    /// @param pin2
    void init(int pin1, int pin2, bool holdMode = false)
    {
        _pin1 = pin1;
        _pin2 = pin2;

        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);

        // _timePrev = micros();
        _timePrev = 0;
        _timeCurrent = _timePrev;
        _holdMode = holdMode;
        _value = 0;

        getDirection(); // 空読みして値をいれておく
    }

    /// @brief 動作方向を取得
    /// @return 0:none plus:clockwise minus:counter clockwise
    int8_t getDirection(bool accelerate = false)
    {
        static byte stateHistory[4] = {0}; // 状態履歴を保持
        static byte historyIndex = 0;     // 履歴のインデックス

        // オーバーフローを考慮した時間差の計算
        if ((micros() - _timePrev) < 500)
        {
            if (!_holdMode)
                _value = 0;

            return _value;
        }

        byte value1, value2;
        getPinValue(&value1, &value2);
        byte state = value1 | (value2 << 1);

        // 状態履歴を更新
        stateHistory[historyIndex] = state;
        historyIndex = (historyIndex + 1) % 4;

        // 状態履歴がすべて同じ場合のみ処理を進める
        bool stable = true;
        for (byte i = 1; i < 4; ++i)
        {
            if (stateHistory[i] != stateHistory[0])
            {
                stable = false;
                break;
            }
        }

        if (!stable)
        {
            return _value; // 状態が安定していない場合は値を更新しない
        }

        _index = (_index << 2) + (state & 3);
        _index &= 15;

        switch (_index)
        {
        case 0xd:
            _timePrev = _timeCurrent;
            _timeCurrent = micros();
            _index = 0;
            _value = accelerate ? getDelta() : 1;
            break;
        case 0x7:
            _timePrev = _timeCurrent;
            _timeCurrent = micros();
            _index = 0;
            _value = accelerate ? getDelta() * -1 : -1;
            break;
        default:
            if (!accelerate)
            {
                _timePrev = _timeCurrent;
                _timeCurrent = micros();
            }
            if (!_holdMode)
            {
                _value = 0;
            }
            break;
        }

        return _value;
    }

    ulong lastRotationTime()
    {
        return _timeCurrent - _timePrev;
    }

    byte getDelta()
    {
        for (byte i = 0; i < RE_DELTA_THRESHOLD_COUNT; ++i)
        {
            if (lastRotationTime() < _thresholds[i])
            {
                return _deltas[i];
            }
        }

        return 1;
    }

    byte getValue()
    {
        byte result = _value;
        if (_holdMode)
            _value = 0;
        return result;
    }

protected:
    byte _pin1;
    byte _pin2;
    byte _index;
    ulong _timePrev;
    ulong _timeCurrent;
    byte _value;
    bool _holdMode;

    virtual void getPinValue(byte *pValue1, byte *pValue2)
    {
        *pValue1 = readPin(_pin1);
        *pValue2 = readPin(_pin2);
    }

    inline byte readPin(int pin)
    {
        // return digitalRead(pin);
        return gpio_get(pin);
    }
};
