/*!
 * RotaryEncoder class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#define RE_DELTA_THRESHOLD_COUNT 5
static const uint16_t _thresholds[RE_DELTA_THRESHOLD_COUNT] = {6000, 12000, 24000, 32000, 64000};
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
    void init(int pin1, int pin2, bool lastValueHold = false)
    {
        _pin1 = pin1;
        _pin2 = pin2;

        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);

        _timePrev = 0;
        _timeCurrent = 0;
        _timePrevAcc = 0;
        _timeCurrentAcc = 0;
        _value = 0;
        _lastValueHold = lastValueHold;

        getDirection(); // 空読みして値をいれておく
    }

    /// @brief 動作方向を取得
    /// @return 0:none plus:clockwise minus:counter clockwise
    int8_t getDirection(bool accelerate = false)
    {
        // オーバーフローを考慮した時間差の計算
        if ((micros() - _timePrev) < 500)
        {
            if (_lastValueHold == false)
                _value = 0;

            return _value;
        }
        _timePrev = _timeCurrent;
        _timeCurrent = micros();

        byte value1, value2;
        getPinValue(&value1, &value2);
        byte state = value1 | (value2 << 1);

        // 状態履歴を更新
        _stateHistory[_historyIndex] = state;
        _historyIndex = (_historyIndex + 1) % _histCount;

        // 状態履歴がすべて同じ場合のみ処理を進める
        bool stable = true;
        for (byte i = 1; i < _histCount; ++i)
        {
            if (_stateHistory[i] != _stateHistory[0])
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
        _index &= 0xf;
        byte direction = 0;
        switch (_index)
        {
        case 0xd:
            _timePrevAcc = _timeCurrentAcc;
            _timeCurrentAcc = _timeCurrent;
            _index = 0;
            direction = accelerate ? getDelta() : 1;
            // バックラッシュ対策
            if (_value >= 0)
            {
                _value = direction;
            }
            break;
        case 0x7:
            _timePrevAcc = _timeCurrentAcc;
            _timeCurrentAcc = _timeCurrent;
            _index = 0;
            direction = accelerate ? getDelta() * -1 : -1;
            // バックラッシュ対策
            if (_value <= 0)
            {
                _value = direction;
            }
            break;
        default:
            // 加速度検出ありのときは入力区間を計測したいので未入力時はなにもしない
            if (accelerate == false)
            {
                _timePrevAcc = _timeCurrentAcc;
                _timeCurrentAcc = _timeCurrent;
            }
            if (_lastValueHold == false)
            {
                _value = 0;
            }
            break;
        }

        return _value;
    }

    ulong lastRotationTime()
    {
        return _timeCurrentAcc - _timePrevAcc;
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
        // _lastValueHold true時、getValueで使用したら値を破棄する
        if (_lastValueHold == true)
            _value = 0;
        return result;
    }

protected:
    byte _pin1;
    byte _pin2;
    byte _index;
    ulong _timePrev;
    ulong _timeCurrent;
    ulong _timePrevAcc;
    ulong _timeCurrentAcc;
    byte _value;
    bool _lastValueHold;

    constexpr static byte _histCount = 4;
    byte _stateHistory[_histCount] = {0}; // 状態履歴を保持
    byte _historyIndex = 0;     // 履歴のインデックス

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
