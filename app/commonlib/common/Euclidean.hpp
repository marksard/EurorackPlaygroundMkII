/*
 * Euclidean class
 * Copyright 2024 marksard
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <Arduino.h>

class Euclidean
{
public:
    const static uint8_t EUCLID_MAX_STEPS = 16;

public:
    Euclidean()
    {
        _recursive_max = EUCLID_MAX_STEPS;
        _currentStep = 0;
        _onsets = 0;
        _stepSize = 0;
        _lockSteps = false;
        // memset(_steps, 0, sizeof(_steps));
        setArrayAll(_steps, 0, EUCLID_MAX_STEPS);
    }

    uint16_t getNext()
    {
        if (_lockSteps)
            return _steps[_currentStep];

        uint16_t result = _steps[_currentStep];
        // Serial.print(result);
        // Serial.print(",");
        // Serial.print(_currentStep);
        // Serial.println();
        _currentStep++;
        if (_currentStep >= _stepSize)
        {
            _currentStep = 0;
        }

        return result;
    }

    void resetCurrent() { _currentStep = 0; }
    uint8_t getCurrent() { return _currentStep; }
    uint8_t getOnsets() { return _onsets; }
    uint8_t getStepSize() { return _stepSize; }
    uint8_t getCurrentTrigger() { return _steps[_currentStep]; }
    const uint16_t *getSteps() { return _steps; }

    bool updateEncWhenGenerate(int8_t enc0, int8_t enc1)
    {
        uint8_t onsets = constrain(_onsets + enc0, 0, _stepSize);
        uint8_t stepSize = constrain(_stepSize + enc1, _onsets, EUCLID_MAX_STEPS);
        if (_onsets == onsets && _stepSize == stepSize)
        {
            return false;
        }
        _onsets = onsets;
        _stepSize = stepSize;
        generate();
        return true;
    }

    // ユークリッドリズムを生成
    bool generate(uint16_t onsets, uint16_t stepSize)
    {
        if (_stepSize == stepSize && _onsets == onsets)
        {
            return false;
        }
        _onsets = onsets;
        _stepSize = stepSize;
        generate();
        return true;
    }

    void generate()
    {
        _lockSteps = true;

        uint16_t front[EUCLID_MAX_STEPS] = {0};
        uint16_t back[EUCLID_MAX_STEPS] = {0};

        setArrayAll(front, 1, EUCLID_MAX_STEPS);
        setArrayAll(back, 0, EUCLID_MAX_STEPS);
        setArrayAll(_steps, 0, EUCLID_MAX_STEPS);
        _recursive_max = EUCLID_MAX_STEPS;

        uint16_t result_len = _stepSize;
        if (_onsets > 0)
        {
            generateEuclideanRecursive(front, back, _onsets, _stepSize - _onsets, _steps, &result_len);
        }

        uint16_t res = concatBits(_steps, result_len);
        bit2ArrayValues(res, _steps, _stepSize);
        _lockSteps = false;

        // for (int8_t i = 0; i < _stepSize; ++i)
        // {
        //     Serial.print(_steps[i]);
        //     Serial.print(",");
        // }
        // Serial.println();
    }

protected:
    // 立っている最上位ビットを見つける
    uint16_t foundHighBitIndex(uint16_t value)
    {
        for (int8_t i = EUCLID_MAX_STEPS; i > 0; --i)
        {
            if ((value & (0x1 << i)) != 0)
            {
                return i;
            }
        }
        return 0;
    }

    // memset代わり
    void setArrayAll(uint16_t *dstData, uint16_t value, uint16_t len)
    {
        for (int8_t i = 0; i < len; ++i)
        {
            dstData[i] = value;
        }
    }

    // bit連結
    uint16_t concatBits(uint16_t *bit, uint16_t len)
    {
        uint16_t result = 0;
        for (int8_t i = 0; i < len; ++i)
        {
            uint16_t pos = 0;
            pos = foundHighBitIndex(bit[i]);
            result = (result << (pos + 1)) | bit[i];
        }
        return result;
    }

    // bitをarrayに分離
    void bit2ArrayValues(uint16_t binary, uint16_t array[], uint16_t size)
    {
        for (int8_t i = size - 1; i >= 0; --i)
        {
            array[i] = binary & 1;
            binary >>= 1;
        }
    }

    // 再起でユークリディアンな組み合わせにしてく
    void generateEuclideanRecursive(uint16_t *front, uint16_t *back, uint16_t front_len, uint16_t back_len, uint16_t *result, uint16_t *result_len)
    {
        _recursive_max--;
        if (_recursive_max < 0)
        {
            return;
        }

        // Serial.print(back_len);
        // Serial.print(",");
        // Serial.println(front_len);
        if ((back_len == 1 && back[0]) || back_len == 0)
        {
            *result_len = front_len + back_len;
            memcpy(result, front, front_len * sizeof(uint16_t));
            memcpy(result + front_len, back, back_len * sizeof(uint16_t));
            return;
        }

        uint16_t new_front_len = 0;
        uint16_t new_front[EUCLID_MAX_STEPS] = {0};
        while (front_len > 0 && back_len > 0)
        {
            uint16_t back_val = back[--back_len];
            uint16_t front_val = front[--front_len];
            uint16_t front_sft = foundHighBitIndex(back_val) + 1;
            new_front[new_front_len++] = (front_val << front_sft) + back_val;
        }

        uint16_t new_back_len = 0;
        uint16_t new_back[EUCLID_MAX_STEPS] = {0};
        for (int8_t i = 0; i < front_len; ++i)
        {
            new_back[new_back_len++] = front[i];
        }
        for (int8_t i = 0; i < back_len; ++i)
        {
            new_back[new_back_len++] = back[i];
        }

        generateEuclideanRecursive(new_front, new_back, new_front_len, new_back_len, result, result_len);
    }

protected:
    bool _lockSteps;
    int8_t _recursive_max;
    uint8_t _currentStep;
    uint8_t _onsets;
    uint8_t _stepSize;
    uint16_t _steps[EUCLID_MAX_STEPS];
};
