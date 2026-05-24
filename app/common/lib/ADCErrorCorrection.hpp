/*
 * RP2040 12bit ADCErrorCorrection class
 * Copyright 2025 marksard
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
#include <hardware/adc.h>

// References:
// Integral and Differential Nonlinearity (INL/DNL)
// https://pico-adc.markomo.me/INL-DNL/

// ・RP2040 ADCのエラッタの補正(INL/DNL)
// ・デフォルト5000mV分圧→3269.9mV(3.3Vレギュ-1%ライン)回路対応
class ADCErrorCorrection
{
public:
    ADCErrorCorrection()
    {
        _lastVref = 0.0;
        _noiseFloor = 0.0;
        for (int i = 0; i < 4096; ++i)
        {
            _voctPow[i] = 0.0;
            _INL[i] = 0.0;
            _correctedAdc[i] = 0;
        }

        _adcInputRatio = 1.0;
        // デフォルト3.26989Vだと意図的に3.3Vから1%近く下げててINLへ与える影響があるため差分を補完
        // _inlRatio = (1.0 - (_adcInputMax / 3.3)) * -1;
        _inlRatio = -0.01;
    }

    ADCErrorCorrection(float adc_input_max)
        : _adcInputMax(adc_input_max), _voctInputRatio(_inputMax / adc_input_max)
    {
        ADCErrorCorrection();
    }

    void init(float vref, float noiseFloor)
    {
        generateLUT(vref, noiseFloor);
    }

    void generateLUT(float vref, float noiseFloor)
    {
        generateINL(noiseFloor);
        generate(vref);
    }

    uint16_t getADCAvg16(int8_t pin)
    {
        uint16_t raw = 0;
        pin = pin == 4 ? 4 : pin - A0;
        for (int i = 0; i < 16; ++i)
        {
            adc_select_input(pin);
            raw += adc_read();
            sleep_ms(2);
        }

        raw = raw >> 4;
        return raw;
    }

    uint16_t getADCAvg16Fast(int8_t pin)
    {
        uint16_t raw = 0;
        pin = pin == 4 ? 4 : pin - A0;
        adc_select_input(pin);
        for (int i = 0; i < 16; ++i)
        {
            raw += adc_read();
        }

        raw = raw >> 4;
        return raw;
    }

    uint16_t getADCMin16(int8_t pin)
    {
        uint16_t raw = -1;
        pin = pin == 4 ? 4 : pin - A0;
        for (int i = 0; i < 16; ++i)
        {
            adc_select_input(pin);
            uint16_t tmp = adc_read();
            raw = min(tmp, raw);
            sleep_ms(2);
        }

        return raw;
    }

    uint16_t getADCMax16(int8_t pin)
    {
        uint16_t raw = 0;
        pin = pin == 4 ? 4 : pin - A0;
        for (int i = 0; i < 16; ++i)
        {
            adc_select_input(pin);
            uint16_t tmp = adc_read();
            raw = max(tmp, raw);
            sleep_ms(2);
        }

        return raw;
    }

    uint16_t getADCTrimAvg16(int8_t pin)
    {
        int raw = 0;
        pin = pin == 4 ? 4 : pin - A0;

        int16_t minValue = 4095;
        int16_t maxValue = -1;
        int16_t avgValues[16] = {0};
        for (int i = 0; i < 16; ++i)
        {
            adc_select_input(pin);
            adc_read();
            sleep_ms(2);
            int16_t value = adc_read();
            avgValues[i] = value;
            minValue = min(minValue, value);
            maxValue = max(maxValue, value);
        }

        for (int i = 0; i < 16; ++i)
        {
            if (avgValues[i] == minValue) continue;
            if (avgValues[i] == maxValue) continue;
            raw += avgValues[i];
        }
        raw = raw / 14;

        return raw;
    }

    uint16_t correctedInputScaleAdc(int16_t adc)
    {
        return adc * _adcInputRatio;
    }

    float getADC2VRef(int16_t adc)
    {
        float vref = (_adcInputMax * 4095.0) / adc;
        return vref;
    }

    float getLastVRef()
    {
        return _lastVref;
    }

    float getLastNoiseFloor()
    {
        return _noiseFloor;
    }

    float voctPow(int16_t adc)
    {
        adc = constrain(adc, 0, 4095);
        return _voctPow[adc];
    }

    int16_t correctedAdc(int16_t adc)
    {
        adc = constrain(adc, 0, 4095);
        return _correctedAdc[adc];
    }

protected:
    // INL生成
    void generateINL(float noiseFloor)
    {
        _noiseFloor = noiseFloor;
        // DNLスパイク箇所（例：+8.9 LSB）、その累積によるINL
        const int spikes[] = {512, 1536, 2560, 3584};
        const float spikeLSBs[] = {8.9, 8.9, 7.8, 8.9};
        // 初期値として実機計測値に近似する値で初期化
        for (int i = 0; i < 4096; ++i)
        {
            // ノイズを考慮（とりあえずリニアに影響度を下げていく）
            _INL[i] = (_inlRatio * i) - (noiseFloor - ((noiseFloor / 4095) * i));
        }

        for (int i = 0; i < 4; ++i)
        {
            int spike_pos = spikes[i];
            // スパイク以降すべてに誤差を足していく（積分）
            for (int j = spike_pos - 1; j < 4096; ++j)
            {
                if (spike_pos > j)
                {
                    _INL[j] += spikeLSBs[i] / 2.0;
                }
                else
                {
                    _INL[j] += spikeLSBs[i];
                }
            }
        }
    }

    // LUT生成
    void generate(float vref)
    {
        // Serial.println("adc,_INL[],correctedAdc[],_voctPow");
        _lastVref = vref;
        _adcInputRatio = vref / _adcInputMax;
        float lsb_voltage = vref / 4095.0; // 1 LSBあたりの電圧
        for (int adc = 0; adc < 4096; ++adc)
        {
            // INLでADCコードを補正（floatに変換して）
            float correctedAdc = adc + _INL[adc];                   // LSB単位で補正
            correctedAdc = constrain(correctedAdc, 0.0, 4095.0); // 範囲を0〜4095に制限
            float voltage = correctedAdc * lsb_voltage;
            float voct = voltage * _voctInputRatio;
            _correctedAdc[adc] = correctedAdc * _adcInputRatio;
            _voctPow[adc] = pow(2, voct); // 電圧からVOCT周波数に変換して格納
            // Serial.print(adc);
            // Serial.print(",");
            // Serial.print(_INL[adc], 4);
            // Serial.print(",");
            // Serial.print(correctedAdc * _adcInputRatio, 4);
            // Serial.print(",");
            // Serial.print(_voctPow[adc], 4);
            // Serial.println();
        }
    }

protected:
    float _lastVref;
    float _noiseFloor;
    float _voctPow[4096];
    int16_t _correctedAdc[4096];
    float _INL[4096];
    // V/OCT入力電圧最大
    const float _inputMax = 5.0;
    // 0.1%精密抵抗分圧によるADCへの最大入力電圧
    const float _adcInputMax = 189000.0 / (100000.0 + 189000.0) * _inputMax;
    const float _voctInputRatio = _inputMax / _adcInputMax;
    float _inlRatio;
    float _adcInputRatio;
};
