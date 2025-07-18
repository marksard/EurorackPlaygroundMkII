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

// 以下の複数の目的を持つ
// ・RP2040 ADCのエラッタの補正(INL/DNL)
// ・温度センサからVrefを推測し、V/OCT電圧から周波数に変換するLUTを生成
// 5000mVを3269.9mV(3.3Vレギュレータ誤差1%最低ラインを想定)に分圧し
// てADCへ入力している回路を使うことが前提
class ADCErrorCorrection
{
public:
    ADCErrorCorrection()
    {
        _lastVref = 0.0f;
        _noiseFloor = 0.0f;
        for (int i = 0; i < 4096; ++i)
        {
            _voctPow[i] = 0.0f;
            _INL[i] = 0.0f;
            _correctedAdc[i] = 0;
            // _voct[i] = 0.0f;
        }
    }

    void init(float vref = 0.0f, float noiseFloor = 0.0f)
    {
        generateINL(noiseFloor);
        // adc_init();ほかで読んでなければ呼ぶ
        if (vref == 0.0f)
        {
            vref = getSpeculationVRef(true);
        }
        else {
            _lastVref = vref;
        }

        generate(vref);
    }

    void generateLUT(float vref = 0.0f)
    {
        if (vref == 0.0f)
        {
            vref = getSpeculationVRef(false);
        }
        else {
            _lastVref = vref;
        }

        generate(vref);
    }

    // INL生成
    void generateINL(float noiseFloor = 0.0f)
    {
        _noiseFloor = noiseFloor;
        // DNLスパイク箇所（例：+8.9 LSB）、その累積によるINL
        const int spikes[] = {512, 1536, 2560, 3584};
        // 初期値として実機計測値に近似する値で初期化
        for (int i = 0; i < 4096; ++i)
        {
            // 0.01下がりはRP2040マニュアルのDNLの下がり具合から
            // zero互換機は20程度持ち上がっているので計測でノイズがある場合は一律下げる
            if (noiseFloor > 0.0f)
            {
                // 最後は除いて初期値をいれてく
                if (i < 4095)
                {
                    _INL[i] = (-0.01f * i) - noiseFloor;
                }
            }
            else
            {
                if (i < 4095)
                {
                    _INL[i] = (-0.01f * i);
                }
            }
        }

        for (int i = 0; i < 4; ++i)
        {
            int spike_pos = spikes[i];
            // スパイク以降すべてに誤差を足していく（積分）
            for (int j = spike_pos - 2; j < 4096; ++j)
            {
                // 実機計測の感じからspike1個手前から8.9f、2つ手前は4.45fとした
                if (spike_pos - 1 > j)
                {
                    _INL[j] += 2.5f; // 4.45 LSBを累積
                }
                else {
                    _INL[j] += 8.9f; // 8.9 LSBを累積
                }
            }
        }
    }

    float getSpeculationVRef(bool waiting = false)
    {
        adc_set_temp_sensor_enabled(true);
        if (waiting)
            sleep_ms(100); // センサの安定化待ち
        uint16_t raw = getADCAvg16(4); // ADC4温度センサ
        adc_set_temp_sensor_enabled(false);
        // 適当な係数で3V3電圧を推定
        float vref = 0.695f * 4095 / raw;
        // float vref = 0.6961f * 4095 / raw;
        // float vref = 0.6988f * 4095 / raw;
        _lastVref = vref;
        return vref;
    }

    uint16_t getADCAvg16(int8_t pin)
    {
        uint16_t raw = 0;
        pin = pin == 4 ? 4 : pin - A0;
        for (int i = 0; i < 16; ++i)
        {
            adc_select_input(pin);
            raw += adc_read();
            sleep_ms(1);
        }
        raw = raw >> 4;
        return raw;
    }

    float getADC2VRef(int16_t adc)
    {
        float vref = (_adc_input_max * 4095.0f) / adc;
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

    // float voct(int16_t adc)
    // {
    //     adc = constrain(adc, 0, 4095);
    //     return _voct[adc];
    // }

protected:
    // LUT生成
    void generate(float vref)
    {
        float lsb_voltage = vref / 4095.0f; // 1 LSBあたりの電圧
        for (int adc = 0; adc < 4096; ++adc)
        {
            // INLでADCコードを補正（floatに変換して）
            float correctedAdc = adc + _INL[adc];                   // LSB単位で補正
            correctedAdc = constrain(correctedAdc, 0.0f, 4095.0f); // 範囲を0〜4095に制限
            float voltage = correctedAdc * lsb_voltage;
            float voct = voltage * _scale;
            _correctedAdc[adc] = correctedAdc;
            // _voct[adc] = voct;
            _voctPow[adc] = pow(2, voct); // 電圧からVOCT周波数に変換して格納
        }
    }

protected:
    float _lastVref;
    float _noiseFloor;
    float _voctPow[4096];
    int16_t _correctedAdc[4096];
    // float _voct[4096];
    float _INL[4096];
    // V/OCT入力電圧最大
    const float _input_max = 5.0f;
    // 0.1%精密抵抗分圧によるADCへの最大入力電圧
    const float _adc_input_max = 189000.0f / (100000.0f + 189000.0f) * _input_max; 
    const float _scale = _input_max / _adc_input_max;
};
