/*!
 * OscilloscopeLite class
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
#include <U8g2lib.h>

#define FRM_LFT 20
#define FRM_RGT 127
const static uint16_t FRM_SIZE = (FRM_RGT - FRM_LFT);
#define FRM_TOP 0
#define FRM_BTM 56
#define FRM_LEN 4
#define FRM_DIV 4
const static uint16_t FRM_CENTER = (FRM_TOP + ((FRM_BTM - FRM_TOP) >> 1));
const static uint16_t FRM_DIV_SPAN = (FRM_SIZE / FRM_DIV);

#define DATA_BIT 12
#define DATA_MAX_VALUE 4095
#define DATA_MAX_VALUE_F 4095.0
#define DATA_MAX_VOLT 5.0
#define DRAW_AREA_SIZE FRM_SIZE
#define HORIZONTAL_SCALE_MAX 50
// スナップショットサイズ：トリガー検出範囲をサイズに指定しているので描画サイズの2倍必要）
// リングバッファサイズ：レコード範囲（スナップショット取得時に間引きして取得するのでスナップショットサイズの倍数になる）
const static uint16_t SNAPSHOT_SIZE = DRAW_AREA_SIZE * 2;
const static uint16_t RING_BUF_DATA_SIZE = (SNAPSHOT_SIZE * HORIZONTAL_SCALE_MAX);
// スナップショットからの間引き数を大まかに設定させる
#define SNAPSHOT_INDEX_MAX 10
const static uint16_t snapshotSteps[SNAPSHOT_INDEX_MAX] = {
    1, 2, 5, 10, 20, 30, 40, 50, 100, 200};

class RingBuff
{
public:
    int16_t _buf[RING_BUF_DATA_SIZE];
    int16_t _snapShot[SNAPSHOT_SIZE];
    int16_t _writeIndex;
    bool _lock;
    RingBuff()
    {
        _lock = false;
        _writeIndex = 0;
        for (int16_t i = 0; i < RING_BUF_DATA_SIZE; ++i)
        {
            _buf[i] = 0;
        }
        for (int16_t i = 0; i < SNAPSHOT_SIZE; ++i)
        {
            _snapShot[i] = 0;
        }
    }

    void set(int16_t value)
    {
        if (_lock)
            return;
        _buf[_writeIndex] = value;
        _writeIndex++;
        if (_writeIndex >= RING_BUF_DATA_SIZE)
        {
            _writeIndex = 0;
        }
    }

    void snapShot(int16_t scale)
    {
        _lock = true;
        for (int16_t i = 0; i < SNAPSHOT_SIZE; ++i)
        {
            _snapShot[i] = readBuf(i * scale);
        }

        _lock = false;
    }

    int16_t get(int16_t index)
    {
        return _snapShot[index];
    }

protected:
    int16_t readBuf(int16_t index)
    {
        int16_t readIndex = _writeIndex + 1 + index;
        readIndex = readIndex % RING_BUF_DATA_SIZE;
        return _buf[readIndex];
    }
};

class OscilloscopeLite
{
public:
    OscilloscopeLite(U8G2 *pU8g2, uint8_t dispOffsetTop)
    {
        init(pU8g2, dispOffsetTop);
    }

    OscilloscopeLite(uint32_t sampling_freq)
    {
        // 表示窓の分割時間計算用：1000us(1ms) / (サンプリング周波数 / ウィンドウサイズ) / 分割数
        _divRatio = 1000.0 / ((float)sampling_freq / (float)FRM_SIZE) / (float)FRM_DIV;
        init(NULL, 0);
    }

    void init(U8G2 *pU8g2, uint8_t dispOffsetTop)
    {
        _pU8g2 = pU8g2;
        _horizontalScale = 0;
        _verticalScale = 0;
        _dataAve = 0;
        _rangeMax = 5;
        _rangeMin = 0;
        _triggerPoint = 10;
        _trigger = true;
        _left = 0;
        _top = dispOffsetTop;
    }

    void write(int16_t value)
    {
        _data.set(value);
    }

    void draw(bool encMode, const char *pDataName, bool isBias)
    {
        drawFrame();
        drawString(encMode, pDataName, isBias);
        if (_horizontalScale >= SNAPSHOT_INDEX_MAX)
        {
            shiftAndAppendLongTermData();
            drawData();
        }
        else
        {
            snapShotData();
            drawData();
        }
    }

    void addHorizontalScale(int8_t value)
    {
        if (value == 0)
            return;
        _verticalScale = _verticalScale + value;
        setHorizontalScale(_verticalScale);
    }

    void setHorizontalScale(int8_t value)
    {
        _horizontalScale = constrain(value, 0, SNAPSHOT_INDEX_MAX);
    }

    void addVerticalScale(int8_t value)
    {
        if (value == 0)
            return;
        _verticalScale = _verticalScale + value;
        setVerticalScale(_verticalScale);
    }

    void setVerticalScale(int8_t value)
    {
        _verticalScale = constrain(value, 0, 2);
        _rangeMin = _verticalScale;
        _rangeMax = 5 - _verticalScale;
    }

    void setTrigger(bool value)
    {
        _trigger = value;
    }

    bool getTrigger() { return _trigger; }

protected:
    void clearDataBuff()
    {
        for (uint8_t i = 0; i < SNAPSHOT_SIZE; ++i)
        {
            _dataBuff[i] = 0;
        }
    }

    void snapShotData()
    {
        int16_t dataMin = DATA_MAX_VALUE;
        int16_t dataMax = 0;
        uint32_t sum = 0;

        _data.snapShot(snapshotSteps[_horizontalScale]);
        for (uint8_t i = 0; i < SNAPSHOT_SIZE; ++i)
        {
            uint16_t tmp = _data.get(i);
            _dataBuff[i] = tmp;
            sum += tmp;
            dataMin = min(dataMin, tmp);
            dataMax = max(dataMax, tmp);
        }

        _dataAve = sum / SNAPSHOT_SIZE;

        // 垂直側表示範囲を広げてみやすく
        // _rangeMin = dataMin - 20;
        // _rangeMin = max((_rangeMin / 10) * 10, 0);
        // _rangeMax = dataMax + 20;
        // _rangeMax = min((_rangeMax / 10) * 10, DATA_MAX_VALUE);

        _triggerPoint = 0;
        if (!_trigger)
            return;
        // 0～FRM_SIZEの範囲で立ち上がりを探す
        uint32_t triggerRange = (dataMax + dataMin) >> 1;
        for (uint8_t i = 0; i < FRM_SIZE; ++i)
        {
            uint16_t tmpP1 = _dataBuff[i + 1];
            uint16_t tmp = _dataBuff[i];
            if ((tmp < triggerRange) &&
                (tmpP1 >= triggerRange))
            {
                _triggerPoint = i;
                break;
            }
        }
    }

    int16_t appendLongTermData()
    {
        static int16_t index = 0;
        _data.snapShot(1);
        for (uint8_t i = 0; i < 2; ++i)
        {
            _dataBuff[index] = _data.get((SNAPSHOT_SIZE / 2) * i);
            if (index >= DRAW_AREA_SIZE)
            {
                break;
            }

            index++;
        }

        if (index >= DRAW_AREA_SIZE)
        {
            clearDataBuff();
            index = 0;
        }

        // _rangeMax = DATA_MAX_VALUE;
        // _rangeMin = 0;
        int16_t tmp = 0;
        uint32_t sum = 0;
        for (uint8_t i = 0; i < DRAW_AREA_SIZE; ++i)
        {
            tmp = _dataBuff[i];
            sum += tmp;
        }

        _dataAve = sum / DRAW_AREA_SIZE;

        _triggerPoint = 0;

        return index;
    }

    void shiftAndAppendLongTermData()
    {
        _data.snapShot(SNAPSHOT_INDEX_MAX);
        _dataBuff[DRAW_AREA_SIZE] = _data.get(0);
        // for (uint8_t i = 0; i < 16; ++i)
        // {
        //     _dataBuff[i + DRAW_AREA_SIZE] = _data.get(i);
        // }

        // _rangeMax = DATA_MAX_VALUE;
        // _rangeMin = 0;
        int16_t tmp = 0;
        uint32_t sum = 0;
        for (uint8_t i = 0; i < DRAW_AREA_SIZE; ++i)
        {
            _dataBuff[i] = _dataBuff[i + 1];
            tmp = _dataBuff[i];
            sum += tmp;
        }

        _dataAve = sum / DRAW_AREA_SIZE;

        _triggerPoint = 0;
    }

    void drawData(int16_t drawEnd = DRAW_AREA_SIZE)
    {
        int16_t y, y2;
        int16_t rangeMin = _rangeMin * _convertCoffVolt;
        int16_t rangeMax = _rangeMax * _convertCoffVolt;

        for (int16_t x = 0; x < drawEnd; x += 2)
        {
            uint8_t bufIndex = max(x + _triggerPoint, 0);
            uint8_t bufIndexMinusOne = max(((int16_t)bufIndex - 1), 0);
            int16_t tmpM1 = _dataBuff[bufIndexMinusOne];
            int16_t tmp = _dataBuff[bufIndex];
            y = constrain(map(tmpM1, rangeMin, rangeMax, FRM_BTM - 1, FRM_TOP + 1), FRM_TOP + 1, FRM_BTM - 1);
            y2 = constrain(map(tmp, rangeMin, rangeMax, FRM_BTM - 1, FRM_TOP + 1), FRM_TOP + 1, FRM_BTM - 1);
            _pU8g2->drawLine(_left + FRM_LFT + x, _top + y, _left + FRM_LFT + x + 1, _top + y2);

            // 線を太くする（上下にも点を打つ）
            _pU8g2->drawPixel(_left + FRM_LFT + x,     _top + y + 1);
            _pU8g2->drawPixel(_left + FRM_LFT + x,     _top + y - 1);
            _pU8g2->drawPixel(_left + FRM_LFT + x + 1, _top + y2 + 1);
            _pU8g2->drawPixel(_left + FRM_LFT + x + 1, _top + y2 - 1);
        }
    }

    // 1Vあたりのピクセル数を計算（整数演算）
    int voltageToPixel(int voltage, int minVoltage, int maxVoltage)
    {
        int pixelsPerVolt = FRM_BTM / (maxVoltage - minVoltage);
        int pixelCoord = FRM_BTM - ((voltage - minVoltage) * pixelsPerVolt);
        constrain(pixelCoord, 0, FRM_BTM);
        return pixelCoord;
    }

    void drawFrame()
    {
        for (uint8_t i = _rangeMin; i < _rangeMax; ++i)
        {
            int y = voltageToPixel(i, _rangeMin, _rangeMax);
            _pU8g2->drawHLine(_left + FRM_LFT + 1, y + FRM_TOP, FRM_LEN);
        }

        int16_t rangeMin = _rangeMin * _convertCoffVolt;
        int16_t rangeMax = _rangeMax * _convertCoffVolt;
        _pU8g2->drawHLine(_left + FRM_RGT - FRM_LEN, _top + FRM_TOP, FRM_LEN);
        _pU8g2->drawHLine(_left + FRM_RGT - FRM_LEN, _top + FRM_BTM - 1, FRM_LEN);
        int16_t ave = map(_dataAve, rangeMin, rangeMax, FRM_BTM - 1, FRM_TOP + 1);
        _pU8g2->drawHLine(_left + FRM_LFT - FRM_LEN, _top + ave, FRM_LEN);
        for (uint8_t x = FRM_LFT; x <= FRM_RGT; x += 8)
        {
            _pU8g2->drawHLine(_left + x, _top + FRM_CENTER, 2);
        }

        for (uint8_t x = 0; x < FRM_DIV + 1; ++x)
        {
            _pU8g2->drawVLine(_left + FRM_LFT + (FRM_DIV_SPAN * x), _top + FRM_TOP, FRM_LEN);
            _pU8g2->drawVLine(_left + FRM_RGT, _top + FRM_BTM - FRM_LEN + 1, FRM_LEN);
        }
    }

    void drawString(bool encMode, const char *pDataName, bool isBias)
    {
        static char chrBuff[32] = {0};
        float tmp = 0.0;
        _pU8g2->setFont(u8g2_font_5x8_tf);

        if (encMode)
        {
            _pU8g2->drawHLine(_left, _top + FRM_TOP + 9, _left + FRM_LFT - FRM_LEN);
            _pU8g2->drawHLine(_left, _top + FRM_BTM - 2, _left + FRM_LFT - FRM_LEN);
        }

        tmp = _rangeMax - (isBias ? 2.5 : 0);
        sprintf(chrBuff, "%3.1f", tmp);
        _pU8g2->drawStr(_left + 1, _top + FRM_TOP, chrBuff);

        sprintf(chrBuff, "A+B", tmp);
        _pU8g2->drawStr(_left, _top + FRM_CENTER - 8, chrBuff);
        sprintf(chrBuff, "bias", tmp);
        _pU8g2->drawStr(_left, _top + FRM_CENTER, chrBuff);

        tmp = _rangeMin - (isBias ? 2.5 : 0);
        sprintf(chrBuff, "%3.1f", tmp);
        _pU8g2->drawStr(_left + 1, _top + FRM_BTM - 10, chrBuff);

        if (_horizontalScale >= SNAPSHOT_INDEX_MAX)
        {
            sprintf(chrBuff, "A:%s B:%s pot:   slow", pDataName, _trigger ? "TRG" : "---");
        }
        else
        {
            sprintf(chrBuff, "A:%s B:%s pot:%5.1fms", pDataName, _trigger ? "TRG" : "---", _divRatio * snapshotSteps[_horizontalScale]);
        }

        _pU8g2->drawStr(_left, _top + FRM_BTM, chrBuff);
    }

protected:
    float _convertCoffVolt = DATA_MAX_VALUE_F / DATA_MAX_VOLT;
    U8G2 *_pU8g2;
    int16_t _horizontalScale;
    int16_t _verticalScale;
    int16_t _dataAve;
    int16_t _rangeMax;
    int16_t _rangeMin;
    int16_t _triggerPoint;
    uint8_t _left;
    uint8_t _top;
    RingBuff _data;
    float _divRatio;
    bool _trigger;
    int16_t _dataBuff[SNAPSHOT_SIZE];
};
