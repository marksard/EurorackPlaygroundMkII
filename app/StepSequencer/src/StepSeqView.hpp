/*!
 * StepSeqView
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

class StepSeqView
{
public:
    StepSeqView(U8G2 *pU8g2, uint8_t origin_x, uint8_t origin_y)
    {
        _pU8g2 = pU8g2;
        _origin_x = origin_x;
        _origin_y = origin_y;
    }

    void dispSteps(uint8_t keyStart, uint8_t keyEnd, uint8_t gateStart, uint8_t gateEnd, uint8_t *pOcts, uint8_t *pKeys, uint8_t *pGates, uint8_t *pAccs, int8_t gateLenAdder, int8_t octaveAddr)
    {
        static char disp_item[StepSeqModel::Gate::Max] = {'-', 'T', 'S', 'H', 'L', 'G'};
        for (uint8_t i = 0; i < 16; ++i)
        {
            uint8_t x = pos2X(i);
            uint8_t y = pos2Y(i);
            uint8_t keyInv = i >= keyStart && i <= keyEnd ? 1 : 0;
            uint8_t gateInv = i >= gateStart && i <= gateEnd ? 1 : 0;
            uint8_t gate = constrain(pGates[i] + gateLenAdder, 0, StepSeqModel::Gate::G);
            dispStepUnit(x, y,
                         keyInv,
                         gateInv,
                         constrain(pOcts[i] + octaveAddr, 0, 5),
                         pKeys[i],
                         disp_item[gate],
                         pAccs[i] == 1 ? '*' : ' ');
        }
    }

    void dispSettingPos(uint8_t value)
    {
        uint8_t x = pos2X(value);
        uint8_t y = pos2Y(value);
        _pU8g2->drawBox(x, y + 20, 16, 3);
    }

    void dispGatePos(uint8_t value)
    {
        uint8_t x = pos2X(value);
        uint8_t y = pos2Y(value);
        _pU8g2->drawBox(x, y + 10, 16, 10);
    }

    void dispKeyPos(uint8_t value)
    {
        uint8_t x = pos2X(value);
        uint8_t y = pos2Y(value);
        _pU8g2->drawBox(x, y, 16, 10);
    }

private:
    U8G2 *_pU8g2;
    uint8_t _origin_x;
    uint8_t _origin_y;

    void dispStepUnit(uint8_t x, uint8_t y, uint8_t keyInv, uint8_t gateInv, uint8_t oct, uint8_t key, uint8_t gate, uint8_t acc)
    {
        static char disp_buf[2] = {0};
        if (keyInv)
        {
            disp_buf[0] = '0' + oct;
            disp_buf[1] = '\0';
            _pU8g2->drawStr(x + 2, y + 1, disp_buf);
            disp_buf[0] = '0' + key;
            disp_buf[1] = '\0';
            _pU8g2->drawStr(x + 10, y + 1, disp_buf);
        }

        if (gateInv)
        {
            disp_buf[0] = acc;
            disp_buf[1] = '\0';
            _pU8g2->drawStr(x + 2, y + 10, disp_buf);
            disp_buf[0] = gate;
            disp_buf[1] = '\0';
            _pU8g2->drawStr(x + 10, y + 10, disp_buf);
        }

        // _pU8g2->drawFrame(x, y, 16, 10);
        // _pU8g2->drawFrame(x, y + 10, 16, 10);
        _pU8g2->drawFrame(x, y, 16, 20);
    }

    uint8_t pos2X(uint8_t value) { return ((value % 8) * 16) + _origin_x; }
    uint8_t pos2Y(uint8_t value) { return ((value >> 3) * 24) + _origin_y; }
};
