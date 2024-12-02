/*
 * EuclideanDisp class
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

#define MAX_EUCLIDIAN_SIZE 16

class Point
{
public:
    Point()
    {
        x = 0;
        y = 0;
    }

public:
    int8_t x;
    int8_t y;
};

class EuclideanDisp
{
public:
    EuclideanDisp()
    {
    }

    void init(uint8_t centerX, uint8_t centerY, uint8_t radius)
    {
        _circleCenterX = centerX;
        _circleCenterY = centerY;
        _dotRadius = radius;
        _circleRadius = _dotRadius;
        _polygonRadius = _circleRadius - 2;
    }

    void generateCircle(int stepSize)
    {
        for (int i = 0; i <= stepSize; ++i)
        {
            float angle = i * (2.0 * PI / stepSize);
            _dotPos[i].x = _circleCenterX + _dotRadius * sin(angle);
            _dotPos[i].y = _circleCenterY - _dotRadius * cos(angle);
            _polygonPos[i].x = _circleCenterX + _polygonRadius * sin(angle);
            _polygonPos[i].y = _circleCenterY - _polygonRadius * cos(angle);
        }
    }

    void drawCircle(U8G2 *_pU8g2, int stepSize, int pos, const uint16_t *triggers)
    {
        // char disp_buf[33] = {0};
        // _pU8g2->clearBuffer();
        // _pU8g2->drawCircle(_circleCenterX, _circleCenterY, _circleRadius);

        int lastTriggerIndex = 0;
        for (int i = 0; i <= stepSize; ++i)
        {
            int x = _dotPos[i].x;
            int y = _dotPos[i].y;
            if (i == pos)
            {
                if (triggers[i] == 1)
                {
                    _pU8g2->drawDisc(x, y, 3);
                }
                else
                {
                    _pU8g2->drawDisc(x, y, 2);
                }
            }
            else
            {
                if (triggers[i] == 1)
                {
                    _pU8g2->drawDisc(x, y, 1);
                }
                else
                {
                    _pU8g2->drawPixel(x, y);
                }
            }

            if (i > 0)
            {
                if (stepSize == i || triggers[i] == 1)
                {
                    int x = _polygonPos[i].x;
                    int y = _polygonPos[i].y;
                    int prev_x = _polygonPos[lastTriggerIndex].x;
                    int prev_y = _polygonPos[lastTriggerIndex].y;
                    _pU8g2->drawLine(prev_x, prev_y, x, y);
                }
            }

            if (triggers[i] == 1)
            {
                lastTriggerIndex = i;
            }
            // sprintf(disp_buf, "%02d, %02d", x, y);
            // _pU8g2->drawStr(0, 48, disp_buf);
            // _pU8g2->sendBuffer();
            // delay(100);
        }
    }

protected:
    uint8_t _dotRadius;
    uint8_t _circleRadius;
    uint8_t _circleCenterX;
    uint8_t _circleCenterY;
    uint8_t _polygonRadius;
    Point _polygonPos[MAX_EUCLIDIAN_SIZE + 1];
    Point _dotPos[MAX_EUCLIDIAN_SIZE + 1];
};
