/*!
 * SettingItem
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

template <typename vs = int16_t>
class SettingItem
{
public:
    SettingItem(vs min, vs max, vs step, vs *pAttachValue, const char *name, const char valueNames[][5], int valueNamesCount)
        : _pName(name)
    {
        _min = min;
        _max = max;
        _step = step;
        _ValueNames = valueNames;
        _valueNamesCount = valueNamesCount;
        _value = 0;
        if (pAttachValue == NULL)
        {
            _pAttachValue = &_value;
        }
        else
        {
            _pAttachValue = pAttachValue;
        }
    }

    bool set(vs value)
    {
        vs temp = constrain(value, _min, _max);
        bool result = temp != *_pAttachValue;
        *_pAttachValue = temp;
        return result;
    }

    bool add(vs value)
    {
        vs temp = constrain(*_pAttachValue + (value * _step), _min, _max);
        ;
        bool result = temp != *_pAttachValue;
        *_pAttachValue = temp;
        return result;
    }

    vs get() { return *_pAttachValue; }

    void getDisp(char *disp_buf)
    {
        if (_valueNamesCount > 0)
        {
            int value = (int)constrain(*_pAttachValue, 0, _valueNamesCount - 1);
            sprintf(disp_buf, _pName, _ValueNames[value]);
        }
        else
        {
            sprintf(disp_buf, _pName, *_pAttachValue);
        }
    }

private:
    vs _value;
    vs _min;
    vs _max;
    vs _step;
    const char *_pName;
    const char (*_ValueNames)[5];
    int _valueNamesCount;
    vs *_pAttachValue;
};
typedef SettingItem<int16_t> SettingItem16;
typedef SettingItem<uint16_t> SettingItem16u;
typedef SettingItem<float> SettingItemF;

template <typename vs = int16_t>
void drawSetting(U8G2 *pU8g2, const char *title, SettingItem<vs> *pSettingItems, 
    uint8_t menuIndex, uint8_t menuMax, uint8_t selector = -1)
{
    static char disp_buf[3][32] = {0};
    pU8g2->setFont(u8g2_font_profont22_tf);
    sprintf(disp_buf[0], title);
    pU8g2->drawStr(0, 0, disp_buf[0]);

    pU8g2->setFont(u8g2_font_7x14B_tf);
    static uint8_t menuSlider = 0;
    menuSlider = map(menuIndex, 0, menuMax, 0, 3);
    // Serial.print(menuIndex);
    // Serial.print(",");
    // Serial.print(menuSlider);
    // Serial.println();
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (i >= menuMax)
            break;
        bool sel = menuSlider == i ? true : false;
        pSettingItems[menuIndex - menuSlider + i].getDisp(disp_buf[i]);
        if (sel)
        {
            if (selector == 1)
            {
                pU8g2->drawHLine(7, 28 + (16 * i), 120);
            }
            else
            {
                pU8g2->drawBox(0, 17 + (16 * i), 6, 10);
            }
        }
    }

    pU8g2->drawStr(8, 16, disp_buf[0]);
    pU8g2->drawStr(8, 32, disp_buf[1]);
    pU8g2->drawStr(8, 48, disp_buf[2]);

    if (menuIndex < 1)
    {
        pU8g2->drawHLine(0, 16, 127);
    }
    if (menuIndex == menuMax - 1)
    {
        pU8g2->drawHLine(0, 63, 127);
    }
}
