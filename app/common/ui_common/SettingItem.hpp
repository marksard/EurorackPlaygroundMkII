/*!
 * SettingItem
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

/// 設定項目
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

    // 外部で用意する表示バッファに設定項目名を入れる
    void updateDispStrings(char *disp_buf)
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

// 設定メニューセクション
template <typename vs = int16_t>
struct MenuSection
{
    char title[12];
    SettingItem<vs> *pItemList;
    uint8_t itemCount;
};
typedef MenuSection<int16_t> MenuSection16;
typedef MenuSection<uint16_t> MenuSection16u;
typedef MenuSection<float> MenuSectionF;

// 設定メニューコントロール
template <typename vs = int16_t>
class MenuControl
{
public:
    MenuControl(MenuSection<vs> *pSection, uint8_t sectionCount)
    {
        _pSection = pSection;
        _sectionCount = sectionCount;

        _itemIndex = 0;
        _sectionIndex = 0;
        _underIndex = false;
        _overIndex = false;
    }

    /// @brief メニュー選択
    /// @param value エンコーダーのデルタ値
    /// @return 更新ありなし
    /// セクション内なら前後アイテムに、セクションからはみ出たら前後セクションに移動
    bool select(int8_t value)
    {
        int itemNextIndex = _itemIndex + value;
        int sectionNextIndex = 0;
        _underIndex = false;
        _overIndex = false;
        if (itemNextIndex < 0)
        {
            sectionNextIndex = constrain(_sectionIndex - 1, 0, _sectionCount - 1);
            if (_sectionIndex != sectionNextIndex)
            {
                itemNextIndex = _pSection[sectionNextIndex].itemCount - 1;
            }
            else
            {
                _underIndex = true;
                itemNextIndex = _itemIndex;
            }
        }
        else if (itemNextIndex > _pSection[_sectionIndex].itemCount - 1)
        {
            sectionNextIndex = constrain(_sectionIndex + 1, 0, _sectionCount - 1);
            if (_sectionIndex != sectionNextIndex)
            {
                itemNextIndex = 0;
            }
            else
            {
                _overIndex = true;
                itemNextIndex = _itemIndex;
            }
        }
        else {
            sectionNextIndex = _sectionIndex;
        }

        bool result = _itemIndex != itemNextIndex ? true : false;
        _itemIndex = itemNextIndex;
        result |= _sectionIndex != sectionNextIndex ? true : false;
        _sectionIndex = sectionNextIndex;
        return result;
    }

    bool isUnder() { return (_underIndex); }
    bool isOver() { return (_overIndex); }

    bool addValue2CurrentSetting(float value)
    {
        return _pSection[_sectionIndex].pItemList[_itemIndex].add(value);
    }

    void draw(U8G2 *pU8g2, bool encMode, bool titleHalfDisp = false, bool itemHalfDisp = false)
    {
        drawSetting(pU8g2, _pSection[_sectionIndex].title, 
                            _pSection[_sectionIndex].pItemList, _itemIndex, 
                            _pSection[_sectionIndex].itemCount, encMode,
                            titleHalfDisp, itemHalfDisp);
    }

private:
    void drawSetting(U8G2 *pU8g2, const char *title, SettingItem<vs> *pSettingItems, 
        uint8_t itemIndex, uint8_t itemCount, uint8_t selector,
        bool titleHalfDisp, bool itemHalfDisp)
    {
        int titleStartX = titleHalfDisp ? 64 : 0;
        int itemStartX = itemHalfDisp ? 64 : 0;
        static uint8_t drawStrYPos[3] = { 16, 32, 48 };
        static char disp_buf[3][32] = {0};
        pU8g2->setFont(u8g2_font_profont22_tf);
        sprintf(disp_buf[0], title);
        pU8g2->drawStr(titleStartX, 0, disp_buf[0]);

        pU8g2->setFont(u8g2_font_7x14B_tf);
        static uint8_t menuSlider = 0;
        menuSlider = map(itemIndex, 0, itemCount, 0, 3);
        // Serial.print(itemIndex);
        // Serial.print(",");
        // Serial.print(menuSlider);
        // Serial.println();
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (i >= itemCount)
            {
                break;
            }

            bool sel = menuSlider == i ? true : false;
            pSettingItems[itemIndex - menuSlider + i].updateDispStrings(disp_buf[i]);
            pU8g2->drawStr(itemStartX + 8, drawStrYPos[i], disp_buf[i]);
            if (sel)
            {
                if (selector == 1)
                {
                    pU8g2->drawHLine(itemStartX + 7, 28 + (16 * i), 120 - itemStartX);
                }
                else
                {
                    pU8g2->drawBox(itemStartX, 17 + (16 * i), 6, 10);
                }
            }
        }


        if (itemIndex < 1)
        {
            pU8g2->drawHLine(itemStartX, 16, 127 - itemStartX);
        }
        if (itemIndex == itemCount - 1)
        {
            pU8g2->drawHLine(itemStartX, 63, 127 - itemStartX);
        }
    }

private:
    int _itemIndex;
    int _sectionIndex;
    MenuSection<vs> *_pSection;
    uint8_t _sectionCount;
    bool _underIndex;
    bool _overIndex;
};
typedef MenuControl<int16_t> MenuControl16;
typedef MenuControl<uint16_t> MenuControl16u;
typedef MenuControl<float> MenuControlF;
