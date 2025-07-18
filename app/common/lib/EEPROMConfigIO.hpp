/*!
 * EEPROMConfigIO
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
// #include <EEPROM.h>

struct BaseTemplateConfig
{
    char ver[15];
};

template <typename vs = BaseTemplateConfig>
class EEPROMConfigIO
{
public:
    EEPROMConfigIO(int startAddress = 0)
        : _startAddress(startAddress),
          Config()
    {
    }

    static void initEEPROM(size_t size = 1024)
    {
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
        EEPROM.begin(size);
#else
        EEPROM.begin();
#endif
    }

    void loadUserConfig()
    {
        BaseTemplateConfig verCheck;
        EEPROM.get<BaseTemplateConfig>(_startAddress, verCheck);

        // バージョン文字列部分だけ読んで違えば読まない
        if (strcmp(verCheck.ver, Config.ver))
        {
            return;
        }

        EEPROM.get<vs>(_startAddress, Config);
    }

    void saveUserConfig()
    {
        EEPROM.put<vs>(_startAddress, Config);
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
        EEPROM.commit();
#endif
    }

    // 読み書き自由
    vs Config;

private:
    int _startAddress;
};
