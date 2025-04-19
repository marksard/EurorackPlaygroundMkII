/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "PatternSeq.hpp"

void initEEPROM()
{
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.begin(1024);
#else
    EEPROM.begin();
#endif
}

// 設定値系
const static char *UI_VER = "rytsq_conf_005";
struct UserConfig
{
    char ver[15];
    // setting values
    float pattern[SEQUENCER_TOTAL];
    float pitches[SEQUENCER_TOTAL];
    float decays[SEQUENCER_TOTAL];
    float volumes[SEQUENCER_TOTAL];
    float triggers[SEQUENCER_TOTAL];
    float pans[SEQUENCER_TOTAL];
    float mutes[SEQUENCER_TOTAL];
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->pattern[0] = 3;
    pUserConfig->pattern[1] = 16;
    pUserConfig->pattern[2] = 28;
    pUserConfig->pattern[3] = 29;
    pUserConfig->pattern[4] = 25;
    pUserConfig->pattern[5] = 32;
    for (int i = 0; i < SEQUENCER_TOTAL; ++i)
    {
        pUserConfig->pitches[i] = 1.0;
        pUserConfig->decays[i] = 1.0;
        pUserConfig->triggers[i] = 0;
        pUserConfig->mutes[i] = 0;
    }
    pUserConfig->volumes[0] = 0.7;
    pUserConfig->volumes[1] = 1.0;
    pUserConfig->volumes[2] = 1.0;
    pUserConfig->volumes[3] = 0.8;
    pUserConfig->volumes[4] = 1.0;
    pUserConfig->volumes[5] = 1.0;
    pUserConfig->pans[0] = 4;
    pUserConfig->pans[1] = 0;
    pUserConfig->pans[2] = 3;
    pUserConfig->pans[3] = 1;
    pUserConfig->pans[4] = 2;
    pUserConfig->pans[5] = 2;
}

void loadUserConfig(UserConfig *pUserConfig)
{
    EEPROM.get<UserConfig>(startUserConfigAddress, *pUserConfig);
    if (strcmp(pUserConfig->ver, UI_VER))
    {
        initUserConfig(pUserConfig);
    }
}

void saveUserConfig(UserConfig *pUserConfig)
{
    EEPROM.put<UserConfig>(startUserConfigAddress, *pUserConfig);
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.commit();
#endif
}
