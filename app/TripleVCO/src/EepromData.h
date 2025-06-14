/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once

#include <Arduino.h>
#include <EEPROM.h>

void initEEPROM()
{
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.begin(1024);
#else
    EEPROM.begin();
#endif
}

// 設定値系
const static char *UI_VER = "svco_conf_004\0";
struct UserConfig
{
    char ver[15];
    int16_t voctTune;
    int16_t oscAWave;
    float oscACoarse;
    int16_t oscAPhaseShift;
    int16_t oscAFolding;
    int16_t oscBWave;
    float oscBCoarse;
    int16_t oscBPhaseShift;
    int16_t oscBFolding;
    int16_t oscCWave;
    float oscCCoarse;
    int16_t oscCPhaseShift;
    int16_t oscCFolding;
    int16_t rangeMode;
    int16_t oscBVOct;
    int16_t oscCVOct;
    int16_t oscAParaCV;
    int16_t oscBWaveCV;
    int16_t smoothLevel;
    int16_t smoothncurve;
    int16_t smoothMaxFreq;
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->voctTune = 0;
    pUserConfig->oscAWave = 0;
    pUserConfig->oscACoarse = 32;
    pUserConfig->oscAPhaseShift = 5;
    pUserConfig->oscAFolding = 0;
    pUserConfig->oscBWave = 0;
    pUserConfig->oscBCoarse = 32;
    pUserConfig->oscBPhaseShift = 5;
    pUserConfig->oscBFolding = 0;
    pUserConfig->oscCWave = 0;
    pUserConfig->oscCCoarse = 32;
    pUserConfig->oscCPhaseShift = 5;
    pUserConfig->oscCFolding = 0;
    pUserConfig->rangeMode = 0;
    pUserConfig->oscBVOct = 0;
    pUserConfig->oscCVOct = 0;
    pUserConfig->oscAParaCV = 0;
    pUserConfig->oscBWaveCV = 0;
    pUserConfig->smoothLevel = 100;
    pUserConfig->smoothncurve = 3;
    pUserConfig->smoothMaxFreq = 3;
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
