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
const static char *UI_VER = "cvco_conf_001\0";
struct UserConfig
{
    char ver[15];
    int16_t voctTune;
    int16_t oscAWave;
    int16_t oscACoarse;
    int16_t oscAPhaseShift;
    int16_t oscAFolding;
    int16_t oscAParaCV;
    int16_t arpMode;
    int16_t rootMinus;
    int16_t seventhMinus;
    int16_t voctHold;
    int16_t boostLevel;
    int16_t quantizeCV;
    int16_t quantizeScale;
    int16_t quantizeOct;
    int16_t quantizeHold;
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
    pUserConfig->oscAParaCV = 0;
    pUserConfig->arpMode = 0;
    pUserConfig->rootMinus = 1;
    pUserConfig->seventhMinus = 0;
    pUserConfig->voctHold = 0;
    pUserConfig->boostLevel = 0;
    pUserConfig->quantizeCV = 1;
    pUserConfig->quantizeScale = 5;
    pUserConfig->quantizeOct = 3;
    pUserConfig->quantizeHold = 1;
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
