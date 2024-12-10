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
const static char *UI_VER = "stpsq_conf_001";
struct UserConfig
{
    char ver[15];
    // step seq
    int16_t octUnder;
    int16_t octUpper;
    int16_t gateMin;
    int16_t gateMax;
    int16_t gateInitial;
    // common
    int16_t seqSyncMode;
    int16_t bpm;
    int16_t scale;
    int16_t swing;
    // euclid
    int16_t euclidPos;
    int16_t euclidSyncDiv;
    int16_t euclidOnsets;
    int16_t euclidStepSize;
    // sample and hold
    int16_t shTrigger;
    int16_t shSource;
    int16_t shIntOctMax;
    int16_t shIntSpeed;
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->octUnder = -1;
    pUserConfig->octUpper = 2;
    pUserConfig->gateMin = 2;
    pUserConfig->gateMax = 4;
    pUserConfig->gateInitial = 2;
    pUserConfig->seqSyncMode = 0;
    pUserConfig->bpm = 133;
    pUserConfig->scale = 2;
    pUserConfig->swing = 0;
    pUserConfig->euclidPos = 0;
    pUserConfig->euclidSyncDiv = 0;
    pUserConfig->euclidOnsets = 4;
    pUserConfig->euclidStepSize = 16;
    pUserConfig->shTrigger = 1;
    pUserConfig->shSource = 0;
    pUserConfig->shIntOctMax = 3;
    pUserConfig->shIntSpeed = 20;
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
