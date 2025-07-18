/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once

#include <Arduino.h>

// 設定値系
struct UserConfig
{
    char ver[15] = "cvco_conf_002\0";
    int16_t voctTune;
    int16_t oscAWave;
    int16_t oscACoarseIndex;
    int16_t oscAPhaseShift;
    int16_t oscAFolding;
    int16_t oscAParaCV;
    int16_t arpMode;
    int16_t rootMinus;
    int16_t seventhMinus;
    int16_t voctHold;
    int16_t scale;
    int16_t quantizeCV;
    int16_t quantizeScale;
    int16_t quantizeOct;
    int16_t quantizeHold;
    UserConfig()
    {
        voctTune = -33;
        oscAWave = 0;
        oscACoarseIndex = 32;
        oscAPhaseShift = 5;
        oscAFolding = 0;
        oscAParaCV = 0;
        arpMode = 0;
        rootMinus = 1;
        seventhMinus = 0;
        voctHold = 0;
        scale = 0;
        quantizeCV = 1;
        quantizeScale = 5;
        quantizeOct = 3;
        quantizeHold = 1;
    }
};
