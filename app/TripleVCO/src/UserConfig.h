/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once
#include <Arduino.h>

struct UserConfig
{
    char ver[15] = "svco_conf_004\0";;
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
    UserConfig()
    {
        voctTune = 0;
        oscAWave = 0;
        oscACoarse = 32;
        oscAPhaseShift = 5;
        oscAFolding = 0;
        oscBWave = 0;
        oscBCoarse = 32;
        oscBPhaseShift = 5;
        oscBFolding = 0;
        oscCWave = 0;
        oscCCoarse = 32;
        oscCPhaseShift = 5;
        oscCFolding = 0;
        rangeMode = 0;
        oscBVOct = 0;
        oscCVOct = 0;
        oscAParaCV = 0;
        oscBWaveCV = 0;
        smoothLevel = 100;
        smoothncurve = 3;
        smoothMaxFreq = 3;
    }
};
