/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once

#include <Arduino.h>

struct UserConfig
{
    char ver[15] = "stpsq_conf_001";
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
    int16_t euclidOnsetsSource;
    // sample and hold
    int16_t shTrigger;
    int16_t shSource;
    int16_t shIntOctMax;
    int16_t shIntSpeed;
    UserConfig()
    {
        octUnder = -1;
        octUpper = 2;
        gateMin = 2;
        gateMax = 4;
        gateInitial = 2;
        seqSyncMode = 0;
        bpm = 133;
        scale = 2;
        swing = 0;
        euclidPos = 0;
        euclidSyncDiv = 0;
        euclidOnsets = 4;
        euclidStepSize = 16;
        euclidOnsetsSource = 0;
        shTrigger = 1;
        shSource = 0;
        shIntOctMax = 3;
        shIntSpeed = 20;
    }
};
