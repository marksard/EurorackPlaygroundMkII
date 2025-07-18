/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
 */

#pragma once

#include <Arduino.h>
#include "PatternSeq.hpp"

struct UserConfig
{
    char ver[15] = "rytsq_conf_001";
    // setting values
    float pattern[SEQUENCER_TOTAL];
    float pitches[SEQUENCER_TOTAL];
    float decays[SEQUENCER_TOTAL];
    float volumes[SEQUENCER_TOTAL];
    float triggers[SEQUENCER_TOTAL];
    float pans[SEQUENCER_TOTAL];
    float mutes[SEQUENCER_TOTAL];
    float agcMaxGain = 0.9;
    float isUpdateStepIndicator = 0.0;
    UserConfig()
    {
        pattern[0] = 3;
        pattern[1] = 16;
        pattern[2] = 28;
        pattern[3] = 29;
        pattern[4] = 25;
        pattern[5] = 32;
        for (int i = 0; i < SEQUENCER_TOTAL; ++i)
        {
            pitches[i] = 1.0;
            decays[i] = 0.8;
            triggers[i] = 0;
            mutes[i] = 0;
        }
        volumes[0] = 0.7;
        volumes[1] = 1.0;
        volumes[2] = 0.5;
        volumes[3] = 0.7;
        volumes[4] = 0.9;
        volumes[5] = 1.0;
        pans[0] = 0;
        pans[1] = 4;
        pans[2] = 3;
        pans[3] = 1;
        pans[4] = 2;
        pans[5] = 2;
        agcMaxGain = 0.9;
        isUpdateStepIndicator = 0.0;
    }
};
