/*!
 * MultiWaveOscEx class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "lib/MultiWaveOsc.hpp"

extern const char* noteName[];

class MultiWaveOscEx : public MultiWaveOsc
{
public:
    const char waveName[Wave::MAX + 1][9] = {" SQUARE ", "SAWWAVE ", "MUL-TRI", "TRIANGLE", "  SINE  ", "W-NOISE "};

public:
    MultiWaveOscEx() : MultiWaveOsc()
    {
        _coarse = 0;
        _coarseNoteNameIndex = 0;
    }

    void setFrequencyFromNoteNameIndex(int8_t value)
    {
        value = constrain(value, 0, 127);
        setFrequency(noteFreq[value]);
    }

    bool setCourceFromNoteNameIndex(int8_t value)
    {
        bool result = _coarseNoteNameIndex != value;
        value = constrain(value, 0, 127);
        _coarseNoteNameIndex = value;
        _coarse = noteFreq[value];
        return result;
    }

    float getCource() { return _coarse; }

    bool setNoteNameFromFrequency(float frequency)
    {
        uint8_t noteNameIndex = getNoteNameFromFreq(frequency);
        bool result = _coarseNoteNameIndex != noteNameIndex;
        _coarseNoteNameIndex = noteNameIndex;
        return result;
    }

    bool setFreqName(float frequency)
    {
        bool result = false;
        if (frequency > (_coarse + 0.1) || frequency < (_coarse - 0.1))
        {
            result = true;
            _coarse = frequency;
        }
        return result;
    }
    const char *getWaveName() { return waveName[_wave]; }
    const char *getNoteName() { return noteName[_coarseNoteNameIndex]; }

    const char *getNoteNameOrFreq(bool freqName = true)
    {
        if (freqName)
        {
            sprintf(_freqName, "%6.2f", _coarse);
            return _freqName;
        }

        return noteName[_coarseNoteNameIndex];
    }

private:
    char _freqName[8];
    float _coarse;
    uint8_t _coarseNoteNameIndex;
};

const char* noteName[] = {
    "C  0",
    "C# 0",
    "D  0",
    "D# 0",
    "E  0",
    "F  0",
    "F# 0",
    "G  0",
    "G# 0",
    "A  0",
    "A# 0",
    "B  0",
    "C  1",
    "C# 1",
    "D  1",
    "D# 1",
    "E  1",
    "F  1",
    "F# 1",
    "G  1",
    "G# 1",
    "A  1",
    "A# 1",
    "B  1",
    "C  2",
    "C# 2",
    "D  2",
    "D# 2",
    "E  2",
    "F  2",
    "F# 2",
    "G  2",
    "G# 2",
    "A  2",
    "A# 2",
    "B  2",
    "C  3",
    "C# 3",
    "D  3",
    "D# 3",
    "E  3",
    "F  3",
    "F# 3",
    "G  3",
    "G# 3",
    "A  3",
    "A# 3",
    "B  3",
    "C  4",
    "C# 4",
    "D  4",
    "D# 4",
    "E  4",
    "F  4",
    "F# 4",
    "G  4",
    "G# 4",
    "A  4",
    "A# 4",
    "B  4",
    "C  5",
    "C# 5",
    "D  5",
    "D# 5",
    "E  5",
    "F  5",
    "F# 5",
    "G  5",
    "G# 5",
    "A  5",
    "A# 5",
    "B  5",
    "C  6",
    "C# 6",
    "D  6",
    "D# 6",
    "E  6",
    "F  6",
    "F# 6",
    "G  6",
    "G# 6",
    "A  6",
    "A# 6",
    "B  6",
    "C  7",
    "C# 7",
    "D  7",
    "D# 7",
    "E  7",
    "F  7",
    "F# 7",
    "G  7",
    "G# 7",
    "A  7",
    "A# 7",
    "B  7",
    "C  8",
    "C# 8",
    "D  8",
    "D# 8",
    "E  8",
    "F  8",
    "F# 8",
    "G  8",
    "G# 8",
    "A  8",
    "A# 8",
    "B  8",
    "C  9",
    "C# 9",
    "D  9",
    "D# 9",
    "E  9",
    "F  9",
    "F# 9",
    "G  9",
    "G# 9",
    "A  9",
    "A# 9",
    "B  9",
    "C 10",
    "C#10",
    "D 10",
    "D#10",
    "E 10",
    "F 10",
    "F#10",
    "G 10"};