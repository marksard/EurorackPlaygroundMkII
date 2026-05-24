/*!
 * ValueLock
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>

class ValueLock
{
public:
    ValueLock()
    {
        _lastValue = 0;
        _lock = true;
        _first = true;
    }

    bool update(int16_t value, int16_t lockThreshold = 32)
    {
        if (_first == true)
        {
            if (value == 0)
            {
                return true;
            }
            
            _first = false;
            _lastValue = value;
            return true;
        }

        // ロック・ロック解除
        if (_lock == true)
        {
            if (lockThreshold < abs(_lastValue - value))
            {
                _lock = false;
            }
        }
        else
        {
            _lastValue = value;
        }
        
        return _lock;
    }

    void setLock(bool value)
    {
        _lock = value;
    }

    bool getLock()
    {
        return _lock;
    }

protected:
    int16_t _lastValue;
    bool _lock;
    bool _first;
};
