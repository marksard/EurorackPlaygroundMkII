/*!
 * PWM wrappers
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include "hardware/pwm.h"  // PWMの制御用ヘッダー

// OUT_A/Bとは違うPWMチャンネルのPWM割り込みにすること
uint initPWMIntr(uint gpio, irq_handler_t handler, uint *pSlice, float sampleFreq, uint16_t wrap = 512, float cpuClock = 133000000.0)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    *pSlice = slice;

    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // 割り込み頻度
    pwm_set_wrap(slice, wrap - 1);
    pwm_set_clkdiv(slice, cpuClock / ((float)wrap * sampleFreq));
    pwm_set_enabled(slice, true);
    return slice;
}

void initPWM(uint gpio, uint16_t resolution, bool start = true, bool apol = false)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    pwm_config conf = pwm_get_default_config();
    pwm_config_set_output_polarity(&conf, apol, false);
    pwm_config_set_wrap(&conf, resolution - 1);
    // 最速にして滑らかなPWMを得る
    pwm_config_set_clkdiv(&conf, 1);
    pwm_init(slice, &conf, start);

    // pwm_set_wrap(slice, resolution - 1);
    // // 最速にして滑らかなPWMを得る
    // pwm_set_clkdiv(slice, 1);
    // pwm_set_enabled(slice, start);
}

void disablePWM(uint gpio)
{
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice, false);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, false);
}