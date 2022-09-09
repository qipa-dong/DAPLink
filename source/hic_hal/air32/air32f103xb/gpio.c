/**
 * @file    gpio.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "air32f10x.h"
#include "DAP_config.h"
#include "gpio.h"
#include "daplink.h"
#include "util.h"

static void busy_wait(uint32_t cycles)
{
    volatile uint32_t i;
    i = cycles;

    while (i > 0) {
        i--;
    }
}

//static uint32_t tim1_clk_div(uint32_t apb2clkdiv)
//{
//    switch (apb2clkdiv) {
//        case RCC_CFGR_PPRE2_DIV2:
//            return 1;
//        case RCC_CFGR_PPRE2_DIV4:
//            return 2;
//        case RCC_CFGR_PPRE2_DIV8:
//            return 4;
//        default: // RCC_CFGR_PPRE2_DIV1
//            return 1;
//    }
//}

static void output_clock_enable(void)
{
//      TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    RCC_ClocksTypeDef clk_init;
//    TIM_OCInitTypeDef pwm_config;
//    uint32_t period;
//    uint32_t source_clock;

//    RCC_GetClocksFreq(&clk_init);

//    /* Compute the period value to have TIMx counter clock equal to 8000000 Hz */
//    source_clock = SystemCoreClock / tim1_clk_div(clk_init.PCLK2_Frequency);
//    period = (uint32_t)(source_clock / 8000000) - 1;// todo

//    /* Set TIMx instance */

//    TIM_TimeBaseStructure.TIM_Period            = period;
//    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
//    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
//    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//period / 2;

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

//    TIM_DeInit(TIM1);
//    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//    
//    pwm_config.TIM_OCMode = TIM_OCMode_PWM2;
//    pwm_config.TIM_Pulse = 0;
//    pwm_config.TIM_OCPolarity = TIM_OCPolarity_High;
//    pwm_config.TIM_OCNPolarity = TIM_OCPolarity_High;
////pwm_config.OCFastMode
//    pwm_config.TIM_OCIdleState = TIM_OCIdleState_Reset;
//    pwm_config.TIM_OCNIdleState = TIM_OCIdleState_Reset;
//    TIM_OCStructInit(&pwm_config);

//    TIM_SetCompare1(TIM1, period / 2);
//    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    RCC_MCOConfig(RCC_MCO_HSE);

    return;
}

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // enable clock to ports
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    // Enable USB connect pin
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
    // Disable JTAG to free pins for other uses
    // Note - SWD is still enabled
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);

    USB_CONNECT_PORT_ENABLE();
    USB_CONNECT_OFF();
    GPIO_InitStructure.GPIO_Pin = USB_CONNECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(USB_CONNECT_PORT, &GPIO_InitStructure);
    // configure LEDs
    GPIO_WriteBit(RUNNING_LED_PORT, RUNNING_LED_PIN, Bit_SET);
    GPIO_InitStructure.GPIO_Pin = RUNNING_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RUNNING_LED_PORT, &GPIO_InitStructure);

    GPIO_WriteBit(CONNECTED_LED_PORT, CONNECTED_LED_PIN, Bit_SET);
    GPIO_InitStructure.GPIO_Pin = CONNECTED_LED_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(CONNECTED_LED_PORT, &GPIO_InitStructure);

    GPIO_WriteBit(PIN_CDC_LED_PORT, PIN_CDC_LED, Bit_SET);
    GPIO_InitStructure.GPIO_Pin = PIN_CDC_LED;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(PIN_CDC_LED_PORT, &GPIO_InitStructure);

    GPIO_WriteBit(PIN_MSC_LED_PORT, PIN_MSC_LED, Bit_SET);
    GPIO_InitStructure.GPIO_Pin = PIN_MSC_LED;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(PIN_MSC_LED_PORT, &GPIO_InitStructure);

    // reset button configured as gpio open drain output with a pullup
    GPIO_WriteBit(nRESET_PIN_PORT, nRESET_PIN, Bit_SET);
    GPIO_InitStructure.GPIO_Pin = nRESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(nRESET_PIN_PORT, &GPIO_InitStructure);
    GPIO_ForcePuPdCmd(nRESET_PIN_PORT, ENABLE);
    GPIO_ForcePullUpConfig(nRESET_PIN_PORT, nRESET_PIN);

    // Turn on power to the board. When the target is unpowered
    // it holds the reset line low.
    GPIO_WriteBit(POWER_EN_PIN_PORT, POWER_EN_PIN, Bit_RESET);
    GPIO_InitStructure.GPIO_Pin = POWER_EN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(POWER_EN_PIN_PORT, &GPIO_InitStructure);

    // Setup the 8MHz MCO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    output_clock_enable();

    // Let the voltage rails stabilize.  This is especailly important
    // during software resets, since the target's 3.3v rail can take
    // 20-50ms to drain.  During this time the target could be driving
    // the reset pin low, causing the bootloader to think the reset
    // button is pressed.
    // Note: With optimization set to -O2 the value 1000000 delays for ~85ms
    busy_wait(1000000);
}

void gpio_set_hid_led(gpio_led_state_t state)
{
    // LED is active low
    GPIO_WriteBit(PIN_HID_LED_PORT, PIN_HID_LED, state ? Bit_RESET : Bit_SET);
}

void gpio_set_cdc_led(gpio_led_state_t state)
{
    // LED is active low
    GPIO_WriteBit(PIN_CDC_LED_PORT, PIN_CDC_LED, state ? Bit_RESET : Bit_SET);
}

void gpio_set_msc_led(gpio_led_state_t state)
{
    // LED is active low
    GPIO_WriteBit(PIN_MSC_LED_PORT, PIN_MSC_LED, state ? Bit_RESET : Bit_SET);
}

uint8_t gpio_get_reset_btn_no_fwrd(void)
{
    return (nRESET_PIN_PORT->IDR & nRESET_PIN) ? 0 : 1;
}

uint8_t gpio_get_reset_btn_fwrd(void)
{
    return 0;
}


uint8_t GPIOGetButtonState(void)
{
    return 0;
}

void target_forward_reset(bool assert_reset)
{
    // Do nothing - reset is forwarded in gpio_get_sw_reset
}

void gpio_set_board_power(bool powerEnabled)
{
}
