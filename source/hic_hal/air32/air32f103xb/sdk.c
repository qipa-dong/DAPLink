/**
 * @file    sdk.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2017-2017, ARM Limited, All Rights Reserved
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
#include "cortex_m.h"
#include "air_rcc.h"

TIM_TimeBaseInitTypeDef timer;
uint32_t time_count;

//static uint32_t tim2_clk_div(uint32_t apb1clkdiv);

/**
    * @brief  Switch the PLL source from HSI to HSE bypass, and select the PLL as SYSCLK
  *         source.
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE bypass)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void sdk_init()
{
    RCC_DeInit(); //复位RCC寄存器

	RCC_HSEConfig(RCC_HSE_ON); //使能HSE
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
		; //等待HSE就绪

	RCC_PLLCmd(DISABLE);										 //关闭PLL
	AIR_RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_27, 1); //配置PLL,8*27=216MHz

	RCC_PLLCmd(ENABLE); //使能PLL
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		; //等待PLL就绪

	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //选择PLL作为系统时钟

	RCC_HCLKConfig(RCC_SYSCLK_Div1); //配置AHB时钟
	RCC_PCLK1Config(RCC_HCLK_Div2);	 //配置APB1时钟
	RCC_PCLK2Config(RCC_HCLK_Div1);	 //配置APB2时钟

	RCC_LSICmd(ENABLE); //使能内部低速时钟
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
		;				//等待LSI就绪
	RCC_HSICmd(ENABLE); //使能内部高速时钟
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
		; //等待HSI就绪
}

//HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
//{
//    HAL_StatusTypeDef ret;
//    RCC_ClkInitTypeDef clk_init;
//    uint32_t unused;
//    uint32_t prescaler;
//    uint32_t source_clock;

//    HAL_RCC_GetClockConfig(&clk_init, &unused);

//    /* Compute the prescaler value to have TIMx counter clock equal to 4000 Hz */
//    source_clock = SystemCoreClock / tim2_clk_div(clk_init.APB1CLKDivider);
//    prescaler = (uint32_t)(source_clock / 4000) - 1;

//    /* Set TIMx instance */
//    timer.Instance = TIM2;

//    timer.Init.Period            = 0xFFFF;
//    timer.Init.Prescaler         = prescaler;
//    timer.Init.ClockDivision     = 0;
//    timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
//    timer.Init.RepetitionCounter = 0;

//    __HAL_RCC_TIM2_CLK_ENABLE();

//    ret = HAL_TIM_Base_DeInit(&timer);
//    if (ret != HAL_OK) {
//        return ret;
//    }

//    time_count = 0;
//    ret = HAL_TIM_Base_Init(&timer);
//    if (ret != HAL_OK) {
//        return ret;
//    }

//    ret = HAL_TIM_Base_Start(&timer);
//    if (ret != HAL_OK) {
//        return ret;
//    }

//    return HAL_OK;
//}


//void HAL_IncTick(void)
//{
//    // Do nothing
//}

//uint32_t HAL_GetTick(void)
//{
//    cortex_int_state_t state;
//    state = cortex_int_get_and_disable();
//    const uint32_t ticks = __HAL_TIM_GET_COUNTER(&timer) / 4;
//    time_count += (ticks - time_count) & 0x3FFF;
//    cortex_int_restore(state);
//    return time_count;
//}

//void HAL_SuspendTick(void)
//{
//    HAL_TIM_Base_Start(&timer);
//}

//void HAL_ResumeTick(void)
//{
//    HAL_TIM_Base_Stop(&timer);
//}

//static uint32_t tim2_clk_div(uint32_t apb1clkdiv)
//{
//    switch (apb1clkdiv) {
//        case RCC_CFGR_PPRE1_DIV2:
//            return 1;
//        case RCC_CFGR_PPRE1_DIV4:
//            return 2;
//        case RCC_CFGR_PPRE1_DIV8:
//            return 4;
//        case RCC_CFGR_PPRE1_DIV16:
//            return 8;
//        default:
//            return 1;
//    }
//}
