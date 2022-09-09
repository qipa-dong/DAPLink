/**
 * @file    uart.c
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

#include "string.h"

#include "air32f10x.h"
#include "uart.h"
#include "gpio.h"
#include "util.h"
#include "circ_buf.h"
#include "IO_Config.h"

// For usart

#if 1   // usart1支持13.5M的波特率，但不支持CTS和RTS(和USB接口冲突)
#define CDC_UART                     USART1
#define CDC_UART_ENABLE()            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
#define CDC_UART_DISABLE()           RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,DISABLE);
#define CDC_UART_IRQn                USART1_IRQn
#define CDC_UART_IRQn_Handler        USART1_IRQHandler

#define UART_PINS_PORT_ENABLE()      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE)
#define UART_PINS_PORT_DISABLE()     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,DISABLE);

#define UART_TX_PORT                 GPIOA
#define UART_TX_PIN                  GPIO_Pin_9

#define UART_RX_PORT                 GPIOA
#define UART_RX_PIN                  GPIO_Pin_10

#define UART_CTS_PORT                GPIOA
#define UART_CTS_PIN                 GPIO_Pin_11

#define UART_RTS_PORT                GPIOA
#define UART_RTS_PIN                 GPIO_Pin_12
#else// usart2支持6M的波特率，但支持CTS和RTS
#define CDC_UART                     USART2
#define CDC_UART_ENABLE()            RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
#define CDC_UART_DISABLE()           RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,DISABLE);
#define CDC_UART_IRQn                USART2_IRQn
#define CDC_UART_IRQn_Handler        USART2_IRQHandler

#define UART_PINS_PORT_ENABLE()      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE)
#define UART_PINS_PORT_DISABLE()     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,DISABLE);

#define UART_TX_PORT                 GPIOA
#define UART_TX_PIN                  GPIO_Pin_2

#define UART_RX_PORT                 GPIOA
#define UART_RX_PIN                  GPIO_Pin_3

#define UART_CTS_PORT                GPIOA
#define UART_CTS_PIN                 GPIO_Pin_0

#define UART_RTS_PORT                GPIOA
#define UART_RTS_PIN                 GPIO_Pin_1
#endif


#define RX_OVRF_MSG         "<DAPLink:Overflow>\n"
#define RX_OVRF_MSG_SIZE    (sizeof(RX_OVRF_MSG) - 1)
#define BUFFER_SIZE         (512)

circ_buf_t write_buffer;
uint8_t write_buffer_data[BUFFER_SIZE];
circ_buf_t read_buffer;
uint8_t read_buffer_data[BUFFER_SIZE];

static UART_Configuration configuration = {
    .Baudrate = 115200,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

static void clear_buffers(void)
{
    circ_buf_init(&write_buffer, write_buffer_data, sizeof(write_buffer_data));
    circ_buf_init(&read_buffer, read_buffer_data, sizeof(read_buffer_data));
}

int32_t uart_initialize(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
    USART_ITConfig(CDC_UART, USART_IT_RXNE, DISABLE);
    clear_buffers();

    CDC_UART_ENABLE();
    UART_PINS_PORT_ENABLE();

    //TX pin
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(UART_TX_PORT, &GPIO_InitStructure);
    //RX pin
    GPIO_InitStructure.GPIO_Pin = UART_RX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_RX_PORT, &GPIO_InitStructure);
    GPIO_ForcePuPdCmd(UART_RX_PORT, ENABLE);
    GPIO_ForcePullUpConfig(UART_RX_PORT, UART_RX_PIN);
#if 0  // usart1支持13.5M的波特率，但不支持CTS和RTS(和USB接口冲突)
    //CTS pin, input
    GPIO_InitStructure.GPIO_Pin = UART_CTS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_CTS_PORT, &GPIO_InitStructure);
    GPIO_ForcePuPdCmd(UART_CTS_PORT, ENABLE);
    GPIO_ForcePullUpConfig(UART_CTS_PORT, UART_CTS_PIN);
    //RTS pin, output low
    GPIO_ResetBits(UART_RTS_PORT, UART_RTS_PIN);
    GPIO_InitStructure.GPIO_Pin = UART_RTS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(UART_RTS_PORT, &GPIO_InitStructure);
#endif

    NVIC_EnableIRQ(CDC_UART_IRQn);

    return 1;
}

int32_t uart_uninitialize(void)
{
    USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
    USART_ITConfig(CDC_UART, USART_IT_RXNE, DISABLE);
    clear_buffers();
    return 1;
}

int32_t uart_reset(void)
{
    USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
    USART_ITConfig(CDC_UART, USART_IT_RXNE, DISABLE);
    clear_buffers();
    USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    USART_InitTypeDef USART_InitStructure;

    memset(&USART_InitStructure, 0, sizeof(USART_InitStructure));

    // parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD) {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    } else if(config->Parity == UART_PARITY_EVEN) {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    } else if(config->Parity == UART_PARITY_NONE) {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    } else {   //Other not support
        USART_InitStructure.USART_Parity = USART_Parity_No;
        configuration.Parity = UART_PARITY_NONE;
    }

    // stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2) {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    } else if(config->StopBits == UART_STOP_BITS_1_5) {
        //USART_InitStructure.USART_StopBits = USART_StopBits_1_5;// todo not support?
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        configuration.StopBits = UART_STOP_BITS_2;
    } else if(config->StopBits == UART_STOP_BITS_1) {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    } else {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }

    //Only 8 bit support
    configuration.DataBits = UART_DATA_BITS_8;
    if (USART_InitStructure.USART_Parity == USART_Parity_Odd || USART_InitStructure.USART_Parity == USART_Parity_Even) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    } else {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }

    // No flow control
    // usart1支持13.5M的波特率，但不支持CTS和RTS(和USB接口冲突)
    configuration.FlowControl = UART_FLOW_CONTROL_NONE;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    
    // Specified baudrate
    configuration.Baudrate = config->Baudrate;
    USART_InitStructure.USART_BaudRate = config->Baudrate;

    // TX and RX
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    // Disable uart and tx/rx interrupt
    USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
    USART_ITConfig(CDC_UART, USART_IT_RXNE, DISABLE);

    clear_buffers();
    
    USART_DeInit(CDC_UART);
    USART_Init(CDC_UART, &USART_InitStructure);
    USART_Cmd(CDC_UART, ENABLE);

    USART_ITConfig(CDC_UART, USART_IT_RXNE, ENABLE);

    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = UART_FLOW_CONTROL_NONE;

    return 1;
}

void uart_set_control_line_state(uint16_t ctrl_bmp)
{
}

int32_t uart_write_free(void)
{
    return circ_buf_count_free(&write_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt = circ_buf_write(&write_buffer, data, size);
    USART_ITConfig(CDC_UART, USART_IT_TXE, ENABLE);

    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    return circ_buf_read(&read_buffer, data, size);
}

void CDC_UART_IRQn_Handler(void)
{
    const uint32_t sr = CDC_UART->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t dat = CDC_UART->DR;
        uint32_t free = circ_buf_count_free(&read_buffer);
        if (free > RX_OVRF_MSG_SIZE) {
            circ_buf_push(&read_buffer, dat);
        } else if (RX_OVRF_MSG_SIZE == free) {
            circ_buf_write(&read_buffer, (uint8_t*)RX_OVRF_MSG, RX_OVRF_MSG_SIZE);
        } else {
            // Drop character
        }
    }

    if (sr & USART_SR_TXE) {
        if (circ_buf_count_used(&write_buffer) > 0) {
            CDC_UART->DR = circ_buf_pop(&write_buffer);
        } else {
            USART_ITConfig(CDC_UART, USART_IT_TXE, DISABLE);
        }
    }
}
