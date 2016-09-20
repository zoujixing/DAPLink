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

#include "stm32f10x.h"
#include "uart.h"
#include "gpio.h"

// For usart
#define USARTx_BASE                  USART2
#define USARTx_CLOCK_ENABLE()        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define USARTx_CLOCK_DISABLE()       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE)
#define USARTx_IRQn                  USART2_IRQn
#define USARTx_IRQn_Handler          USART2_IRQHandler

#define USARTx_PINS_PORT_ENABLE()    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE)
#define USARTx_PINS_PORT_DISABLE()   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , DISABLE)

#define USARTx_TX_PORT               GPIOA
#define USARTx_TX_PIN                GPIO_Pin_2
#define USARTx_TX_PIN_SOURCE         GPIO_PinSource2

#define USARTx_RX_PORT               GPIOA
#define USARTx_RX_PIN                GPIO_Pin_3
#define USARTx_RX_PIN_SOURCE         GPIO_PinSource3

#define USARTx_CTS_PORT              GPIOA
#define USARTx_CTS_PIN               GPIO_Pin_0
#define USARTx_CTS_PIN_SOURCE        GPIO_PinSource0

#define USARTx_RTS_PORT              GPIOA
#define USARTx_RTS_PIN               GPIO_Pin_1
#define USARTx_RTS_PIN_SOURCE        GPIO_PinSource1

// Size must be 2^n for using quick wrap around
#define  BUFFER_SIZE (512)

typedef struct {
    volatile uint8_t  data[BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
}ring_buf_t;

static ring_buf_t tx_buffer, rx_buffer;

static uint32_t tx_in_progress = 0;

static UART_Configuration configuration = {
    .Baudrate = 9600,
    .DataBits = UART_DATA_BITS_8,
    .Parity = UART_PARITY_NONE,
    .StopBits = UART_STOP_BITS_1,
    .FlowControl = UART_FLOW_CONTROL_NONE,
};

extern uint32_t SystemCoreClock;

static void delay(uint32_t n)
{
    while(n--) {
        __NOP();
        __NOP();
        __NOP();
    }
}

static void clear_buffers(void)
{
    memset((void *)&rx_buffer, 0xBB, sizeof(ring_buf_t));
    rx_buffer.head = 0;
    rx_buffer.tail = 0;
    memset((void *)&tx_buffer, 0xBB, sizeof(ring_buf_t));
    tx_buffer.head = 0;
    tx_buffer.tail = 0;
}

static int16_t read_available(ring_buf_t *buffer)
{
    return ((BUFFER_SIZE + buffer->head - buffer->tail) % BUFFER_SIZE);
}

static int16_t write_available(ring_buf_t *buffer)
{
    int16_t cnt;

    cnt = (buffer->tail - buffer->head - 1);
    if(cnt < 0)
        cnt += BUFFER_SIZE;

    return cnt;
}

int32_t uart_initialize(void)
{
    uint16_t data_bits;
    uint16_t parity;
    uint16_t stop_bits;

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    USART_ITConfig(USARTx_BASE, USART_IT_RXNE|USART_IT_TXE, DISABLE);
    clear_buffers();

    USARTx_CLOCK_ENABLE();
    USARTx_PINS_PORT_ENABLE();
    //TX pin
    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(USARTx_TX_PORT, &GPIO_InitStructure);
    //RX pin
    GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USARTx_RX_PORT, &GPIO_InitStructure);
    //CTS pin, input
    GPIO_InitStructure.GPIO_Pin = USARTx_CTS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(USARTx_CTS_PORT, &GPIO_InitStructure);
    //RTS pin, output high
    GPIO_InitStructure.GPIO_Pin = USARTx_RTS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(USARTx_RTS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);

    //Only 8 bit support
    data_bits = USART_WordLength_8b;
    // parity
    if(configuration.Parity == UART_PARITY_ODD)
        parity = USART_Parity_Odd;
    else if(configuration.Parity == UART_PARITY_EVEN)
        parity = USART_Parity_Even;
    else
        parity = USART_Parity_No;
    // stop bits
    if(configuration.StopBits == UART_STOP_BITS_2)
        stop_bits = USART_StopBits_2;
    else if(configuration.StopBits == UART_STOP_BITS_1_5)
        stop_bits = USART_StopBits_1_5;
    else if(configuration.StopBits == UART_STOP_BITS_1)
        stop_bits = USART_StopBits_1;
    else
        stop_bits = USART_StopBits_1;
    //
    USART_InitStructure.USART_BaudRate = configuration.Baudrate;
    USART_InitStructure.USART_WordLength = data_bits;
    USART_InitStructure.USART_StopBits = stop_bits;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx_BASE, &USART_InitStructure);
    //
    NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    // Enable RX interrupt
    USART_ITConfig(USARTx_BASE, USART_IT_RXNE, ENABLE);
    // Initially disable TxEmpty Interrupt
    USART_ITConfig(USARTx_BASE, USART_IT_TXE, DISABLE);

    NVIC_ClearPendingIRQ(USARTx_IRQn);

    USART_Cmd(USARTx_BASE, ENABLE);
    // Set RTS LOW
    GPIO_ResetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);

    return 1;
}

int32_t uart_uninitialize(void)
{
    USART_Cmd(USARTx_BASE, DISABLE);
    USART_ITConfig(USARTx_BASE, USART_IT_RXNE|USART_IT_TXE, DISABLE);
    clear_buffers();
    return 1;
}

int32_t uart_reset(void)
{
    uart_initialize();
    tx_in_progress = 0;
    return 1;
}

int32_t uart_set_configuration(UART_Configuration *config)
{
    uint16_t data_bits;
    uint16_t parity;
    uint16_t stop_bits;

    USART_InitTypeDef USART_InitStructure;
    // Set RTS HIGH
    GPIO_SetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);
    delay(500);

    // Disable uart and tx/rx interrupter
    //USART_Cmd(USARTx_BASE, DISABLE);
    //USART_ITConfig(USARTx_BASE, USART_IT_RXNE|USART_IT_TXE, DISABLE);
    NVIC_DisableIRQ(USARTx_IRQn);
    
    //Only 8 bit support
    data_bits = USART_WordLength_8b;
    configuration.DataBits = UART_DATA_BITS_8;
    // parity
    configuration.Parity = config->Parity;
    if(config->Parity == UART_PARITY_ODD)
        parity = USART_Parity_Odd;
    else if(config->Parity == UART_PARITY_EVEN)
        parity = USART_Parity_Even;
    else if(config->Parity == UART_PARITY_NONE)
        parity = USART_Parity_No;
    else {   //Other not support
        parity = USART_Parity_No;
        configuration.Parity = UART_PARITY_NONE;
    }
    // stop bits
    configuration.StopBits = config->StopBits;
    if(config->StopBits == UART_STOP_BITS_2)
        stop_bits = USART_StopBits_2;
    else if(config->StopBits == UART_STOP_BITS_1_5)
        stop_bits = USART_StopBits_1_5;
    else if(config->StopBits == UART_STOP_BITS_1)
        stop_bits = USART_StopBits_1;
    else {
        stop_bits = USART_StopBits_1;
        configuration.StopBits = UART_STOP_BITS_1;
    }
    configuration.Baudrate = config->Baudrate;
    configuration.FlowControl = UART_FLOW_CONTROL_NONE;
    //
    USART_InitStructure.USART_BaudRate = config->Baudrate;
    USART_InitStructure.USART_WordLength = data_bits;
    USART_InitStructure.USART_StopBits = stop_bits;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx_BASE, &USART_InitStructure);
    // Enable RX interrupt
    //USART_ITConfig(USARTx_BASE, USART_IT_RXNE, ENABLE);
    // Initially disable TxEmpty Interrupt
    //USART_ITConfig(USARTx_BASE, USART_IT_TXE, DISABLE);
    // Enable USARTx
    //USART_Cmd(USARTx_BASE, ENABLE);
    NVIC_EnableIRQ(USARTx_IRQn);
    // Check whether rx_buffer is full, if not, set RTS LOW
    if( write_available(&rx_buffer) > 0)
        GPIO_ResetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);

    return 1;
}

int32_t uart_get_configuration(UART_Configuration *config)
{
    config->Baudrate = configuration.Baudrate;
    config->DataBits = configuration.DataBits;
    config->Parity   = configuration.Parity;
    config->StopBits = configuration.StopBits;
    config->FlowControl = configuration.FlowControl;

    return 1;
}

int32_t uart_write_free(void)
{
    return write_available(&tx_buffer);
}

int32_t uart_write_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt, len;

    if(size == 0)
        return 0;

    len = write_available(&tx_buffer);
    if(len > size)
        len = size;

    cnt = len;
    while(len--) {
        tx_buffer.data[tx_buffer.head++] = *data++;
        if(tx_buffer.head >= BUFFER_SIZE)
            tx_buffer.head = 0;
    }

    if((!tx_in_progress) && (GPIO_ReadInputDataBit(USARTx_CTS_PORT, USARTx_CTS_PIN)==Bit_RESET)) {
        // Wait for tx is free
        //while(USART_GetITStatus(CDC_UART, USART_IT_TXE) == RESET);

        tx_in_progress = 1;
        USART_SendData(USARTx_BASE, tx_buffer.data[tx_buffer.tail++]);
        if(tx_buffer.tail >= BUFFER_SIZE)
            tx_buffer.tail = 0;
        // Enale tx interrupt
        USART_ITConfig(USARTx_BASE, USART_IT_TXE, ENABLE);
    }

    return cnt;
}

int32_t uart_read_data(uint8_t *data, uint16_t size)
{
    uint32_t cnt, len;

    if(size == 0) {
        return 0;
    }

    len = read_available(&rx_buffer);
    if(len > size)
        len = size;

    cnt = len;
    while(len--) {
        *data++ = rx_buffer.data[rx_buffer.tail++];
        if(rx_buffer.tail >= BUFFER_SIZE)
            rx_buffer.tail = 0;
    }
    // Release RTS
    if((write_available(&rx_buffer)>2) && (GPIO_ReadOutputDataBit(USARTx_RTS_PORT, USARTx_RTS_PIN)==Bit_SET))
        GPIO_ResetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);

    return cnt;
}

void USARTx_IRQn_Handler(void)
{
    uint8_t  dat;
    uint32_t cnt;

    if(USART_GetITStatus(USARTx_BASE, USART_IT_ERR) != RESET){
        USART_ClearITPendingBit(USARTx_BASE, USART_IT_ERR|USART_IT_RXNE|USART_IT_TXE);
    }

    if(USART_GetITStatus(USARTx_BASE, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(USARTx_BASE, USART_IT_RXNE);
        cnt = write_available(&rx_buffer);
        dat = USART_ReceiveData(USARTx_BASE);
        if(cnt) {
            rx_buffer.data[rx_buffer.head++] = dat;
            if(rx_buffer.head >= BUFFER_SIZE)
                rx_buffer.head = 0;
            
            if(cnt <= 2) {
                // for flow control, need to set RTS = 1
                GPIO_SetBits(USARTx_RTS_PORT, USARTx_RTS_PIN);
            }
        }
    }

    if(USART_GetITStatus(USARTx_BASE, USART_IT_TXE) != RESET) {
        USART_ClearITPendingBit(USARTx_BASE, USART_IT_TXE);
        cnt = read_available(&tx_buffer);
        if(cnt == 0) {
            USART_ITConfig(USARTx_BASE, USART_IT_TXE, DISABLE);
            tx_in_progress = 0;
        }
        else {
            if(GPIO_ReadInputDataBit(USARTx_CTS_PORT, USARTx_CTS_PIN)==Bit_RESET) {
                // CTS is LOW
                USART_SendData(USARTx_BASE, tx_buffer.data[tx_buffer.tail++]);
                if(tx_buffer.tail >= BUFFER_SIZE)
                    tx_buffer.tail = 0;
                tx_in_progress = 1;
            }
            else {
                USART_ITConfig(USARTx_BASE, USART_IT_TXE, DISABLE);
                tx_in_progress = 0;
            }
        }
    }
}
