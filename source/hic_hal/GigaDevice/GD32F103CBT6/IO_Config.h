/**
 * @file    IO_Config.h
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

#ifndef __IO_CONFIG_H__
#define __IO_CONFIG_H__

#include "gd32f10x.h"
#include "compiler.h"
#include "daplink.h"

COMPILER_ASSERT(DAPLINK_HIC_ID == DAPLINK_HIC_ID_GD32F103CB);

//USB control pin
#define USB_CONNECT_PORT_ENABLE()    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA, ENABLE)
#define USB_CONNECT_PORT_DISABLE()   RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_GPIOA, DISABLE)
#define USB_CONNECT_PORT             GPIOA
#define USB_CONNECT_PIN              GPIO_PIN_15
#define USB_CONNECT_ON()             (USB_CONNECT_PORT->BOR = USB_CONNECT_PIN)
#define USB_CONNECT_OFF()            (USB_CONNECT_PORT->BCR = USB_CONNECT_PIN)

//Connected LED
#define CONNECTED_LED_PORT           GPIOA
#define CONNECTED_LED_PIN            GPIO_PIN_9
#define CONNECTED_LED_PIN_Bit        9

//When bootloader, disable the target port(not used)
#define POWER_EN_PIN_PORT            GPIOB
#define POWER_EN_PIN                 GPIO_PIN_15
#define POWER_EN_Bit                 15

//Press and power, enter bootloader 7
//When daplnk_if, reset target board
#define nRESET_PIN_PORT              GPIOB
#define nRESET_PIN                   GPIO_PIN_0
#define nRESET_PIN_Bit               0

//SWD
#define SWCLK_TCK_PIN_PORT           GPIOB
#define SWCLK_TCK_PIN                GPIO_PIN_13
#define SWCLK_TCK_PIN_Bit            13

#define SWDIO_OUT_PIN_PORT           GPIOB
#define SWDIO_OUT_PIN                GPIO_PIN_14
#define SWDIO_OUT_PIN_Bit            14

#define SWDIO_IN_PIN_PORT            GPIOB
#define SWDIO_IN_PIN                 GPIO_PIN_12
#define SWDIO_IN_PIN_Bit             12

//LEDs
//USB status LED
#define RUNNING_LED_PORT             GPIOB
#define RUNNING_LED_PIN              GPIO_PIN_5
#define RUNNING_LED_Bit              5

#define PIN_HID_LED_PORT             GPIOA
#define PIN_HID_LED                  GPIO_PIN_9
#define PIN_HID_LED_Bit              9

#define PIN_CDC_LED_PORT             GPIOA
#define PIN_CDC_LED                  GPIO_PIN_9
#define PIN_CDC_LED_Bit              9

#define PIN_MSC_LED_PORT             GPIOA
#define PIN_MSC_LED                  GPIO_PIN_9
#define PIN_MSC_LED_Bit              9


#endif