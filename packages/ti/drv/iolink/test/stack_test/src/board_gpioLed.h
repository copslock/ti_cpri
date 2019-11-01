/**
 *  \file   board_gpioLed.c
 *
 *  \brief:
 *  GPIO LED configurations
 *
 *
 */

/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/
#ifndef _BOARD_GPIOLED_H_
#define _BOARD_GPIOLED_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/***********************************************************************/
/* Macros                                        */
/***********************************************************************/
/** @def BOARD_TRICOLOR0_RED
 *       Macro for configuring Tri color0 Red
 */
#define BOARD_TRICOLOR0_RED                     (1u << (0u))
/** @def BOARD_TRICOLOR0_GREEN
 *       Macro for configuring Tri color0 green
 */
#define BOARD_TRICOLOR0_GREEN                   (1u << (1u))
/** @def BOARD_TRICOLOR0_YELLOW
 *       Macro for configuring Tri color0 yellow
 */
#define BOARD_TRICOLOR0_YELLOW                  (1u << (2u))
/** @def BOARD_TRICOLOR1_RED
 *       Macro for configuring Tri color1 red
 */
#define BOARD_TRICOLOR1_RED                     (1u << (3u))
/** @def BOARD_TRICOLOR1_GREEN
 *       Macro for configuring Tri color1 green
 */
#define BOARD_TRICOLOR1_GREEN                   (1u << (4u))
/** @def BOARD_TRICOLOR1_YELLOW
 *       Macro for configuring Tri color1 yellow
 */
#define BOARD_TRICOLOR1_YELLOW                  (1u << (5u))

#ifdef TIESC_SPI_SLAVE_MODE
#ifdef iceAMIC11x
#define SPI_SLAVE_PDI_INT_PIN                   7
#define SPI_SLAVE_FIRMWARE_LOADED_PIN           8
#else
#define SPI_SLAVE_PDI_INT_PIN                   19
#endif
#endif

#ifdef TIESC_SPI_MASTER_MODE
#define SPI_MASTER_PDI_INT_PIN                  19
#define SPI_MASTER_SYNC0_INT_PIN                20
#define SPI_MASTER_SYNC1_INT_PIN                21
#endif

#ifdef SOC_K2G
#define BOARD_LCD_BST_CONV_CTL_GPIO             12
#define BOARD_LCD_RESET                         13
#endif

#define GPIO_PIN_VAL_LOW     (0U)
#define GPIO_PIN_VAL_HIGH    (1U)

/**
 * @brief GPIO Pinmux configuration
 */
void gpioLedPinmuxConfig();

/**
 * @brief API used for toggling Tri color LED
 *
 * @param gpioLeds LEDs to toggle
 * @param value set or reset
 */
void  Board_setTriColorLED(uint32_t gpioLeds, uint8_t value);

#endif /* _BOARD_GPIOLED_H_*/
