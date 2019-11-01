/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** ============================================================================
 *  @file       IOLINK_memoryMap.h
 *
 *  @brief      PRU IO-Link master memory map and initialization
 *  ============================================================================
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifndef IOLINK_MEMORYMAP_H_
#define IOLINK_MEMORYMAP_H_

/* status definition */
#define IOLINK_PRU_STATUS_IDLE             (0x0U)
#define IOLINK_PRU_STATUS_COMPLETE         (0x1U)

/* memory map */
#define globalSts           (0x0000U)
#define globalCtrl          (0x0001U)
#define maxResponseTime     (0x0002U)
#define firmwareRev         (0x0003U)

#define status_idle         (0x00U)
#define status_complete     (0x01U)

#define ctrlCh0             (0x0004U)  /* 32 bits */
#define statusCh0           (0x0008U)  /* 16 bits */
#define errorCh0            (0x000AU) /* 16 bits */

#define ctrlCh1             (0x000CU)
#define statusCh1           (0x0010U)
#define errorCh1            (0x0012U)

#define ctrlCh2             (0x0014U)
#define statusCh2           (0x0018U)
#define errorCh2            (0x001AU)

#define ctrlCh3             (0x001CU)
#define statusCh3           (0x0020U)
#define errorCh3            (0x0022U)

#define ctrlCh4             (0x0024U)
#define statusCh4           (0x0028U)
#define errorCh4            (0x002AU)

#define ctrlCh5             (0x002CU)
#define statusCh5           (0x0030U)
#define errorCh5            (0x0032U)

#define ctrlCh6             (0x0034U)
#define statusCh6           (0x0038U)
#define errorCh6            (0x003AU)

#define ctrlCh7             (0x003CU)
#define statusCh7           (0x0040U)
#define errorCh7            (0x0042U)

#define PRU_CTRL_ADDR(x)    (ctrlCh0 + (0x8U * (x)))
#define PRU_STATUS_ADDR(x)  (statusCh0 + (0x8U * (x)))

#define receivebfrCh0       (0x0100U)
#define receivebfrCh1       (0x0180U)
#define receivebfrCh2       (0x0200U)
#define receivebfrCh3       (0x0280U)
#define receivebfrCh4       (0x0300U)
#define receivebfrCh5       (0x0380U)
#define receivebfrCh6       (0x0400U)
#define receivebfrCh7       (0x0480U)

#define PRU_RXBUF_ADDR(x)   (receivebfrCh0 + (0x80U * (x)))

#define transmitbfrCh0      (0x0500U)
#define transmitbfrCh1      (0x0600U)
#define transmitbfrCh2      (0x0700U)
#define transmitbfrCh3      (0x0800U)
#define transmitbfrCh4      (0x0900U)
#define transmitbfrCh5      (0x0A00U)
#define transmitbfrCh6      (0x0B00U)
#define transmitbfrCh7      (0x0C00U)

#define PRU_TXBUF_ADDR(x)  (transmitbfrCh0 + (0x100U * (x)))

#define channel0mem         (0x0D00U)
#define channel1mem         (0x0D10U)
#define channel2mem         (0x0D20U)
#define channel3mem         (0x0D30U)
#define channel4mem         (0x0D40U)
#define channel5mem         (0x0D50U)
#define channel6mem         (0x0D60U)
#define channel7mem         (0x0D70U)

/* LUT's of PRU 0 */
#define pru0LutAdr          (0x0E00U)
#define sampleratelutAdr    (0x0E00U)
#define startbitfilterAdr   (0x0F00U)
/* LUT's of PRU 1 */
#define pru1LutAdr          (0x0E00U)
#define averagingfilterAdr  (0x0E00U)
#define paritylutAdr        (0x0F00U)


/*
 * PRU pin configuration
 * The PRU uses GPI0 ... 7 for RX0 ... 7
 * TX0 ... 7 can be bound to any of the remaining GPO's
 */
#define channel0TxPin       (8U)
#define channel1TxPin       (9U)
#define channel2TxPin       (10U)
#define channel3TxPin       (11U)
#define channel4TxPin       (19U)
#define channel5TxPin       (12U)
#define channel6TxPin       (13U)
#define channel7TxPin       (18U)

/*
 * TxEn pin configuration
 * You can use any of the available GPIO pins for the TX enable signal
 */

/* TX enable GPIO port hw address */
#define channel0TxEnGpioAdr (0x48320000U)
#define channel1TxEnGpioAdr (0x48322000U)
#define channel2TxEnGpioAdr (0x48322000U)
#define channel3TxEnGpioAdr (0x48322000U)
#define channel4TxEnGpioAdr (0x48320000U)
#define channel5TxEnGpioAdr (0x48322000U)
#define channel6TxEnGpioAdr (0x48322000U)
#define channel7TxEnGpioAdr (0x4804C000U)

/* TX enable GPIO pin configuration */
#define channel0TxEnGpioPin (1U << 10U)
#define channel1TxEnGpioPin (1U << 4U)
#define channel2TxEnGpioPin (1U << 6U)
#define channel3TxEnGpioPin (1U << 23U)
#define channel4TxEnGpioPin (1U << 12U)
#define channel5TxEnGpioPin (1U << 26U)
#define channel6TxEnGpioPin (1U << 25U)
#define channel7TxEnGpioPin (1U << 8U)

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

/*
 * initialization values
 * this section sets the PRU's look-up table entries
 * each LUT is 256 bytes in size and can be exchanged for a more optimized version if necessary
 */

typedef struct lookUpTablesPRU0 lookUpTablesPRU0;
typedef struct lookUpTablesPRU1 lookUpTablesPRU1;

struct lookUpTablesPRU0
{
    uint8_t sampleratelut[256U];
    uint8_t startbitfilterlut[256U];
} __attribute__((packed));

struct lookUpTablesPRU1
{
    uint8_t symbolfilterlut[256U];
    uint8_t paritylut[256U];
} __attribute__((packed));

/* 8 bit look-up table initialization */
lookUpTablesPRU0 initData0 =
{
     /*
      * samplerate LUT
      * This look-up table serves as a baud rate selection table
      * It determined the sample rate of each channel for any of the 3 (or more) baud rates
      * The PRU will check for a bit set on bit position x. If you enter 2 (COM2) in the PRU's baud rate register,
      * it will only take a new sample if bit 2 is set in the LUT for a counter given offset.
      */
     .sampleratelut = {14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8,8,8,14,8,8,8,8,8,12,8,8,8,8,8,12,8,8,8},

     /*
      * startbit LUT
      * This look-up table indicates the position of a start bit edge in a given octet of bits.
      * Zero means that there is no valid startbit in the bit stream.
      */
     .startbitfilterlut = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,3,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,6,6,6,6,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,4,4,0,0,0,0,0,3,0,0,0,2,0,0,0}
};

lookUpTablesPRU1 initData1 =
{
     /*
      * symbolfilter LUT
      * This look-up table is a simple averaging filter.
      * NOTE: The signal level is indicated by bit 7 and not bit 0
      */
     .symbolfilterlut = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x80,0,0,0,0,0,0,0,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0,0,0,0,0,0,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0,0,0,0,0,0,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0,0,0,0,0,0,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0,0,0x80,0,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80},

     /*
      * parity LUT
      * This look-up table calculates the parity of a given octet
      * NOTE: The parity bit is indicated by bit 7 and not bit 0
      */
     .paritylut = {0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0,0x80,0x0,0x0,0x80,0x80,0x0,0x0,0x80,0x0,0x80,0x80,0x0}
};


uint32_t *memInitDataPRU0 = (uint32_t*)(&initData0);
uint32_t *memInitDataPRU1 = (uint32_t*)(&initData1);

#endif /* IOLINK_MEMORYMAP_H_ */
