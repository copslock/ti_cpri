/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/* ============================================================= */
/**
 *   @file  tsip_cfg.h
 *
 *   path  ti/drv/tsip/tsip_cfg.h
 *
 *   @brief  Telecom Serial Interface Port (TSIP) sub-system configuration for application
 *
 */
#ifndef _TSIP_CFG_H
#define _TSIP_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/cslr_device.h>


/**
 *  @def  TSIP_MAX_TIMESLOTS
 *        The maximal number of TSIP timeslots to be used
 *        
 */
#define TSIP_MAX_TIMESLOTS       128

/**
 *  @def  TSIP_ELEMENT_BUF_SIZE
 *        The number of bytes allocated for each timeslot in the dma buffer.
 *        Set to 1 unless hardware companding is used
 */
#define TSIP_ELEMENT_BUF_SIZE    1


/**
 *  @def  TSIP_SUB_FRAME_RATES
 *        Sub frame size for each TSIP port
 */
#define TSIP_SUB_FRAME_RATES        8, 8

/**
 *  @def  TSIP_UNOPENED_PORT_FRAME_ALLOC
 *        Default number of frames per dma buffer for ports
 *        that haven't been enabled yet
 */
#define TSIP_UNOPENED_PORT_FRAME_ALLOC  16


/**
 *  @defgroup Port-Link-Timeslot Bits
 *  @{
 *
 *  @name TSIP Port-Link-Timeslot Bit Allocation
 *
 *  Define which bits in a 16 bit timeslot field specify the port number, 
 *  the link number, and the timeslot number. The bit allocation depends
 *  on the maximal number of ports, links, timeslots to be supported.
 *  In the following example, the maximal number of timeslots, link, and
 *  ports is 512, 8, 4, respectively.
 *
 *     15    14   13   12  11          9    8                   0
 *   /-------------------------------------------------------------\
 *   | unused  |  port   |     link      |     timeslot            |
 *   \-------------------------------------------------------------/
 */
/*@{*/

/**
 *  @def  TSIP_PORT_MSB
 *        MSB of TSIP Port bits
 */
#define TSIP_PORT_MSB    13
/**
 *  @def  TSIP_PORT_LSB
 *        LSB of TSIP Port bits
 */
#define TSIP_PORT_LSB    12
/**
 *  @def  TSIP_LINK_MSB
 *        MSB of TSIP Link bits
 */
#define TSIP_LINK_MSB    11
/**
 *  @def  TSIP_LINK_LSB
 *        LSB of TSIP Link bits
 */
#define TSIP_LINK_LSB     9 
/**
 *  @def  TSIP_TS_MSB
 *        MSB of TSIP Timeslot bits
 */
#define TSIP_TS_MSB       8
/**
 *  @def  TSIP_TS_LSB
 *        LSB of TSIP Timeslot bits
 */
#define TSIP_TS_LSB       0

/*@}*/  
/** @} */


/**
 *  @def  deviceWhoAmI
 *        Macro to get the core ID
 */
extern cregister  volatile unsigned int DNUM;
#define deviceWhoAmI()      DNUM

/**
 *  @def  TSIP_ALIGN_STRUCT
 *        Macro to even align internal carving of memory
 */
#define TSIP_ALIGN_STRUCT(x)       ((x) = (x+7)&(~7))

/**
 *  @def  GLOBAL_ADDR
 *        Macro to convert local address to global address
 */
#define GLOBAL_ADDR(addr) ((((uint32_t)addr>>24)&0xFF) ? ((uint32_t)addr) : ((1<<28)|((int16_t)DNUM<<24)|((uint32_t)(addr)&0x00ffffff)))

/**
 *  @def  TSIP_CORE_STAGGER
 *        Define to use the feature of core staggering.
 *
 *  @brief 
 *       TSIP cross core staggering. If TSIP_CORE_STAGGER is defined the
 *       super-frame size will set to half the value in the configuration, and
 *       even cores and odd cores will have a different half phase. 
 * 
 *      Currently this works only with a super-frame size defined as 8
 *      for all ports. If the value is not 8, tsip_ERR_CONFIG will 
 *      be returned during the initialization time.
 */
//#define TSIP_CORE_STAGGER

/**
 *  @def  TSIP_FOR_LOOP_STAGGER_DELAY
 *        This value is placed into a for loop during core synchronization:
 *
 *        volatile unsigned int x;
 *        for (x = 0; x < TSIP_FOR_LOOP_STAGGER_DELAY; x++);
 *
 *  @brief When a channels DMA is disabled the super-frame interrupts
 *         stop as well. The delay puts some time between when the DMA is disabled
 *         and then re-enabled. If the value is too short then the disable-delay-enable
 *         has no effect. The amount of time required actually varies based on
 *         VBUS load. Smaller values (~100) work for the first couple of cores,
 *         but fail when more cores are downloaded. A value that is too long will
 *         also cause problems because the DMA will run past the current frame
 *         and the synchronization will start in the wrong buffer, causing the
 *         sync to retry indefinitely. A value of 7500 was found experimentally,
 *         but can be changed.
*/
#define TSIP_FOR_LOOP_STAGGER_DELAY  7500

/**
 *  @def  TSIP_APPBUF_PACKED
 *        Define to select input/output data type of TSIP. If TSIP_APPBUF_PACKED
 *        is defined, 8-bit data will be used in application for TSIP data transfer.
 *        Otherwise, 16-bit data will be used.
 */
//#define TSIP_APPBUF_PACKED


/**
 *  @def  TSIP_STATIC_DMA_BUF
 *        Define to use static bufffer allocation
 *        
 *        
 */
#define TSIP_STATIC_DMA_BUF


#ifdef __cplusplus
}
#endif
  

#endif  /* _TSIP_CFG_H */
