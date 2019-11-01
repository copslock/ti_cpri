/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 */

/** ============================================================================
 *  @file       IOLINK_v0.h
 *
 *  @brief      IO-Link Master driver implementation for IO-Link PRU firmware.
 *
 *  The IOLINK header file should be included in an application as follows:
 *  @code
 *  #include <ti/drv/iolink/IOLINK.h>
 *  #include <ti/drv/iolink/src/v0/IOLINK_v0.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef IOLINK_V0_H
#define IOLINK_V0_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <ti/csl/src/ip/icss/V0/cslr_icssm_iep.h>
#include <ti/csl/src/ip/icss/V0/cslr_icssm_intc.h>
#include <ti/csl/src/ip/timer/V1/hw_timer.h>

#include <ti/drv/iolink/IOLINK.h>
#include <ti/drv/iolink/src/IOLINK_osal.h>
#include <ti/drv/pruss/pruicss.h>

/** ===========================================================================
 *
 * @defgroup IOLINK_V0_API IOLINK v0 driver API
 * @ingroup IOLINK_V0_API
 *
 * ============================================================================
 */
/**
@defgroup IOLINK_V0_DATASTRUCT  IOLINK v0 driver Data Structures
@ingroup IOLINK_V0_API
*/
/**
@defgroup IOLINK_V0_FUNCTION  IOLINK v0 driver Functions
@ingroup IOLINK_V0_API
*/
/**
@defgroup IOLINK_V0_ENUM IOLINK  IOLINK v0 driver Enumerated Data Types
@ingroup IOLINK_V0_API
*/

/** ===========================================================================
 *  @addtogroup IOLINK_V0_ENUM
    @{
 * ============================================================================
 */

/*!
 *  @brief Max number of channels per IO-Link instance
 */
#define IOLINK_MAX_NUM_CHN              8U

/*!
 * This command sets the stack callback functions to the driver.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of callback functions structure
 */
#define IOLINK_CTRL_SET_CALLBACKS     (0U)

/*!
 * IOLINK control to send user specificed commands to the PRU. This IOCTL returns
 * IOLINK_STATUS_SUCCESS if the command is sent successfully. Returns IOLINK_STATUS_ERROR
 * otherwise.
 * The argument to this IOCTL is the uint32_t * of the channel, command, commard arg
 */
#define IOLINK_CTRL_SEND_CMD          (1U)

/*!
 * This command sets the TX/RX buffer for send/receive request.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of channel, txBufLen, rxBufLen, txBuf, rxBuf.
 */
#define IOLINK_CTRL_SET_XFER_BUFFER   (2U)

/*!
 * This command starts a new TX/RX data cycle.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of channel
 */
#define IOLINK_CTRL_START_XFER        (3U)

/*!
 * This command starts a timer.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of channel, timer type, delay type, delay.
 */
#define IOLINK_CTRL_START_TIMER       (4U)

/*!
 * This command stops a timer.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of timer type.
 */
#define IOLINK_CTRL_STOP_TIMER        (5U)

/*!
 * This command sets the delay time of a cycle timer.
 * This command returns IOLINK_STATUS_SUCCESS if successful.
 * Returns IOLINK_STATUS_ERROR otherwise.
 * The arguments to this IOCTL is the uint32_t * of channel, timer delay
 */
#define IOLINK_CTRL_SET_CYCLE_TIMER   (6U)

/*!
 *  @brief IO-Link handle definitions
 */
#define IOLINK_HANDLE_COUNT             (1U)
#define IOLINK_HANDLE_ICSS0_PRU0        (0U)
#define IOLINK_HANDLE_ICSS0_PRU1        (1U)
#define IOLINK_HANDLE_ICSS1_PRU0        (2U)
#define IOLINK_HANDLE_ICSS1_PRU1        (3U)

/*!
 *  @brief IO-Link commands
 */
#define IOLINK_COMMAND_STARTPULSE       (0U)
#define IOLINK_COMMAND_SETCOM           (1U)

/*!
 *  @brief IO-Link timer types
 */
#define IOLINK_TIMER_TYPE_10MS          (0U)
#define IOLINK_TIMER_TYPE_ADJ           (1U)
#define IOLINK_TIMER_TYPE_CYCLE         (2U)

/*!
 *  @brief IO-Link adjuster timer types
 */
#define IOLINK_TIMER_ADJ_TREN           (0U)
#define IOLINK_TIMER_ADJ_TDMT           (1U)

/* @} */

/**
 *  \addtogroup IOLINK_V0_DATASTRUCT
 *  @{
 */

/*!
 *  @brief union structure for 8-bit and 32-bit conversion
 */
typedef union IOLINK_dataConverter_s
{
    uint8_t     *data8bit;
    uint32_t    *data32bit;
} IOLINK_dataConverter;

/*!
 *  @brief IO-Link PRU ICSS config data structure
 */
typedef struct IOLINK_PruIcssConfig_s
{
    /*! ICSS instance number (0 or 1)      */
    uint32_t       icssNum;
    /*! PRU instance number (0 or 1)       */
    uint32_t       pruNum;
    /*! PRU data memory 0                  */
    uint32_t       dataMem0;
    /*! PRU data memory 1                  */
    uint32_t       dataMem1;
    /*! PRU instruction memory             */
    uint32_t       instMem;
    /*! PRU data memory base address       */
    uint32_t       dataMemBaseAddr;
    /*! PRU intc base address       */
    uint32_t       intcBaseAddr;

} IOLINK_PruIcssConfig;

/*!
 *  @brief IO-Link PRU ICSS HWI attributes
 */
typedef struct IOLINK_PruIcssHwiAttrs_s
{
    /*! SoC core interrupt number      */
    uint32_t coreIntNum;
    /*! SoC event ID                   */
    uint32_t socEvId;
    /*! interrupt priority             */
    uint32_t intPriority;

} IOLINK_PruIcssHwiAttrs;

/*!
 *  @brief  IOLINK_v0 PRU software IP attributes
 */
typedef struct IOLINK_v0_SwAttrs_s
{
    /*! ICSS PRU firmware version                       */
    uint32_t                  version;
    /*! ICSS PRU configuration                          */
    IOLINK_PruIcssConfig      pruIcssConfig;
    /*! HWI config attrs of IO-Link cycle counter       */
    IOLINK_PruIcssHwiAttrs    cycleCounterIntConfig;
    /*! HWI config attrs of IO-Link adjustable timer    */
    IOLINK_PruIcssHwiAttrs    adjustableTimerIntConfig;
    /*! HWI config attrs of IO-Link PRU complete        */
    IOLINK_PruIcssHwiAttrs    pruCompleteIntConfig;
    
} IOLINK_v0_SwAttrs;

/*!
 *  @brief IO-Link channel specific 100us cycle timer configuration
 */
typedef struct IOLINK_cycleTimerConfig_s
{
    bool        enable[IOLINK_MAX_NUM_CHN];
    uint32_t    timer[IOLINK_MAX_NUM_CHN];
    uint32_t    delay[IOLINK_MAX_NUM_CHN];
} IOLINK_cycleTimerConfig;

/*!
 *  @brief IO-Link channel specific 10ms software timer configuration
 */
typedef struct IOLINK_swTimerConfig_s
{
    bool        enable[IOLINK_MAX_NUM_CHN];
    uint32_t    timerCnt;
    uint32_t    timer;
    uint32_t    timerDiv;
} IOLINK_swTimerConfig;

/*!
 *  @brief IO-Link adjustable timer configuration
 *
 *         Structure storing the high precision hardware timer configuration
 *         (used for start up sequence timings)
 */
typedef struct IOLINK_Adjustable_TimerConfig_s
{
    uint32_t     timerType;
    uint32_t     activeChannel;
} IOLINK_AdjTimerConfig;

/*!
 *  @brief IO-Link channel specific timer configuration
 */
typedef struct IOLINK_TimerConfig_s
{
    IOLINK_cycleTimerConfig         cycleTimer;
    IOLINK_swTimerConfig            swTimer;
    IOLINK_AdjTimerConfig           adjTimer;
} IOLINK_TimerConfig;

/*!
 *  @brief IO-Link channel specific receive buffer configuration
 */
typedef struct IOLINK_RxBufConfig_s
{
    uint8_t *rxBufAddr[IOLINK_MAX_NUM_CHN];
    uint8_t  rxBufLen[IOLINK_MAX_NUM_CHN];
} IOLINK_RxBufConfig;

/*!
 *  @brief      The definition of a callback function for cycle timer
 *
 *  @param      IOLINK_Handle           IO-Link handle #
 *
 *  @param      channel                 IO-Link channel #
 *
 */
typedef void (*IOLINK_cycleTimerCallbackFxn)(IOLINK_Handle handle, uint32_t channel);

/*!
 *  @brief      The definition of a callback function for software timer
 *
 *  @param      IOLINK_Handle           IO-Link handle #
 *
 *  @param      channel                 IO-Link channel #
 *
 */
typedef void (*IOLINK_swTimerCallbackFxn)(IOLINK_Handle handle, uint32_t channel);

/*!
 *  @brief      The definition of a callback function for adjustable timer
 *
 *  @param      IOLINK_Handle           IO-Link handle #
 *
 *  @param      channel                 IO-Link channel #
 *
 *  @param      uint32_t                timer delay type
 *
 */
typedef void (*IOLINK_adjTimerCallbackFxn)(IOLINK_Handle handle, uint32_t channel, uint32_t delayType);

/*!
 *  @brief      The definition of a callback function for data transfer
 *
 *  @param      IOLINK_Handle           IO-Link handle #
 *
 *  @param      channel                 IO-Link channel #
 *
 */
typedef void (*IOLINK_xferRspCallbackFxn)(IOLINK_Handle handle, uint32_t channel);

/*!
 *  @brief      The definition of a callback function for data transfer error
 *
 *  @param      IOLINK_Handle           IO-Link handle #
 *
 *  @param      channel                 IO-Link channel #
 *
 */
typedef void (*IOLINK_xferErrRspCallbackFxn)(IOLINK_Handle handle, uint32_t channel);

/*!
 *  @brief  IOLINK callback function pointers data structure
 */
typedef struct IOLINK_v0_Callbacks_s
{
    /*! cycle timer callback function pointer */
    IOLINK_cycleTimerCallbackFxn cycleTimerCallback;
    /*! 10 msec timer callbackFxn function pointer */
    IOLINK_swTimerCallbackFxn    swTimerCallback;
    /*! adjustable timer callback function pointer */
    IOLINK_adjTimerCallbackFxn   adjTimerCallback;
    /*! data transfer response callback function pointer */
    IOLINK_xferRspCallbackFxn    xferRspCallback;
    /*! data transfer error response callback function pointer */
    IOLINK_xferErrRspCallbackFxn xferErrRspCallback;

} IOLINK_v0_Callbacks;

/*!
 *  @brief  IOLINK_v0 Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct IOLINK_v0_Object_s
{
    /*! ICSS PRU handle */
    PRUICSS_Handle        pruIcssHandle;
    /*! Saved IO-Link params */
    IOLINK_Params         iolinkParams;

    /*! Cycle timer h/w interrupt handler */
    void                 *cycleCounterHwi;
    /*! Adjustable timer h/w interrupt handler */
    void                 *adjustableTimerHwi;
    /*! PRU transfer complete h/w interrupt handler */
    void                 *pruCompleteHwi;

    /*! Software timer s/w interrupt handler */
    void                 *softwareTimerSwi;
    /*! Receive enable delay adjustable timer s/w interrupt handler */
    void                 *tRenTimerSwi;
    /*! Master message delay adjusable timer s/w interrupt handler */
    void                 *tDMTTimerSwi;

    /*! cycle timer timeout s/w interrupt handler */
    void                 *cycleTimerElapsedSwi[IOLINK_MAX_NUM_CHN];
    /*! PRU transfer complete s/w interrupt handler */
    void                 *pruCompleteSwi[IOLINK_MAX_NUM_CHN];

    /*! Timer configuration data */
    IOLINK_TimerConfig    timerConfig;
    /*! RX buffer configuration data */
    IOLINK_RxBufConfig    rxBufConfig;

    /*! callback function pointer data */
    IOLINK_v0_Callbacks   callbacks;

    /*! flag to indicate PRU instance is open */
    bool                  isOpen;

} IOLINK_v0_Object, *IOLINK_v0_Handle;

/* @} */

/**
 *  \addtogroup IOLINK_V0_FUNCTION
 *  @{
 */

/* IOLINK function table pointer */
extern const IOLINK_FxnTable IOLINK_v0_FxnTable;

/* SoC specific functions */
extern void IOLINK_pruIcssPinMuxCfg(void);

/* TBD: need to move to osal */
extern void IOLINK_cTimerInit(void);
extern void IOLINK_adjustableTimerInit(void);
extern void IOLINK_adjustableTimerStart(uint32_t compare);
extern void IOLINK_adjustableTimerStop(void);
extern void IOLINK_clearCycleTimerInt(void);
extern void IOLINK_clearAdjTimerInt(void);
extern void IOLINK_clearPruCompInt(void);

extern void IOLINK_registerSwi(uintptr_t arg0, uintptr_t arg1, void *isrFnPtr, void *swi);
extern void IOLINK_destructSwi(void *swi);
extern void IOLINK_postSwi(void *swi);

/* @} */

#ifdef __cplusplus
}
#endif

#endif /* IOLINK_V0_H */
