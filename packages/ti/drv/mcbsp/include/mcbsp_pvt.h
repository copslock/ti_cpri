/*
 * mcbsp_pvt.h
 *
 * McBSP Driver internal header file
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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
/**
 *  \file     mcbsp_pvt.h
 *
 *  \brief    Header file conataining the driver internal definitions and data
 *            structures.
 *
 *            (C) Copyright 2012, Texas Instruments, Inc
 *
 */

#ifndef _MCBSP_PVT_H_
#define _MCBSP_PVT_H_


/*============================================================================*/
/*                        INCLUDE FILES                                       */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* MCBSP LLD includes */
#include <ti/csl/cslr_mcbsp.h>
#include <ti/sdo/edma3/drv/edma3_drv.h>
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/csl/cslr_device.h>

/*============================================================================*/
/*                            MACRO DEFINTIONS                                */
/*============================================================================*/

#define Mcbsp_MAXLINKCNT               (2u)
/**<  Maximum number of EDMA jobs linked at a time (Must be 2).               */

#define Mcbsp_TXEVENTQUE               (1u)
/**<  Transmit EDMA channel event queue number                                */

#define Mcbsp_RXEVENTQUE               (2u)
/**<  Receive EDMA channel event queue number                                 */

#define Mcbsp_CNT_MAX_VAL              (0xFFFFu)
/**<  Max possible value of aCnt, bCnt and cCnt                               */

#define Mcbsp_STATUS_INVALID           (0xFFFF)
/**<  Generic invalidate status                                               */

#define Mcbsp_STATUS_VALID             (0x1)
/**<  Generic validate status                                                 */

#define Mcbsp_MAX_IOBUF_SIZE           (32768u)
/**< Maximum sixe of the I/O buffer programmable                              */

#define Mcbsp_FRAME_LENGTH             (127u)
/**< No of frames Max supported by the mcbsp                                  */

#ifndef CSL_MCBSP_PER_CNT
#define CSL_MCBSP_PER_CNT CSL_MCBSP_CNT
#endif
/*============================================================================*/
/*                         ENUMERATED DATA TYPES                              */
/*============================================================================*/

/**
 *  \brief Mcbsp driver state
 *
 *   Mcbsp driver state enums used to track the driver and channel state.
 */
typedef enum Mcbsp_DriverState_t
{
    Mcbsp_DriverState_DELETED,
    Mcbsp_DriverState_CREATED,
    Mcbsp_DriverState_INITIALIZED,
    Mcbsp_DriverState_OPENED,
    Mcbsp_DriverState_CLOSED,
    Mcbsp_DriverState_DEINITIALIZED
}Mcbsp_DriverState;
/**< Mcbsp driver and channel states                              */

/**
 * \brief Mcbsp SPCR control enums
 *
 *  These enums are used to control the settings of the SPCR register.
 */
typedef enum Mcbsp_SpcrCtrl_t
{
    Mcbsp_SpcrCtrl_RX_ENABLE   = (1u),
    /**< To enable receiver in resetControl Function              */

    Mcbsp_SpcrCtrl_TX_ENABLE   = (2u),
    /**< To enable Transmitter in resetControl Function           */

    Mcbsp_SpcrCtrl_RX_DISABLE  = (4u),
    /**< To disable Receiver in resetControl Function             */

    Mcbsp_SpcrCtrl_TX_DISABLE  = (8u),
    /**< To disable Transmitter in resetControl Function          */

    Mcbsp_SpcrCtrl_FSYNC_ENABLE = (16u),
    /**< To enable Frame Sync Generation in resetControl Function */

    Mcbsp_SpcrCtrl_SRG_ENABLE   = (32u),
    /**< To enable Sample Rate Generator in resetControl Function */

    Mcbsp_SpcrCtrl_FSYNC_DISABLE = (64u),
    /**< To disable Frame Sync Generation in resetControl Function*/

    Mcbsp_SpcrCtrl_SRG_DISABLE = (128u)
    /**< To disable Sample Rate Generator in resetControl Function*/
}Mcbsp_SpcrCtrl;
/**< Mcbsp SPCR control enums                                     */

/*============================================================================*/
/*                              DATA STRUCTURES                               */
/*============================================================================*/

typedef struct Mcbsp_Object_t Mcbsp_Object;

/**
 * \brief Mcbsp channel Object
 *
 *  This structure maintains the current channel state. It also holds
 *  information on DMA channel being used and holds the application
 *  callback function to be called in case of an interrupt.
 *
 *  This structure is initialized by mdCreateChan and a pointer to this
 *  is passed down to all other channel related functions. Lifetime of
 *  the data structure is from its creation by mdCreateChan till it
 *  is invalidated by mdDeleteChan.each instance object will have two channel
 *  object one for TX and one for the RX channel.
 */
typedef struct Mcbsp_ChannelObj_t
{
    uint16_t                    mode;
    /**< Mode of operation of the channel (MCBSP_MODE_INPUT or MCBSP_MODE_OUTPUT)           */

    Mcbsp_DriverState           chanState;
    /**< operational state of the channel (opened/closed)                     */

    Mcbsp_Object               *devHandle;
    /**< Pointer to McBSP device driver instance object                       */

    Mcbsp_CallbackFxn           cbFxn;
    /**< Driver call back function to be called once a I/O buffer is processed*/

    void*                       cbArg;
    /**< Callback Function argument                                           */

    void*                       edmaHandle;
    /**< Handle to the EDMA3 driver(given by application)                     */

    uint32_t                    edmaEventQue;
    /**< EDMA event queue to be used by the channel                           */

    EDMA3_RM_TccCallback        edmaCallback;
    /**< Edma callback function pointer                                       */

    uint32_t                    xferChan;
    /**< eDMA  ChannelID                                                      */

    uint32_t                    tcc ;
    /**< EDMA transfer completion code                                        */

    uint32_t                    pramTbl[Mcbsp_MAXLINKCNT];
    /**< Logical channel numbers of EDMA, which are used for linking          */

    uint32_t                    pramTblAddr[Mcbsp_MAXLINKCNT];
    /**<  Physical address of logical channel numbers of EDMA, which          *
     * are used for linking                                                   */

    void*                       ptrQPendList;
    /**< Queue to hold the pending I/O buffers received from the application  */

    void*                       ptrQFloatList;
    /**< Queue to manage floating I/O buffers in DMA                          */

    Mcbsp_IOBuf                *tempIOBuf;
    /**< Temporary I/O buffer holder                                          */

    Mcbsp_IOBuf                *dataIOBuf;
    /**< Temporary I/O buffer holder used to load the next buffer in to EDMA  */

    uint32_t                    submitCount;
    /**< Number of submit calls pending in the driver                         */

    Mcbsp_BufferFormat          dataFormat;
    /**< Application supplied buffer format                                   */

    Bool                        nextFlag;
    /**< Flag to indicate if the state machine can be stopped or not          */

    volatile Bool               bMuteON;
    /**< Flag to set the mute ON/OFF status                                   */

    volatile Bool               paused;
    /**< Flag to indicate if the audio is paused or not                       */

    volatile Bool               flush;
    /**< Flag to indicate if the Driver IO request flush is set               */

    volatile Bool               isTempIOBufValid;
    /**< Flag to indicate whether a valid buffer is available in tempIOBuf    */

    Bool                        enableHwFifo;
    /**< whether the FIFO has to be enabled for this channel                  */

    Mcbsp_GblErrCallback        gblErrCbk;
    /**< Callback to called in case an error occurs(supplied by application)  */

    uint32_t                    userDataBufferSize;
    /**< Size of the data buffer to be transferred                            */

    void*                       loopJobBuffer;
    /**< Buffer to be transferred when the loop job is running                */

    uint16_t                    loopJobLength;
    /**<Length of userloop job for each serialiser                            */

    uint32_t                    userLoopJobLength;
    /**< user specified loop job length to be used if supplied                */

    uint32_t                    nextLinkParamSetToBeUpdated;
    /**<  Used to store the next index of link param to be updated            */

    volatile Bool               loopjobUpdatedinParamset;
    /**< Used to prevent updating second paramset with loopjob for            *
     * last dataIOBuf edma callback                                           */

    uint16_t                    roundedWordWidth;
    /**< Word length bytes to be transferred for DMA transaction              */

    uint16_t                    currentDataSize;
    /**< data buffer size of the currently transferring buffer                */

    Mcbsp_DataConfig            chanConfig;
    /**< settings to configure the TX or RX hardware sections                 */

    Mcbsp_ClkSetup              clkSetup;
    /**< clock setup for the RX or the TX section                             */

    Bool                        userLoopJob;
    /**< Option to indicate if the user loop job is used or driver loop job   */

    int32_t                     currentError;
    /**< Current I/O buffer error status                                      */

    uint32_t                   numEnabledChannels;
    /**<  Number of channels enabled with multichannel mode                   */

}Mcbsp_ChannelObj;
/**< Mcbsp channel Object                                                     */

/**
 * \brief   Mcbsp instance Object
 *
 * \note    This data structure holds the information pertaining to an instance
 *          of the Mcbsp device.it holds information like the current device
 *          state, handle to the McBSP channels. The data structure is
 *          initialized during "mdBindDev", which is called during DSP-BIOS
 *          initialization, and is persistent till it is invalidated by
 *          "mdUnBindDev".
 */
typedef struct
{
    Bool                        inUse;
    /**< Variable to indicate if the instance is in use or not                */

    int32_t                     instNum;
    /**< Instance number of the current instance                              */

    Mcbsp_DriverState           devState;
    /**< operational state of the driver (created/deleted)                    */

    Mcbsp_DevMode               mode;
    /**< Operating mode of the Mcbsp driver(Mcbsp)                            */

    Mcbsp_OpMode                opMode;
    /**< Operational mode of the driver(INT/DMA/POLLING)                      */

    Bool                        enablecache;
    /**< Buffer operations to be performed by the driver or not               */

    Mcbsp_HwInfo_Unpadded       hwInfo;
    /**< McBSP handle for initial port configuration                          */

    Bool                        stopSmFsXmt;
    /**< TX state machine status(Stopped/Running)                             */

    Bool                        stopSmFsRcv;
    /**< RX state machine status(Stopped/Running)                             */

    Mcbsp_ChannelObj            xmtObj;
    /**< Transmit channel object                                              */

    Mcbsp_ChannelObj            rcvObj;
    /**< Receive channel object                                               */

    Mcbsp_srgConfig             srgrConfig;
    /**< configuration settings for the Sample rate generator                 */

    Bool                        txSrgEnable;
    /**< Flag to indicate if the TX section needs the sample rate generator to*
     * be enabled                                                             */

    Bool                        rxSrgEnable;
    /**< Flag to indicate if the RX section needs the sample rate generator to*
     * be enabled                                                             */
    
    Bool                        srgConfigured;
    /**< Flag to indicate if the SRGR settings are configured or not          */

    volatile Bool               srgEnabled;
    /**< Flag to indicate if the sample rate generator is enabled and running */

    Bool                        txFsgEnable;
    /**< Flag to indicate if the TX section needs the frame sync generator to *
     * be enabled                                                             */

    Bool                        rxFsgEnable;
    /**< Flag to indicate if the RX section needs the frame sync generator to *
     * be enabled                                                             */

    Bool                        fsgConfigured;
    /**< Flag to indicate if the Framesync generator is configured or not     */
    
    volatile Bool               fsgEnabled;
    /**< Flag to indicate if the frame sync generator is enabled and running  */

    uint32_t                    retryCount;
    /**< The retry count value to be used when waiting for the TX empty to be *
     * set                                                                    */

    Bool                        loopJobMode;
    /**< Variable to check if the loop job mode is enabled or not             */

}Mcbsp_Object_Unpadded;

struct Mcbsp_Object_t
{
    /** Data structure without padding, so sizeof() can compute padding */
    Mcbsp_Object_Unpadded obj;
    /** Pad out to end of MCBSP_MAX_CACHE_ALIGN bytes to prevent something else
     * from being placed on same cache line as Mcbsp_Object. Note that pad[0]
     * is illegal, so must add full MCBSP_MAX_CACHE_ALIGN if structure is
     * already padded by chance. */
    uint8_t                 pad[MCBSP_MAX_CACHE_ALIGN - 
                            (sizeof(Mcbsp_Object_Unpadded) % MCBSP_MAX_CACHE_ALIGN)];
};
/**< Mcbsp instance Object                                                    */


int32_t Mcbsp_localResetCtrl(Mcbsp_Object_Unpadded *instHandle, uint32_t selectMask);

void Mcbsp_localCompleteCurrentIo(Mcbsp_ChannelObj *chanHandle);

void Mcbsp_localEdmaCallback(uint32_t tcc, EDMA3_RM_TccStatus status, void* data);

void Mcbsp_localAbortReset(Mcbsp_ChannelObj *chanHandle);

void Mcbsp_localGetNextIndex(uint32_t *index);

int32_t Mcbsp_localSetupEdmaDuringOpen(Mcbsp_ChannelObj *chanHandle);

int32_t Mcbsp_localSubmitIoctl(Mcbsp_ChannelObj *chanHandle,
                             Mcbsp_IOCTL       cmd,
                             void*               arg,
                             void*               param);

int32_t Mcbsp_localUpdtDtPktToLnkPrms(Mcbsp_ChannelObj *chanHandle,
                                    Mcbsp_IOBuf       *const ioBuf);

int32_t Mcbsp_localEdmaChanPaRAMsetup(Mcbsp_ChannelObj *chanHandle);

int32_t Mcbsp_localEdmaProcessPkt(Mcbsp_ChannelObj *chanHandle,
                                Mcbsp_IOBuf       *ioBuf);

void Mcbsp_localCancelAndAbortAllIo(Mcbsp_ChannelObj *chanHandle);

void Mcbsp_localAbortReset(Mcbsp_ChannelObj *chanHandle);

int32_t Mcbsp_localGetIndicesSyncType(Mcbsp_ChannelObj   *chanHandle,
                                    volatile int16_t     *bIndex,
                                    volatile int16_t     *cIndex,
                                    volatile uint16_t    *aCnt,
                                    volatile uint16_t    *bCnt,
                                    volatile uint16_t    *cCnt,
                                    EDMA3_DRV_SyncType *syncType,
                                    Bool                forLoopJobBuf);

void Mcbsp_localGetNextIndex(uint32_t *index);
int32_t Mcbsp_localConfigureSrgr(Mcbsp_Object_Unpadded *instHandle,
                               Mcbsp_ChannelObj *chanHandle);
int32_t Mcbsp_localConfigureRcvChannel(Mcbsp_Object_Unpadded *instHandle,
                                     Mcbsp_ChanParams *params);
int32_t Mcbsp_localConfigureXmtChannel(Mcbsp_Object_Unpadded *instHandle,
                                     Mcbsp_ChanParams *params);
void Mcbsp_localLoadPktToEdma(Mcbsp_ChannelObj *chanHandle,Mcbsp_IOBuf *ioBuf); 
int32_t Mcbsp_localModifySampleRate(Mcbsp_ChannelObj *chanHandle,void* arg);

#ifndef MCBSP_LOOPJOB_ENABLE
void Mcbsp_TxFifo(int32_t arg0,int32_t arg1);
#endif /* MCBSP_LOOPJOB_ENABLE */

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _MCBSP_PVT_H_ */
/*============================================================================*/
/*                         END OF FILE                                        */
/*============================================================================*/
