/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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

/** ============================================================================
 *   @file  aif2fl_getHwStatusAux.h
 *
 *   @brief  API Auxilary header file for Antenna Interface 2 set HW status
 *
 */

/* =============================================================================
 * Revision History
 * ===============
 *  03-Jun-2009 Albert  File Created.
*  06-June-2015  Seb      File imported in the driver
 *
 * =============================================================================
 */

#ifndef _AIF2FLGETHWSTATUSAUX_H_
#define _AIF2FLGETHWSTATUSAUX_H_
 
#include <ti/drv/aif2/aif2fl.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  Get Hardware Status Functions of Antenna Interface 2
 */

/** ============================================================================
 *   @n@b Aif2Fl_getVersion
 *
 *   @b Description
 *   @n This function returns AIF2 version 
 *
 *   @b Arguments
 *   @verbatim

            hAif2        Handle to the aif2 instance
            version     Pointer to get the version instance.

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.

 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_AIF2_PID_MINOR,AIF2_AIF2_PID_CUSTOM,AIF2_AIF2_PID_MAJOR,
 *        AIF2_AIF2_PID_RTL,AIF2_AIF2_PID_FUNC,AIF2_AIF2_PID_SCHEME
 *   @b Example
 *   @verbatim
        Aif2Fl_PidStatus   version;
        Aif2Fl_getVersion (hAif2,  &version);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getVersion (Aif2Fl_Handle   hAif2, Aif2Fl_PidStatus*   version)
{
	uint32_t tmpReg;
	tmpReg = hAif2->regs->AIF2_PID;	
	version->minor = CSL_FEXT(tmpReg, AIF2_AIF2_PID_MINOR);
 	version->custom = CSL_FEXT(tmpReg, AIF2_AIF2_PID_CUSTOM);
	version->major = CSL_FEXT(tmpReg, AIF2_AIF2_PID_MAJOR);
	version->RTL = CSL_FEXT(tmpReg, AIF2_AIF2_PID_RTL);
 	version->func = CSL_FEXT(tmpReg, AIF2_AIF2_PID_FUNC);
	version->scheme = CSL_FEXT(tmpReg, AIF2_AIF2_PID_SCHEME);
}


/** ============================================================================
 *   @n@b Aif2Fl_getVcStat
 *
 *   @b Description
 *   @n This function return the status of VC emu status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance 
                       
     @endverbatim
 *
 *   <b> Return Value </b>  uint16_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_VC_STAT_EMU_HALT
 *        
 *
 *   @b Example
 *   @verbatim
        uint16_t emu_sts;
        emu_sts = Aif2Fl_getVcStat (hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline
uint16_t Aif2Fl_getVcStat (
        Aif2Fl_Handle   hAif2
)
{
	return CSL_FEXT(hAif2->regs->VC_STAT, AIF2_VC_STAT_EMU_HALT);
}


/** ============================================================================
 *   @n@b Aif2Fl_getSdRxLinkStatus
 *
 *   @b Description
 *   @n This function return the status of SD Rx link status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance
           Aif2Fl_SdRxStatus*       pSdRxStat          
     @endverbatim
 *
 *   <b> Return Value </b> void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_SD_RX_STS_RXSYNC,AIF2_SD_RX_STS_RXLOSS_OF_SIGNAL,
 *         AIF2_SD_RX_STS_RXADAPT_DONE
 *   @b Example
 *   @verbatim
        Aif2Fl_SdRxStatus       SdRxStat;
        .....
        Aif2Fl_getSdRxLinkStatus (hAif2, &SdRxStat );
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getSdRxLinkStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_SdRxStatus*       pSdRxStat
)
{
    pSdRxStat->sdRxSync = CSL_FEXT(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_STS,AIF2_SD_RX_STS_RXSYNC);
    pSdRxStat->sdRxLosDetect = CSL_FEXT(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_STS,AIF2_SD_RX_STS_RXLOSS_OF_SIGNAL);
#ifdef K2
    pSdRxStat->sdRxAdaptDone = CSL_FEXT(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_STS,AIF2_SD_RX_STS_RXADAPT_DONE);
#endif
}

#ifndef K2
/** ============================================================================
 *   @n@b Aif2Fl_getSdTxLinkStatus
 *
 *   @b Description
 *   @n This function return the status of SD Tx link status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance
           Aif2Fl_SdTxStatus*       pSdTxStat
     @endverbatim
 *
 *   <b> Return Value </b> void
 *   <b> Pre Condition </b>
  *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_SD_TX_STS_TXBUS_BANDWIDTH,AIF2_SD_TX_STS_TXTEST_FAILURE
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_SdTxStatus       SdTxStat;
        .....
        Aif2Fl_getSdTxLinkStatus (hAif2, &SdTxStat );
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getSdTxLinkStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_SdTxStatus*       pSdTxStat
)
{
    pSdTxStat->sdTxBusWidth= CSL_FEXT(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_STS,AIF2_SD_TX_STS_TXBUS_BANDWIDTH);
    pSdTxStat->sdTxTestFail= CSL_FEXT(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_STS,AIF2_SD_TX_STS_TXTEST_FAILURE);
}
#endif

/** ============================================================================
 *   @n@b Aif2Fl_getSdB8PllLock
 *
 *   @b Description
 *   @n This function return the status of B8 PLL lock 
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance 
                       
     @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_SD_PLL_B8_STS_B8PLL_LOCK
 *        
 *
 *   @b Example
 *   @verbatim
        uint8_t lock_sts;
        lock_sts = Aif2Fl_getSdB8PllLock (hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getSdB8PllLock (
        Aif2Fl_Handle   hAif2
)
{
	return CSL_FEXT(hAif2->regs->SD_PLL_B8_STS, AIF2_SD_PLL_B8_STS_B8PLL_LOCK);
}


/** ============================================================================
 *   @n@b Aif2Fl_getSdB4PllLock
 *
 *   @b Description
 *   @n This function return the status of B4 PLL lock 
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance 
                       
     @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_SD_PLL_B4_STS_B4PLL_LOCK
 *        
 *
 *   @b Example
 *   @verbatim
        uint8_t lock_sts;
        lock_sts = Aif2Fl_getSdB4PllLock (hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getSdB4PllLock (
        Aif2Fl_Handle   hAif2
)
{
	return CSL_FEXT(hAif2->regs->SD_PLL_B4_STS, AIF2_SD_PLL_B4_STS_B4PLL_LOCK);
}


/** ============================================================================
 *   @n@b Aif2Fl_getRmLinkStatus0
 *
 *   @b Description
 *   @n This function gets the  RM  link Status 0
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           Aif2Fl_RmStatus0       Pointer to the RM link Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RM_LK_STS0_SYNC_STATUS,AIF2_RM_LK_STS0_LOS,AIF2_RM_LK_STS0_NUM_LOS_DET,
 *        AIF2_RM_LK_STS0_LOC,AIF2_RM_LK_STS0_FIFO_OVF
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_RmStatus0     RmStat;
         hAif2->arg_link = 1; //get link1 status
        Aif2Fl_getRmLinkStatus0 (hAif2, &RmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRmLinkStatus0 (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RmStatus0*    pRmStat
)
{   
    pRmStat->rmSyncStatus = (Aif2Fl_RmSyncState)CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS0,AIF2_RM_LK_STS0_SYNC_STATUS);
    pRmStat->rmLos = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS0,AIF2_RM_LK_STS0_LOS);
    pRmStat->rmNumLosDetect = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS0,AIF2_RM_LK_STS0_NUM_LOS_DET);
    pRmStat->rmLoc = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS0,AIF2_RM_LK_STS0_LOC);
    pRmStat->rmFifoOverflow = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS0,AIF2_RM_LK_STS0_FIFO_OVF);
    
}


/** ============================================================================
 *   @n@b Aif2Fl_getRmLinkStatus1
 *
 *   @b Description
 *   @n This function gets the  RM  link Status 1
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           Aif2Fl_RmStatus1       Pointer to the RM link Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RM_LK_STS1_NUM_LOS,AIF2_RM_LK_STS1_LCV_CNTR_VALUE
 *        
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_RmStatus1     RmStat;
         hAif2->arg_link = 1; //get link1 status
        Aif2Fl_getRmLinkStatus1 (hAif2, &RmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRmLinkStatus1 (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RmStatus1*    pRmStat
)
{   
    pRmStat->rmNumLos = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS1,AIF2_RM_LK_STS1_NUM_LOS);
    pRmStat->rmLcvCountValue = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS1,AIF2_RM_LK_STS1_LCV_CNTR_VALUE);
    
}


/** ============================================================================
 *   @n@b Aif2Fl_getRmLinkStatus2
 *
 *   @b Description
 *   @n This function gets the  RM  link Status 2
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           Aif2Fl_RmStatus2       Pointer to the RM link Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RM_LK_STS2_CLK_QUAL,AIF2_RM_LK_STS2_SCR_VALUE
 *        
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_RmStatus2     RmStat;
         hAif2->arg_link = 1; //get link1 status
        Aif2Fl_getRmLinkStatus2 (hAif2, &RmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRmLinkStatus2 (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RmStatus2*    pRmStat
)
{   
    pRmStat->rmClockQuality = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS2,AIF2_RM_LK_STS2_CLK_QUAL);
    pRmStat->rmScrValue = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS2,AIF2_RM_LK_STS2_SCR_VALUE);
    
}

/** ============================================================================
 *   @n@b Aif2Fl_getRmLinkStatus3
 *
 *   @b Description
 *   @n This function gets the  RM  link Status 3
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           Aif2Fl_RmStatus3       Pointer to the RM link Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RM_LK_STS3_HFN,AIF2_RM_LK_STS3_BFN_LOW,AIF2_RM_LK_STS3_BFN_HIGH,
 *        AIF2_RM_LK_STS3_HFSYNC_STATE,AIF2_RM_LK_STS3_LOF_STATE
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_RmStatus3     RmStat;
         hAif2->arg_link = 1; //get link1 status
        Aif2Fl_getRmLinkStatus3 (hAif2, &RmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRmLinkStatus3 (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RmStatus3*    pRmStat
)
{   
    pRmStat->rmHfn = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS3,AIF2_RM_LK_STS3_HFN);
    pRmStat->rmBfnLow = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS3,AIF2_RM_LK_STS3_BFN_LOW);
    pRmStat->rmBfnHigh = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS3,AIF2_RM_LK_STS3_BFN_HIGH);
    pRmStat->rmHfsyncState = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS3,AIF2_RM_LK_STS3_HFSYNC_STATE);
    pRmStat->rmLofState = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS3,AIF2_RM_LK_STS3_LOF_STATE);
    
}


/** ============================================================================
 *   @n@b Aif2Fl_getRmLinkStatus4
 *
 *   @b Description
 *   @n This function gets the  RM  link Status 4
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           Aif2Fl_RmStatus4       Pointer to the RM link Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RM_LK_STS4_L1_VERSION,AIF2_RM_LK_STS4_L1_START_UP,AIF2_RM_LK_STS4_L1_RCVD_RST,
 *        AIF2_RM_LK_STS4_L1_RCVD_RAI,AIF2_RM_LK_STS4_L1_RCVD_SDI,AIF2_RM_LK_STS4_L1_RCVD_LOS,
 *        AIF2_RM_LK_STS4_L1_RCVD_LOF,AIF2_RM_LK_STS4_L1_PNTR_P
 *   @b Example
 *   @verbatim
         Aif2Fl_RmStatus4     RmStat;
         hAif2->arg_link = 1; //get link1 status
        Aif2Fl_getRmLinkStatus4 (hAif2, &RmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRmLinkStatus4 (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RmStatus4*    pRmStat
)
{   
    pRmStat->rmL1Version = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_VERSION);
    pRmStat->rmL1Startup = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_START_UP);
    pRmStat->rmL1RST = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_RCVD_RST);
    pRmStat->rmL1RAI = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_RCVD_RAI);
    pRmStat->rmL1SDI = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_RCVD_SDI);
    pRmStat->rmL1LOS = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_RCVD_LOS);
    pRmStat->rmL1LOF = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_RCVD_LOF);
    pRmStat->rmL1PointerP = CSL_FEXT(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_STS4,AIF2_RM_LK_STS4_L1_PNTR_P);
    
}


/** ============================================================================
 *   @n@b Aif2Fl_getTmCpriHfnStatus
 *
 *   @b Description
 *   @n This function return the status of TM CPRI HFN status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance. should use arg_link to select link 
                       
     @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_TM_LK_HFN_HFN
 *        
 *
 *   @b Example
 *   @verbatim
        uint8_t hfn_sts;
        hAif2->arg_link = LINK1;
        hfn_sts = Aif2Fl_getTmCpriHfnStatus (hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getTmCpriHfnStatus (
        Aif2Fl_Handle   hAif2
)
{
	return CSL_FEXT(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_HFN, AIF2_TM_LK_HFN_HFN);
}


/** ============================================================================
 *   @n@b Aif2Fl_getTmLinkStatus
 *
 *   @b Description
 *   @n This function gets the  TM  link Status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance. should use arg_link to select link
           pTmStat       Pointer to the TM link Status instance.
            
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_TM_LK_STAT_TM_FAIL,AIF2_TM_LK_STAT_FIFO_UNFL,
 *        AIF2_TM_LK_STAT_FRM_MISALIGN,AIF2_TM_LK_STAT_FRM_STATE
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_TmStatus     TmStat;
         hAif2->arg_link = 1; //get Tm link1 status
        Aif2Fl_getTmLinkStatus (hAif2, &TmStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getTmLinkStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_TmStatus*    pTmStat
)
{   
    pTmStat->tmFail = CSL_FEXT(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_STAT,AIF2_TM_LK_STAT_TM_FAIL);
    pTmStat->tmFifoUnderflow = CSL_FEXT(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_STAT,AIF2_TM_LK_STAT_FIFO_UNFL);
    pTmStat->tmFrameMisalign = CSL_FEXT(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_STAT,AIF2_TM_LK_STAT_FRM_MISALIGN);
    pTmStat->tmFrameStatus = (Aif2Fl_TmSyncState)CSL_FEXT(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_STAT,AIF2_TM_LK_STAT_FRM_STATE);
    
}


/** ============================================================================
 *   @n@b Aif2Fl_getRtFifoDepthStatus
 *
 *   @b Description
 *   @n RT Internal FIFO depth Status.
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link 
     @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RT_LK_DPTH_RT_DEPTH
 *        
 *   @b Example
 *   @verbatim
        uint8_t depth;
        hAif2->arg_link = LINK1;
        depth = Aif2Fl_getRtFifoDepthStatus (hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getRtFifoDepthStatus (
        Aif2Fl_Handle   hAif2
)
{
	return CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_DPTH, AIF2_RT_LK_DPTH_RT_DEPTH);
}


/** ============================================================================
 *   @n@b Aif2Fl_getRtHeaderStatus
 *
 *   @b Description
 *   @n This function gets the  Retransmitter  link Status
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           pRtStat       Pointer to the retransmitter header Status instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RT_HDR_ERR_DMA_CHAN,AIF2_RT_HDR_ERR_HDR_ERR
 *        
 *   @b Example
 *   @verbatim
         Aif2Fl_RtHeaderStatus     RtStat;
         hAif2->arg_link = 1; //get Rt link1 status
        Aif2Fl_getRtHeaderStatus (hAif2, &RtStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRtHeaderStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RtHeaderStatus*    pRtStat
)
{   
    pRtStat->HeaderError = CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_HDR_ERR,AIF2_RT_HDR_ERR_HDR_ERR);
    pRtStat->DmaChannel = CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_HDR_ERR,AIF2_RT_HDR_ERR_DMA_CHAN);
}


/** ============================================================================
 *   @n@b Aif2Fl_getRtLinkStatus
 *
 *   @b Description
 *   @n This function gets the  Retransmitter  link Status
 *
 *   @b Arguments
 *   @verbatim

           hAif2      Handle to the aif2 instance. should use arg_link to select link
           pRtStat       Pointer to the retransmitter link Status instance.
            
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_RT_LK_STAT_RT_HDR_ERR_STS,AIF2_RT_LK_STAT_RT_EM_STS,
 *        AIF2_RT_LK_STAT_RT_UNFL_STS,AIF2_RT_LK_STAT_RT_OVFL_STS,
 *        AIF2_RT_LK_STAT_RT_FRM_ERR_STS
 *   @b Example
 *   @verbatim
         Aif2Fl_RtStatus     RtStat;
         hAif2->arg_link = 1; //get Rt link1 status
        Aif2Fl_getRtLinkStatus (hAif2, &RtStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getRtLinkStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_RtStatus*    pRtStat
)
{   
    pRtStat->rtHeaderError = CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_STAT,AIF2_RT_LK_STAT_RT_HDR_ERR_STS);
    pRtStat->rtEmptyMessage = CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_STAT,AIF2_RT_LK_STAT_RT_EM_STS);
    pRtStat->rtFifoUnderflow = CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_STAT,AIF2_RT_LK_STAT_RT_UNFL_STS);
    pRtStat->rtFifoOverflow= CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_STAT,AIF2_RT_LK_STAT_RT_OVFL_STS);
    pRtStat->rtFrameError= CSL_FEXT(hAif2->regs->G_RT_LKS[hAif2->arg_link].RT_LK_STAT,AIF2_RT_LK_STAT_RT_FRM_ERR_STS);
}


/** ============================================================================
 *   @n@b Aif2Fl_getPdChannelStatus
 *
 *   @b Description
 *   @n This function get PD 128 Channel Status.
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           uint32_t*       Pointer to the PD channel Status instance.       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_PD_CHAN_STS_CHAN_ON
 *        
 *   @b Example
 *   @verbatim
         uint32_t     PdChanStat[4];
         
        Aif2Fl_getPdChannelStatus (hAif2, &PdChanStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getPdChannelStatus (
        Aif2Fl_Handle   hAif2,
        uint32_t*          ChanStat
)
{   
    ChanStat[0] = CSL_FEXT(hAif2->regs->PD_CHAN_STS[0],AIF2_PD_CHAN_STS_CHAN_ON);
    ChanStat[1] = CSL_FEXT(hAif2->regs->PD_CHAN_STS[1],AIF2_PD_CHAN_STS_CHAN_ON);
    ChanStat[2] = CSL_FEXT(hAif2->regs->PD_CHAN_STS[2],AIF2_PD_CHAN_STS_CHAN_ON);
    ChanStat[3] = CSL_FEXT(hAif2->regs->PD_CHAN_STS[3],AIF2_PD_CHAN_STS_CHAN_ON);
}


/** ============================================================================
 *   @n@b Aif2Fl_getPdPacketStatus
 *
 *   @b Description
 *   @n This function get PD packet status for 128 Channels.
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           uint32_t*       Pointer to the PD packet Status instance. bit 0: out packet  bit 1: in packet       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_PD_PKT_STS_CHAN_PKT
 *        
 *   @b Example
 *   @verbatim
         uint32_t     PdPacketStat[4];
         
        Aif2Fl_getPdPacketStatus (hAif2, &PdPacketStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getPdPacketStatus (
        Aif2Fl_Handle   hAif2,
        uint32_t*          PacketStat
)
{   
    PacketStat[0] = CSL_FEXT(hAif2->regs->PD_PKT_STS[0],AIF2_PD_PKT_STS_CHAN_PKT);
    PacketStat[1] = CSL_FEXT(hAif2->regs->PD_PKT_STS[1],AIF2_PD_PKT_STS_CHAN_PKT);
    PacketStat[2] = CSL_FEXT(hAif2->regs->PD_PKT_STS[2],AIF2_PD_PKT_STS_CHAN_PKT);
    PacketStat[3] = CSL_FEXT(hAif2->regs->PD_PKT_STS[3],AIF2_PD_PKT_STS_CHAN_PKT);
}


/** ============================================================================
 *   @n@b Aif2Fl_getPeChannelStatus
 *
 *   @b Description
 *   @n This function get PE 128 Channel Status.
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           uint32_t*       Pointer to the PE channel Status instance.       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_PE_CHAN_STS_CHAN_ON_STS
 *        
 *   @b Example
 *   @verbatim
         uint32_t     PeChanStat[4];
         
        Aif2Fl_getPeChannelStatus (hAif2, &PeChanStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getPeChannelStatus (
        Aif2Fl_Handle   hAif2,
        uint32_t*          ChanStat
)
{   
    ChanStat[0] = CSL_FEXT(hAif2->regs->PE_CHAN_STS[0],AIF2_PE_CHAN_STS_CHAN_ON_STS);
    ChanStat[1] = CSL_FEXT(hAif2->regs->PE_CHAN_STS[1],AIF2_PE_CHAN_STS_CHAN_ON_STS);
    ChanStat[2] = CSL_FEXT(hAif2->regs->PE_CHAN_STS[2],AIF2_PE_CHAN_STS_CHAN_ON_STS);
    ChanStat[3] = CSL_FEXT(hAif2->regs->PE_CHAN_STS[3],AIF2_PE_CHAN_STS_CHAN_ON_STS);
}


/** ============================================================================
 *   @n@b Aif2Fl_getPePacketStatus
 *
 *   @b Description
 *   @n This function get PE packet status for 128 Channels.
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance. should use arg_link to select link
           uint32_t*       Pointer to the PE packet Status instance. bit 0: out packet  bit 1: in packet       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   @n  None
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_PE_PKT_STS_PKT_STS
 *        
 *   @b Example
 *   @verbatim
         uint32_t     PePacketStat[4];
         
        Aif2Fl_getPePacketStatus (hAif2, &PePacketStat);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getPePacketStatus (
        Aif2Fl_Handle   hAif2,
        uint32_t*          PacketStat
)
{   
    PacketStat[0] = CSL_FEXT(hAif2->regs->PE_PKT_STS[0],AIF2_PE_PKT_STS_PKT_STS);
    PacketStat[1] = CSL_FEXT(hAif2->regs->PE_PKT_STS[1],AIF2_PE_PKT_STS_PKT_STS);
    PacketStat[2] = CSL_FEXT(hAif2->regs->PE_PKT_STS[2],AIF2_PE_PKT_STS_PKT_STS);
    PacketStat[3] = CSL_FEXT(hAif2->regs->PE_PKT_STS[3],AIF2_PE_PKT_STS_PKT_STS);
}


 /** ============================================================================
 *   @n@b Aif2Fl_getInDbDebugOffsetData
 *
 *   @b Description
 *   @n Get Write and Read Offset Value at address in DB_IDB_DEBUG_OFS
 *
 *   @b Arguments
 *   @verbatim
 
           hAif2            Handle to the aif2 instance     
           uint8_t*          pointer to write and read offset data 
     @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_DB_IDB_DEBUG_OFS_DAT_WOFF
 *
 *   @b Example
 *   @verbatim
        uint8_t  offset[2];
        
        Aif2Fl_getInDbDebugOffsetData(hAif2, &offset[0]);

        uint8_t write_offset = offset[0];
        uint8_t read_offset = offset[1];
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getInDbDebugOffsetData (
        Aif2Fl_Handle   hAif2,
        uint8_t*                offset
)

{ 
	
     offset[0] = CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_OFS_DAT,AIF2_DB_IDB_DEBUG_OFS_DAT_WOFF);
	 offset[1] = CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_OFS_DAT,AIF2_DB_IDB_DEBUG_OFS_DAT_ROFF);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEDbDebugData
 *
 *   @b Description
 *   @n Get Debug data written to Egress DB RAM (128 bit)
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            *uint32_t   pointer to 128 bit Debug data 

     @endverbatim
 *
 *   <b> Return Value </b>  void
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n AIF2_DB_EDB_DEBUG_D0_DATA
 *
 *   @b Example
 *   @verbatim
        uint32_t    DebugData[4];
       
        Aif2Fl_getEDbDebugData(hAif2, &DebugData[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEDbDebugData(
    Aif2Fl_Handle    hAif2,
    uint32_t*                debug_data
)
{
    debug_data[0] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_D0, AIF2_DB_EDB_DEBUG_D0_DATA);  
    debug_data[1] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_D1, AIF2_DB_EDB_DEBUG_D1_DATA);  
    debug_data[2] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_D2, AIF2_DB_EDB_DEBUG_D2_DATA);  
    debug_data[3] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_D3, AIF2_DB_EDB_DEBUG_D3_DATA);  
}
 


/** ============================================================================
 *   @n@b Aif2Fl_getEDbDebugSideData
 *
 *   @b Description
 *   @n Get Egress DB debug side band data (sop, eop, xcnt only)
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            Aif2Fl_DbSideData     Side data structure for debug

     @endverbatim
 *
 *   <b> Return Value </b>  void
 *
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n 
 *        AIF2_DB_EDB_DEBUG_SBND_SOP,AIF2_DB_EDB_DEBUG_SBND_EOP,
 *        AIF2_DB_EDB_DEBUG_SBND_XCNT,AIF2_DB_EDB_DEBUG_SBND_SYMBOL
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_DbSideData   SideData;
         
        Aif2Fl_getEDbDebugSideData(hAif2, &SideData);

        SideData.bSop = ...;
        SideData.bEop = ...;
        SideData.xcnt = ...;
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEDbDebugSideData(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DbSideData*          side_data
)
{
     side_data->bSop =     CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_SBND,AIF2_DB_EDB_DEBUG_SBND_SOP);
     side_data->bEop =     CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_SBND,AIF2_DB_EDB_DEBUG_SBND_EOP);
     side_data->xcnt  =     CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_SBND,AIF2_DB_EDB_DEBUG_SBND_XCNT);
     //side_data->Symbol=       CSL_FEXT(hAif2->regs->DB_IDB_DEBUG_SBND,AIF2_DB_EDB_DEBUG_SBND_SYMBOL);
}

 /** ============================================================================
 *   @n@b Aif2Fl_getEDbDebugOffsetData
 *
 *   @b Description
 *   @n Get Write and Read Offset Value at address in DB_EDB_DEBUG_OFS
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           uint8_t *         ponter to write and read offset data 
     @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_DB_EDB_DEBUG_OFS_DAT_WOFF
 *
 *   @b Example
 *   @verbatim
        uint8_t  offset[2];
        
        Aif2Fl_getEDbDebugOffsetData(hAif2, &offset[0]);

        uint8_t write_offset = offset[0];
        uint8_t read_offset = offset[1];
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEDbDebugOffsetData (
        Aif2Fl_Handle   hAif2,
        uint8_t*                 offset
)

{ 
        offset[0] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_OFS_DAT,AIF2_DB_EDB_DEBUG_OFS_DAT_WOFF);
	 offset[1] = CSL_FEXT(hAif2->regs->DB_EDB_DEBUG_OFS_DAT,AIF2_DB_EDB_DEBUG_OFS_DAT_ROFF);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEgrEopCount
 *
 *   @b Description
 *   @n Get Egress EOP count from DB
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_DB_EDB_EOP_CNT_EOP_CNT
 *
 *   @b Example
 *   @verbatim
        uint32_t  eop_cnt;
        
        eop_cnt = Aif2Fl_getEgrEopCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getEgrEopCount (
        Aif2Fl_Handle   hAif2
)

{ 
        return CSL_FEXT(hAif2->regs->DB_EDB_EOP_CNT ,AIF2_DB_EDB_EOP_CNT_EOP_CNT);
}


/** ============================================================================
 *   @n@b Aif2Fl_getIngrEopCount
 *
 *   @b Description
 *   @n Get Ingress EOP count
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AD_ISCH_EOP_CNT_EOP_CNT
 *
 *   @b Example
 *   @verbatim
        uint32_t  eop_cnt;
        
        eop_cnt = Aif2Fl_getIngrEopCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getIngrEopCount (
        Aif2Fl_Handle   hAif2
)

{ 
        return CSL_FEXT(hAif2->regs->AD_ISCH_EOP_CNT ,AIF2_AD_ISCH_EOP_CNT_EOP_CNT);
}


/** ============================================================================
 *   @n@b Aif2Fl_getAtLinkPiCapture
 *
 *   @b Description
 *   @n Get Aif2 timer Pi captured value for link
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance    should use hAif2->arg_link to select link
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t  
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  pi;
        hAif2->arg_link = LINK0;
        pi = Aif2Fl_getAtLinkPiCapture(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtLinkPiCapture (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->PI_DATA[hAif2->arg_link].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
}


/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtCapture
 *
 *   @b Description
 *   @n Get Aif2 Rad timer captured clock, symbol, frame count value upon a phyt frame boundary
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           Aif2Fl_AtCaptRadt    Radt clock, symbol and frame value structure
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_CAPTRADT_RADTCLOCKCOUNT_CAPTURE,AIF2_AT_CAPTRADT_RADTSYMBOLCOUNT_CAPTURE,
 *         AIF2_AT_CAPTRADT_RADTFRAMECOUNT_CAPTURE
 *   @b Example
 *   @verbatim
        Aif2Fl_AtCaptRadt  radt_count;
        
        Aif2Fl_getAtRadtCapture(hAif2,&radt_count);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getAtRadtCapture (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_AtCaptRadt*     radt
)
{ 
        radt->clock=  CSL_FEXT(hAif2->regs->AT_CAPTRADT, AIF2_AT_CAPTRADT_RADTCLOCKCOUNT_CAPTURE);
	 radt->symbol=  CSL_FEXT(hAif2->regs->AT_CAPTRADT, AIF2_AT_CAPTRADT_RADTSYMBOLCOUNT_CAPTURE);
	 radt->frame =  CSL_FEXT(hAif2->regs->AT_CAPTRADT, AIF2_AT_CAPTRADT_RADTFRAMECOUNT_CAPTURE);
}
 
 /** ============================================================================
 *   @n@b Aif2Fl_getAtRp1TypeCapture
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 type capture value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint8_t  
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_TYPE_CAPTURE_RP1TYPE_CAPTURED
 *
 *   @b Example
 *   @verbatim
        uint8_t  type;
        
        type = Aif2Fl_getAtRp1TypeCapture(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getAtRp1TypeCapture (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_TYPE_CAPTURE,AIF2_AT_RP1_TYPE_CAPTURE_RP1TYPE_CAPTURED);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1TodCaptureLsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 TOD capture value lsb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_TOD_CAPTURE_L_RP1TOD_CAPTURE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  tod_lo;
        
        tod_lo = Aif2Fl_getAtRp1TodCaptureLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1TodCaptureLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_TOD_CAPTURE_L,AIF2_AT_RP1_TOD_CAPTURE_L_RP1TOD_CAPTURE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1TodCaptureMsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 TOD capture value msb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_TOD_CAPTURE_H_RP1TOD_CAPTURE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  tod_hi;
        
        tod_hi = Aif2Fl_getAtRp1TodCaptureMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1TodCaptureMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_TOD_CAPTURE_H,AIF2_AT_RP1_TOD_CAPTURE_H_RP1TOD_CAPTURE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1Rp3CaptureLsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 RP3 capture value lsb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_RP3_CAPTURE_L_RP1RP3_CAPTURE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  RP3Lo;
        
        RP3Lo = Aif2Fl_getAtRp1Rp3CaptureLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1Rp3CaptureLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_RP3_CAPTURE_L,AIF2_AT_RP1_RP3_CAPTURE_L_RP1RP3_CAPTURE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1Rp3CaptureMsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 RP3 capture value msb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_RP3_CAPTURE_H_RP1RP3_CAPTURE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  RP3Hi;
        
        RP3Hi = Aif2Fl_getAtRp1Rp3CaptureMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1Rp3CaptureMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_RP3_CAPTURE_H,AIF2_AT_RP1_RP3_CAPTURE_H_RP1RP3_CAPTURE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1RadCaptureLsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 RAD capture value lsb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_RAD_CAPTURE_L_RP1RADIO_SYSTEM_CAPTURE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  RADLo;
        
        RADLo = Aif2Fl_getAtRp1RadCaptureLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1RadCaptureLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_RAD_CAPTURE_L,AIF2_AT_RP1_RAD_CAPTURE_L_RP1RADIO_SYSTEM_CAPTURE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRp1RadCaptureMsb
 *
 *   @b Description
 *   @n Get Aif2 timer RP1 Radio system capture value msb
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
     @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RP1_RAD_CAPTURE_H_RP1RADIO_SYSTEM_CAPTURE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  RADHi;
        
        RADHi = Aif2Fl_getAtRp1RadCaptureMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRp1RadCaptureMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RP1_RAD_CAPTURE_H,AIF2_AT_RP1_RAD_CAPTURE_H_RP1RADIO_SYSTEM_CAPTURE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtPhytClkCount
 *
 *   @b Description
 *   @n Get Aif2 Phy timer clock count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_PHYT_CLKCNT_VALUE_PHYTCLOCK_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtPhytClkCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtPhytClkCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_PHYT_CLKCNT_VALUE,AIF2_AT_PHYT_CLKCNT_VALUE_PHYTCLOCK_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtPhytFrameLsb
 *
 *   @b Description
 *   @n Get Aif2 Phy timer frame count lsb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_PHYT_FRM_VALUE_LSBS_PHYTFRAME_VALUE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtPhytFrameLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtPhytFrameLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_PHYT_FRM_VALUE_LSBS, AIF2_AT_PHYT_FRM_VALUE_LSBS_PHYTFRAME_VALUE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtPhytFrameMsb
 *
 *   @b Description
 *   @n Get Aif2 Phy timer frame count msb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_PHYT_FRM_VALUE_MSBS_PHYTFRAME_VALUE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtPhytFrameMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtPhytFrameMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_PHYT_FRM_VALUE_MSBS, AIF2_AT_PHYT_FRM_VALUE_MSBS_PHYTFRAME_VALUE_MSBS);
}


/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtClkCount
 *
 *   @b Description
 *   @n Get Aif2 Rad timer clock count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_VALUE_LSBS_RADTCLOCK_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtRadtClkCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRadtClkCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RADT_VALUE_LSBS,AIF2_AT_RADT_VALUE_LSBS_RADTCLOCK_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtSymCount
 *
 *   @b Description
 *   @n Get Aif2 Rad timer symbol count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_VALUE_LSBS_RADTSYMBOL_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint8_t  count;
        
        count = Aif2Fl_getAtRadtSymCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getAtRadtSymCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RADT_VALUE_LSBS,AIF2_AT_RADT_VALUE_LSBS_RADTSYMBOL_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtFrameLsb
 *
 *   @b Description
 *   @n Get Aif2 Rad timer frame count lsb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_VALUE_MID_RADTFRAME_VALUE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtRadtFrameLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRadtFrameLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RADT_VALUE_MID, AIF2_AT_RADT_VALUE_MID_RADTFRAME_VALUE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtFrameMsb
 *
 *   @b Description
 *   @n Get Aif2 Rad timer frame count msb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_VALUE_MSBS_RADTFRAME_VALUE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtRadtFrameMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRadtFrameMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RADT_VALUE_MSBS, AIF2_AT_RADT_VALUE_MSBS_RADTFRAME_VALUE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtUlRadtClkCount
 *
 *   @b Description
 *   @n Get Aif2 UL Rad timer clock count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_ULRADT_VALUE_LSBS_ULRADTCLOCK_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtUlRadtClkCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtUlRadtClkCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_ULRADT_VALUE_LSBS,AIF2_AT_ULRADT_VALUE_LSBS_ULRADTCLOCK_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtUlRadtSymCount
 *
 *   @b Description
 *   @n Get Aif2 UL Rad timer symbol count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_ULRADT_VALUE_LSBS_ULRADTSYMBOL_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint8_t  count;
        
        count = Aif2Fl_getAtUlRadtSymCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getAtUlRadtSymCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_ULRADT_VALUE_LSBS,AIF2_AT_ULRADT_VALUE_LSBS_ULRADTSYMBOL_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtUlRadtFrameLsb
 *
 *   @b Description
 *   @n Get Aif2 UL Rad timer frame count lsb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_ULRADT_VALUE_MID_ULRADTFRAME_VALUE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtUlRadtFrameLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtUlRadtFrameLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_ULRADT_VALUE_MID, AIF2_AT_ULRADT_VALUE_MID_ULRADTFRAME_VALUE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtUlRadtFrameMsb
 *
 *   @b Description
 *   @n Get Aif2 UL Rad timer frame count msb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_ULRADT_VALUE_MSBS_ULRADTFRAME_VALUE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtUlRadtFrameMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtUlRadtFrameMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_ULRADT_VALUE_MSBS, AIF2_AT_ULRADT_VALUE_MSBS_ULRADTFRAME_VALUE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtDlRadtClkCount
 *
 *   @b Description
 *   @n Get Aif2 DL Rad timer clock count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_DLRADT_VALUE_LSBS_DLRADTCLOCK_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtDlRadtClkCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtDlRadtClkCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_DLRADT_VALUE_LSBS,AIF2_AT_DLRADT_VALUE_LSBS_DLRADTCLOCK_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtDlRadtSymCount
 *
 *   @b Description
 *   @n Get Aif2 DL Rad timer symbol count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint8_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_DLRADT_VALUE_LSBS_DLRADTSYMBOL_COUNT_VALUE
 *
 *   @b Example
 *   @verbatim
        uint8_t  count;
        
        count = Aif2Fl_getAtDlRadtSymCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint8_t Aif2Fl_getAtDlRadtSymCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_DLRADT_VALUE_LSBS,AIF2_AT_DLRADT_VALUE_LSBS_DLRADTSYMBOL_COUNT_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtDlRadtFrameLsb
 *
 *   @b Description
 *   @n Get Aif2 DL Rad timer frame count lsb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_DLRADT_VALUE_MID_DLRADTFRAME_VALUE_LSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtDlRadtFrameLsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtDlRadtFrameLsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_DLRADT_VALUE_MID, AIF2_AT_DLRADT_VALUE_MID_DLRADTFRAME_VALUE_LSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtDlRadtFrameMsb
 *
 *   @b Description
 *   @n Get Aif2 DL Rad timer frame count msb value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_DLRADT_VALUE_MSBS_DLRADTFRAME_VALUE_MSBS
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtDlRadtFrameMsb(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtDlRadtFrameMsb (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_DLRADT_VALUE_MSBS, AIF2_AT_DLRADT_VALUE_MSBS_DLRADTFRAME_VALUE_MSBS);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtWcdmaValue
 *
 *   @b Description
 *   @n Get Aif2 Rad timer wcdma count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           Aif2Fl_AtWcdmaCount     WCDMA chip, slot and frame value structure
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_WCDMA_VALUE_CHIPVALUE,AIF2_AT_RADT_WCDMA_VALUE_SLOTVALUE,
 *         AIF2_AT_RADT_WCDMA_VALUE_FRAMEVALUE
 *   @b Example
 *   @verbatim
        Aif2Fl_AtWcdmaCount  wcdma_count;
        
        Aif2Fl_getAtRadtWcdmaValue(hAif2,&wcdma_count);

        wcdma_count.chip = ....
        wcdma_count.slot = .....
        wcdma_count.frame = ....
        
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getAtRadtWcdmaValue (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_AtWcdmaCount*     wcdma
)
{ 
        wcdma->chip =  CSL_FEXT(hAif2->regs->AT_RADT_WCDMA_VALUE, AIF2_AT_RADT_WCDMA_VALUE_CHIPVALUE);
	 wcdma->slot =  CSL_FEXT(hAif2->regs->AT_RADT_WCDMA_VALUE, AIF2_AT_RADT_WCDMA_VALUE_SLOTVALUE);
	 wcdma->frame =  CSL_FEXT(hAif2->regs->AT_RADT_WCDMA_VALUE, AIF2_AT_RADT_WCDMA_VALUE_FRAMEVALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtUlRadtWcdmaValue
 *
 *   @b Description
 *   @n Get Aif2 Ul Rad timer wcdma count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           Aif2Fl_AtWcdmaCount     WCDMA chip, slot and frame value structure
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_ULRADT_WCDMA_VALUE_CHIPVALUE,AIF2_AT_ULRADT_WCDMA_VALUE_SLOTVALUE,
 *         AIF2_AT_ULRADT_WCDMA_VALUE_FRAMEVALUE
 *   @b Example
 *   @verbatim
        Aif2Fl_AtWcdmaCount  wcdma_count;
        
        Aif2Fl_getAtUlRadtWcdmaValue(hAif2,&wcdma_count);

        wcdma_count.chip = ....
        wcdma_count.slot = .....
        wcdma_count.frame = ....
        
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getAtUlRadtWcdmaValue (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_AtWcdmaCount*     wcdma
)
{ 
        wcdma->chip =  CSL_FEXT(hAif2->regs->AT_ULRADT_WCDMA_VALUE, AIF2_AT_ULRADT_WCDMA_VALUE_CHIPVALUE);
	 wcdma->slot =  CSL_FEXT(hAif2->regs->AT_ULRADT_WCDMA_VALUE, AIF2_AT_ULRADT_WCDMA_VALUE_SLOTVALUE);
	 wcdma->frame =  CSL_FEXT(hAif2->regs->AT_ULRADT_WCDMA_VALUE, AIF2_AT_ULRADT_WCDMA_VALUE_FRAMEVALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtDlRadtWcdmaValue
 *
 *   @b Description
 *   @n Get Aif2 Dl Rad timer wcdma count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           Aif2Fl_AtWcdmaCount     WCDMA chip, slot and frame value structure
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_DLRADT_WCDMA_VALUE_CHIPVALUE,AIF2_AT_DLRADT_WCDMA_VALUE_SLOTVALUE,
 *         AIF2_AT_DLRADT_WCDMA_VALUE_FRAMEVALUE
 *   @b Example
 *   @verbatim
        Aif2Fl_AtWcdmaCount  wcdma_count;
        
        Aif2Fl_getAtDlRadtWcdmaValue(hAif2,&wcdma_count);

        wcdma_count.chip = ....
        wcdma_count.slot = .....
        wcdma_count.frame = ....
        
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getAtDlRadtWcdmaValue (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_AtWcdmaCount*     wcdma
)
{ 
        wcdma->chip =  CSL_FEXT(hAif2->regs->AT_DLRADT_WCDMA_VALUE, AIF2_AT_DLRADT_WCDMA_VALUE_CHIPVALUE);
	 wcdma->slot =  CSL_FEXT(hAif2->regs->AT_DLRADT_WCDMA_VALUE, AIF2_AT_DLRADT_WCDMA_VALUE_SLOTVALUE);
	 wcdma->frame =  CSL_FEXT(hAif2->regs->AT_DLRADT_WCDMA_VALUE, AIF2_AT_DLRADT_WCDMA_VALUE_FRAMEVALUE);
}


/** ============================================================================
 *   @n@b Aif2Fl_getAtRadtTstampClkCount
 *
 *   @b Description
 *   @n Get Aif2 Rad timer time stamp count value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
      @endverbatim
 *
 *   <b> Return Value </b>  uint32_t
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_RADT_TSTAMP_VALUE_RADTTSTAMP_CLOCK_COUNTER_VALUE
 *
 *   @b Example
 *   @verbatim
        uint32_t  count;
        
        count = Aif2Fl_getAtRadtTstampClkCount(hAif2);
        
     @endverbatim
 * ===========================================================================
 */
static inline
uint32_t Aif2Fl_getAtRadtTstampClkCount (
        Aif2Fl_Handle   hAif2
)
{ 
        return CSL_FEXT(hAif2->regs->AT_RADT_TSTAMP_VALUE, AIF2_AT_RADT_TSTAMP_VALUE_RADTTSTAMP_CLOCK_COUNTER_VALUE);
}

/** ============================================================================
 *   @n@b Aif2Fl_getAtGsmTcount
 *
 *   @b Description
 *   @n Get Aif2 GSM Tcount value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     
           Aif2Fl_AtWcdmaCount     WCDMA chip, slot and frame value structure
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_AT_GSM_TCOUNT_VALUE_T1,AIF2_AT_GSM_TCOUNT_VALUE_T2,
 *         AIF2_AT_GSM_TCOUNT_VALUE_T3
 *   @b Example
 *   @verbatim
        Aif2Fl_AtGsmTCount  tcount;
        
        Aif2Fl_getAtGsmTcount(hAif2,&tcount);

        tcount.t1 = ....
        tcount.t2 = .....
        tcount.t3 = ....
        
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getAtGsmTcount (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_AtGsmTCount*     tcount
)
{ 
        tcount->t1 =  CSL_FEXT(hAif2->regs->AT_GSM_TCOUNT_VALUE, AIF2_AT_GSM_TCOUNT_VALUE_T1);
	 tcount->t2 =  CSL_FEXT(hAif2->regs->AT_GSM_TCOUNT_VALUE, AIF2_AT_GSM_TCOUNT_VALUE_T2);
	 tcount->t3 =  CSL_FEXT(hAif2->regs->AT_GSM_TCOUNT_VALUE, AIF2_AT_GSM_TCOUNT_VALUE_T3);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeDbIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE DB interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeDbInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_DB_IRS_DB_EE_I_TRC_RAM_OVFL_ERR,       AIF2_EE_DB_IRS_DB_EE_I_TOKEN_OVFL_ERR,
 *         AIF2_EE_DB_IRS_DB_EE_I_FIFO_OVFL_ERR,              AIF2_EE_DB_IRS_DB_EE_I_PD2DB_FULL_ERR,
 *         AIF2_EE_DB_IRS_DB_EE_E_PS_AXC_ERR,                 AIF2_EE_DB_IRS_DB_EE_E_CD_DATA_ERR,
 *         AIF2_EE_DB_IRS_DB_EE_E_CD_DATA_TYPE_ERR;
 *
 *         AIF2_EE_DB_EN_EV0_DB_EE_I_TRC_RAM_OVFL_ERR,       AIF2_EE_DB_EN_EV0_DB_EE_I_TOKEN_OVFL_ERR,
 *         AIF2_EE_DB_EN_EV0_DB_EE_I_FIFO_OVFL_ERR,              AIF2_EE_DB_EN_EV0_DB_EE_I_PD2DB_FULL_ERR,
 *         AIF2_EE_DB_EN_EV0_DB_EE_E_PS_AXC_ERR,                 AIF2_EE_DB_EN_EV0_DB_EE_E_CD_DATA_ERR,
 *         AIF2_EE_DB_EN_EV0_DB_EE_E_CD_DATA_TYPE_ERR;
 *
 *         AIF2_EE_DB_EN_EV1_DB_EE_I_TRC_RAM_OVFL_ERR,       AIF2_EE_DB_EN_EV1_DB_EE_I_TOKEN_OVFL_ERR,
 *         AIF2_EE_DB_EN_EV1_DB_EE_I_FIFO_OVFL_ERR,              AIF2_EE_DB_EN_EV1_DB_EE_I_PD2DB_FULL_ERR,
 *         AIF2_EE_DB_EN_EV1_DB_EE_E_PS_AXC_ERR,                 AIF2_EE_DB_EN_EV1_DB_EE_E_CD_DATA_ERR,
 *         AIF2_EE_DB_EN_EV1_DB_EE_E_CD_DATA_TYPE_ERR;
 *
 *         AIF2_EE_DB_EN_STS_EV0_DB_EE_I_TRC_RAM_OVFL_ERR,       AIF2_EE_DB_EN_STS_EV0_DB_EE_I_TOKEN_OVFL_ERR,
 *         AIF2_EE_DB_EN_STS_EV0_DB_EE_I_FIFO_OVFL_ERR,              AIF2_EE_DB_EN_STS_EV0_DB_EE_I_PD2DB_FULL_ERR,
 *         AIF2_EE_DB_EN_STS_EV0_DB_EE_E_PS_AXC_ERR,                 AIF2_EE_DB_EN_STS_EV0_DB_EE_E_CD_DATA_ERR,
 *         AIF2_EE_DB_EN_STS_EV0_DB_EE_E_CD_DATA_TYPE_ERR;
 *
 *         AIF2_EE_DB_EN_STS_EV1_DB_EE_I_TRC_RAM_OVFL_ERR,       AIF2_EE_DB_EN_STS_EV1_DB_EE_I_TOKEN_OVFL_ERR,
 *         AIF2_EE_DB_EN_STS_EV1_DB_EE_I_FIFO_OVFL_ERR,              AIF2_EE_DB_EN_STS_EV1_DB_EE_I_PD2DB_FULL_ERR,
 *         AIF2_EE_DB_EN_STS_EV1_DB_EE_E_PS_AXC_ERR,                 AIF2_EE_DB_EN_STS_EV1_DB_EE_E_CD_DATA_ERR,
 *         AIF2_EE_DB_EN_STS_EV1_DB_EE_E_CD_DATA_TYPE_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeDbInt  DbInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeDbIntStatus(hAif2,&DbInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeDbIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeDbInt*     DbInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 DbInt->db_ee_i_trc_ram_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_I_TRC_RAM_OVFL_ERR);
	 DbInt->db_ee_i_token_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_I_TOKEN_OVFL_ERR);
	 DbInt->db_ee_i_fifo_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_I_FIFO_OVFL_ERR);
	 DbInt->db_ee_i_pd2db_full_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_I_PD2DB_FULL_ERR);
	 DbInt->db_ee_e_ps_axc_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_E_PS_AXC_ERR);
	 DbInt->db_ee_e_cd_data_err =  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_E_CD_DATA_ERR);
	 DbInt->db_ee_e_cd_data_type_err=  CSL_FEXT(hAif2->regs->EE_DB_IRS, AIF2_EE_DB_IRS_DB_EE_E_CD_DATA_TYPE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 DbInt->db_ee_i_trc_ram_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_I_TRC_RAM_OVFL_ERR);
	 DbInt->db_ee_i_token_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_I_TOKEN_OVFL_ERR);
	 DbInt->db_ee_i_fifo_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_I_FIFO_OVFL_ERR);
	 DbInt->db_ee_i_pd2db_full_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_I_PD2DB_FULL_ERR);
	 DbInt->db_ee_e_ps_axc_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_E_PS_AXC_ERR);
	 DbInt->db_ee_e_cd_data_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_E_CD_DATA_ERR);
	 DbInt->db_ee_e_cd_data_type_err=  CSL_FEXT(hAif2->regs->EE_DB_EN_EV0, AIF2_EE_DB_EN_EV0_DB_EE_E_CD_DATA_TYPE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 DbInt->db_ee_i_trc_ram_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_I_TRC_RAM_OVFL_ERR);
	 DbInt->db_ee_i_token_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_I_TOKEN_OVFL_ERR);
	 DbInt->db_ee_i_fifo_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_I_FIFO_OVFL_ERR);
	 DbInt->db_ee_i_pd2db_full_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_I_PD2DB_FULL_ERR);
	 DbInt->db_ee_e_ps_axc_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_E_PS_AXC_ERR);
	 DbInt->db_ee_e_cd_data_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_E_CD_DATA_ERR);
	 DbInt->db_ee_e_cd_data_type_err=  CSL_FEXT(hAif2->regs->EE_DB_EN_EV1, AIF2_EE_DB_EN_EV1_DB_EE_E_CD_DATA_TYPE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 DbInt->db_ee_i_trc_ram_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_I_TRC_RAM_OVFL_ERR);
	 DbInt->db_ee_i_token_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_I_TOKEN_OVFL_ERR);
	 DbInt->db_ee_i_fifo_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_I_FIFO_OVFL_ERR);
	 DbInt->db_ee_i_pd2db_full_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_I_PD2DB_FULL_ERR);
	 DbInt->db_ee_e_ps_axc_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_E_PS_AXC_ERR);
	 DbInt->db_ee_e_cd_data_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_E_CD_DATA_ERR);
	 DbInt->db_ee_e_cd_data_type_err=  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV0, AIF2_EE_DB_EN_STS_EV0_DB_EE_E_CD_DATA_TYPE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 DbInt->db_ee_i_trc_ram_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_I_TRC_RAM_OVFL_ERR);
	 DbInt->db_ee_i_token_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_I_TOKEN_OVFL_ERR);
	 DbInt->db_ee_i_fifo_ovfl_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_I_FIFO_OVFL_ERR);
	 DbInt->db_ee_i_pd2db_full_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_I_PD2DB_FULL_ERR);
	 DbInt->db_ee_e_ps_axc_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_E_PS_AXC_ERR);
	 DbInt->db_ee_e_cd_data_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_E_CD_DATA_ERR);
	 DbInt->db_ee_e_cd_data_type_err =  CSL_FEXT(hAif2->regs->EE_DB_EN_STS_EV1, AIF2_EE_DB_EN_STS_EV1_DB_EE_E_CD_DATA_TYPE_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeAdIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE AD interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeAdInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_AD_IRS_AD_EE_I_CD_DATA_ERR,       AIF2_EE_AD_IRS_AD_EE_E_CD_SCH_ERR,
 *         AIF2_EE_AD_IRS_AD_EE_I_DMA_0_ERR,            AIF2_EE_AD_IRS_AD_EE_I_DMA_1_ERR,
 *         AIF2_EE_AD_IRS_AD_EE_I_DMA_2_ERR,            AIF2_EE_AD_IRS_AD_EE_E_DMA_0_ERR,
 *         AIF2_EE_AD_IRS_AD_EE_E_DMA_1_ERR,            AIF2_EE_AD_IRS_AD_EE_E_DMA_2_ERR;
 *
 *         AIF2_EE_AD_EN_EV0_AD_EE_I_CD_DATA_ERR,       AIF2_EE_AD_EN_EV0_AD_EE_E_CD_SCH_ERR,
 *         AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_0_ERR,            AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_1_ERR,
 *         AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_2_ERR,            AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_0_ERR,
 *         AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_1_ERR,            AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_2_ERR;
 *
 *         AIF2_EE_AD_EN_EV1_AD_EE_I_CD_DATA_ERR,       AIF2_EE_AD_EN_EV1_AD_EE_E_CD_SCH_ERR,
 *         AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_0_ERR,            AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_1_ERR,
 *         AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_2_ERR,            AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_0_ERR,
 *         AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_1_ERR,            AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_2_ERR;
 *
 *         AIF2_EE_AD_EN_STS_EV0_AD_EE_I_CD_DATA_ERR,       AIF2_EE_AD_EN_STS_EV0_AD_EE_E_CD_SCH_ERR,
 *         AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_0_ERR,            AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_1_ERR,
 *         AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_2_ERR,            AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_0_ERR,
 *         AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_1_ERR,            AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_2_ERR;
 *
 *         AIF2_EE_AD_EN_STS_EV1_AD_EE_I_CD_DATA_ERR,       AIF2_EE_AD_EN_STS_EV1_AD_EE_E_CD_SCH_ERR,
 *         AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_0_ERR,            AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_1_ERR,
 *         AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_2_ERR,            AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_0_ERR,
 *         AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_1_ERR,            AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_2_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAdInt  AdInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeAdIntStatus(hAif2,&AdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeAdIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeAdInt*     AdInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 AdInt->ad_ee_i_cd_data_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_I_CD_DATA_ERR);
	 AdInt->ad_ee_e_cd_sch_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_E_CD_SCH_ERR);
	 AdInt->ad_ee_i_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_I_DMA_0_ERR);
	 AdInt->ad_ee_i_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_I_DMA_1_ERR);
	 AdInt->ad_ee_i_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_I_DMA_2_ERR);
	 AdInt->ad_ee_e_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_E_DMA_0_ERR);
	 AdInt->ad_ee_e_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_E_DMA_1_ERR);
	 AdInt->ad_ee_e_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_IRS, AIF2_EE_AD_IRS_AD_EE_E_DMA_2_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 AdInt->ad_ee_i_cd_data_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_I_CD_DATA_ERR);
	 AdInt->ad_ee_e_cd_sch_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_E_CD_SCH_ERR);
	 AdInt->ad_ee_i_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_0_ERR);
	 AdInt->ad_ee_i_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_1_ERR);
	 AdInt->ad_ee_i_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_I_DMA_2_ERR);
	 AdInt->ad_ee_e_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_0_ERR);
	 AdInt->ad_ee_e_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_1_ERR);
	 AdInt->ad_ee_e_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV0, AIF2_EE_AD_EN_EV0_AD_EE_E_DMA_2_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 AdInt->ad_ee_i_cd_data_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_I_CD_DATA_ERR);
	 AdInt->ad_ee_e_cd_sch_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_E_CD_SCH_ERR);
	 AdInt->ad_ee_i_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_0_ERR);
	 AdInt->ad_ee_i_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_1_ERR);
	 AdInt->ad_ee_i_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_I_DMA_2_ERR);
	 AdInt->ad_ee_e_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_0_ERR);
	 AdInt->ad_ee_e_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_1_ERR);
	 AdInt->ad_ee_e_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_EV1, AIF2_EE_AD_EN_EV1_AD_EE_E_DMA_2_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 AdInt->ad_ee_i_cd_data_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_I_CD_DATA_ERR);
	 AdInt->ad_ee_e_cd_sch_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_E_CD_SCH_ERR);
	 AdInt->ad_ee_i_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_0_ERR);
	 AdInt->ad_ee_i_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_1_ERR);
	 AdInt->ad_ee_i_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_I_DMA_2_ERR);
	 AdInt->ad_ee_e_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_0_ERR);
	 AdInt->ad_ee_e_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_1_ERR);
	 AdInt->ad_ee_e_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV0, AIF2_EE_AD_EN_STS_EV0_AD_EE_E_DMA_2_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 AdInt->ad_ee_i_cd_data_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_I_CD_DATA_ERR);
	 AdInt->ad_ee_e_cd_sch_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_E_CD_SCH_ERR);
	 AdInt->ad_ee_i_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_0_ERR);
	 AdInt->ad_ee_i_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_1_ERR);
	 AdInt->ad_ee_i_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_I_DMA_2_ERR);
	 AdInt->ad_ee_e_dma_0_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_0_ERR);
	 AdInt->ad_ee_e_dma_1_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_1_ERR);
	 AdInt->ad_ee_e_dma_2_err =  CSL_FEXT(hAif2->regs->EE_AD_EN_STS_EV1, AIF2_EE_AD_EN_STS_EV1_AD_EE_E_DMA_2_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeCdIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE CD(PKTDMA) interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeCdInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_CD_IRS_CD_EE_SOP_DESC_STARVE_ERR,       AIF2_EE_CD_IRS_CD_EE_MOP_DESC_STARVE_ERR;
 *
 *         AIF2_EE_CD_EN_EV_CD_EE_SOP_DESC_STARVE_ERR,       AIF2_EE_CD_EN_EV_CD_EE_MOP_DESC_STARVE_ERR;
 *
 *         AIF2_EE_CD_EN_STS_EV_CD_EE_SOP_DESC_STARVE_ERR,       AIF2_EE_CD_EN_STS_EV_CD_EE_MOP_DESC_STARVE_ERR;
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_EeCdInt  CdInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeCdIntStatus(hAif2,&CdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeCdIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeCdInt*     CdInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 CdInt->cd_ee_sop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_IRS, AIF2_EE_CD_IRS_CD_EE_SOP_DESC_STARVE_ERR);
	 CdInt->cd_ee_mop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_IRS, AIF2_EE_CD_IRS_CD_EE_MOP_DESC_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
        CdInt->cd_ee_sop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_EN_EV, AIF2_EE_CD_EN_EV_CD_EE_SOP_DESC_STARVE_ERR);
	 CdInt->cd_ee_mop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_EN_EV, AIF2_EE_CD_EN_EV_CD_EE_MOP_DESC_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
        CdInt->cd_ee_sop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_EN_STS_EV, AIF2_EE_CD_EN_STS_EV_CD_EE_SOP_DESC_STARVE_ERR);
	 CdInt->cd_ee_mop_desc_starve_err =  CSL_FEXT(hAif2->regs->EE_CD_EN_STS_EV, AIF2_EE_CD_EN_STS_EV_CD_EE_MOP_DESC_STARVE_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeSdIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE SD interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeSdInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_SD_IRS_SD_EE_STSPLL_B4_ERR,       AIF2_EE_SD_IRS_SD_EE_STSPLL_B8_ERR;
 *
 *       AIF2_EE_SD_EN_EV0_SD_EE_STSPLL_B4_ERR,       AIF2_EE_SD_EN_EV0_SD_EE_STSPLL_B8_ERR;
 *       AIF2_EE_SD_EN_STS_EV0_SD_EE_STSPLL_B4_ERR,    AIF2_EE_SD_EN_STS_EV0_SD_EE_STSPLL_B8_ERR;
 * 
 *       AIF2_EE_SD_EN_EV1_SD_EE_STSPLL_B4_ERR,       AIF2_EE_SD_EN_EV1_SD_EE_STSPLL_B8_ERR;
 *       AIF2_EE_SD_EN_STS_EV1_SD_EE_STSPLL_B4_ERR,    AIF2_EE_SD_EN_STS_EV1_SD_EE_STSPLL_B8_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeSdInt  SdInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeSdIntStatus(hAif2,&SdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeSdIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeSdInt*     SdInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 SdInt->sd_ee_stspll_b4_err =  CSL_FEXT(hAif2->regs->EE_SD_IRS, AIF2_EE_SD_IRS_SD_EE_STSPLL_B4_ERR);
	 SdInt->sd_ee_stspll_b8_err =  CSL_FEXT(hAif2->regs->EE_SD_IRS, AIF2_EE_SD_IRS_SD_EE_STSPLL_B8_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 SdInt->sd_ee_stspll_b4_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_EV0, AIF2_EE_SD_EN_EV0_SD_EE_STSPLL_B4_ERR);
	 SdInt->sd_ee_stspll_b8_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_EV0, AIF2_EE_SD_EN_EV0_SD_EE_STSPLL_B8_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
     SdInt->sd_ee_stspll_b4_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_STS_EV0, AIF2_EE_SD_EN_STS_EV0_SD_EE_STSPLL_B4_ERR);
	 SdInt->sd_ee_stspll_b8_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_STS_EV0, AIF2_EE_SD_EN_STS_EV0_SD_EE_STSPLL_B8_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 SdInt->sd_ee_stspll_b4_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_EV1, AIF2_EE_SD_EN_EV1_SD_EE_STSPLL_B4_ERR);
	 SdInt->sd_ee_stspll_b8_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_EV1, AIF2_EE_SD_EN_EV1_SD_EE_STSPLL_B8_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
     SdInt->sd_ee_stspll_b4_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_STS_EV1, AIF2_EE_SD_EN_STS_EV1_SD_EE_STSPLL_B4_ERR);
	 SdInt->sd_ee_stspll_b8_err =  CSL_FEXT(hAif2->regs->EE_SD_EN_STS_EV1, AIF2_EE_SD_EN_STS_EV1_SD_EE_STSPLL_B8_ERR);
        }
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeVcIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE VC interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeVcInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_VC_IRS_VC_EE_VBUS_ERR;  AIF2_EE_VC_EN_EV0_VC_EE_VBUS_ERR;
 *         AIF2_EE_VC_EN_STS_EV0_VC_EE_VBUS_ERR;  AIF2_EE_VC_EN_EV1_VC_EE_VBUS_ERR;
 *         AIF2_EE_VC_EN_STS_EV1_VC_EE_VBUS_ERR
 *   @b Example
 *   @verbatim
        Aif2Fl_EeVcInt  VcInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeVcIntStatus(hAif2,&VcInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeVcIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeVcInt*     VcInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS)
	 VcInt->vc_ee_vbus_err =  CSL_FEXT(hAif2->regs->EE_VC_IRS, AIF2_EE_VC_IRS_VC_EE_VBUS_ERR);
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0)
	 VcInt->vc_ee_vbus_err =  CSL_FEXT(hAif2->regs->EE_VC_EN_EV0, AIF2_EE_VC_EN_EV0_VC_EE_VBUS_ERR);
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0)
        VcInt->vc_ee_vbus_err =  CSL_FEXT(hAif2->regs->EE_VC_EN_STS_EV0, AIF2_EE_VC_EN_STS_EV0_VC_EE_VBUS_ERR);
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1)
	 VcInt->vc_ee_vbus_err =  CSL_FEXT(hAif2->regs->EE_VC_EN_EV1, AIF2_EE_VC_EN_EV1_VC_EE_VBUS_ERR);
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1)
        VcInt->vc_ee_vbus_err =  CSL_FEXT(hAif2->regs->EE_VC_EN_STS_EV1, AIF2_EE_VC_EN_STS_EV1_VC_EE_VBUS_ERR);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeAif2RunStatus
 *
 *   @b Description
 *   @n Get Aif2 EE AIF2 run status value 
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance    
           Aif2Fl_EeAif2Run     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_AIF2_RUN_STS_AIF2_PHY_RUN,  AIF2_EE_AIF2_RUN_STS_AIF2_GLOBAL_RUN;
 *         
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAif2Run  Aif2Run;
        
        Aif2Fl_getEeAif2RunStatus(hAif2,&Aif2Run);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeAif2RunStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeAif2Run*     Aif2Run
)
{ 
	 Aif2Run->aif2_phy_run =  CSL_FEXT(hAif2->regs->EE_AIF2_RUN_STS, AIF2_EE_AIF2_RUN_STS_AIF2_PHY_RUN);
	 Aif2Run->aif2_global_run =  CSL_FEXT(hAif2->regs->EE_AIF2_RUN_STS, AIF2_EE_AIF2_RUN_STS_AIF2_GLOBAL_RUN);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeAif2Origination
 *
 *   @b Description
 *   @n Get Aif2 EE error or alarm origination status 
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance    
           Aif2Fl_EeOrigin     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A0,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B0,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A1,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B1,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A2,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B2,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A3,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B3,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A4,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B4,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A5,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B5,
 *         AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A0,  AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B0,
 *         AIF2_EE_ERR_ALRM_ORGN_AT_EN_STS,AIF2_EE_ERR_ALRM_ORGN_SD_EN_STS,
 *         AIF2_EE_ERR_ALRM_ORGN_DB_EN_STS,AIF2_EE_ERR_ALRM_ORGN_AD_EN_STS,
 *         AIF2_EE_ERR_ALRM_ORGN_VC_EN_STS
 *   @b Example
 *   @verbatim
        Aif2Fl_EeOrigin  Aif2Origin;
        
        Aif2Fl_getEeAif2Origination(hAif2,&Aif2Origin);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeAif2Origination (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeOrigin*     Aif2Origin
)
{ 
	 Aif2Origin->lk_en_sts_a0=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A0);
	 Aif2Origin->lk_en_sts_b0=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B0);
	 Aif2Origin->lk_en_sts_a1=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A1);
	 Aif2Origin->lk_en_sts_b1=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B1);
	 Aif2Origin->lk_en_sts_a2=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A2);
	 Aif2Origin->lk_en_sts_b2=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B2);
	 Aif2Origin->lk_en_sts_a3=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A3);
	 Aif2Origin->lk_en_sts_b3=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B3);
	 Aif2Origin->lk_en_sts_a4=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A4);
	 Aif2Origin->lk_en_sts_b4=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B4);
	 Aif2Origin->lk_en_sts_a5=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_A5);
	 Aif2Origin->lk_en_sts_b5=  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_LK_EN_STS_B5);
	 Aif2Origin->at_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_AT_EN_STS);
	 Aif2Origin->sd_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_SD_EN_STS);
	 Aif2Origin->db_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_DB_EN_STS);
	 Aif2Origin->ad_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_AD_EN_STS);
	 Aif2Origin->cd_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_CD_EN_STS);
	 Aif2Origin->vc_en_sts =  CSL_FEXT(hAif2->regs->EE_ERR_ALRM_ORGN, AIF2_EE_ERR_ALRM_ORGN_VC_EN_STS);
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeLinkAIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE Link A interrupt  status value 
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function and hAif2->arg_link to select link
           Aif2Fl_EeLinkAInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_LK_IRS_A_RM_EE_SYNC_STATUS_CHANGE_ERR,      AIF2_EE_LK_IRS_A_RM_EE_NUM_LOS_DET_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_LCV_DET_ERR,                            AIF2_EE_LK_IRS_A_RM_EE_FRAME_BNDRY_DET_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_BLOCK_BNDRY_DET_ERR,            AIF2_EE_LK_IRS_A_RM_EE_MISSING_K28P5_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_MISSING_K28P7_ERR,                 AIF2_EE_LK_IRS_A_RM_EE_K30P7_DET_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_LOC_DET_ERR,                           AIF2_EE_LK_IRS_A_RM_EE_RX_FIFO_OVF_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_RCVD_LOS_ERR,                         AIF2_EE_LK_IRS_A_RM_EE_RCVD_LOF_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_RCVD_RAI_ERR,                         AIF2_EE_LK_IRS_A_RM_EE_RCVD_SDI_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_LOS_ERR,                                   AIF2_EE_LK_IRS_A_RM_EE_LOF_ERR,
 *         AIF2_EE_LK_IRS_A_RM_EE_HFNSYNC_STATE_ERR,                AIF2_EE_LK_IRS_A_RM_EE_LOF_STATE_ERR,
 *         AIF2_EE_LK_IRS_A_TM_EE_FRM_MISALIGN_ERR,                   AIF2_EE_LK_IRS_A_TM_EE_FIFO_STARVE_ERR;
 *
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR,      AIF2_EE_LK_EN_A_EV0_RM_EE_NUM_LOS_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_LCV_DET_ERR,                            AIF2_EE_LK_EN_A_EV0_RM_EE_FRAME_BNDRY_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_BLOCK_BNDRY_DET_ERR,            AIF2_EE_LK_EN_A_EV0_RM_EE_MISSING_K28P5_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_MISSING_K28P7_ERR,                 AIF2_EE_LK_EN_A_EV0_RM_EE_K30P7_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_LOC_DET_ERR,                           AIF2_EE_LK_EN_A_EV0_RM_EE_RX_FIFO_OVF_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_LOS_ERR,                         AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_LOF_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_RAI_ERR,                         AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_SDI_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_LOS_ERR,                                   AIF2_EE_LK_EN_A_EV0_RM_EE_LOF_ERR,
 *         AIF2_EE_LK_EN_A_EV0_RM_EE_HFNSYNC_STATE_ERR,                AIF2_EE_LK_EN_A_EV0_RM_EE_LOF_STATE_ERR,
 *         AIF2_EE_LK_EN_A_EV0_TM_EE_FRM_MISALIGN_ERR,                   AIF2_EE_LK_EN_A_EV0_TM_EE_FIFO_STARVE_ERR;
 *
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR,      AIF2_EE_LK_EN_A_EV1_RM_EE_NUM_LOS_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_LCV_DET_ERR,                            AIF2_EE_LK_EN_A_EV1_RM_EE_FRAME_BNDRY_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_BLOCK_BNDRY_DET_ERR,            AIF2_EE_LK_EN_A_EV1_RM_EE_MISSING_K28P5_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_MISSING_K28P7_ERR,                 AIF2_EE_LK_EN_A_EV1_RM_EE_K30P7_DET_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_LOC_DET_ERR,                           AIF2_EE_LK_EN_A_EV1_RM_EE_RX_FIFO_OVF_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_LOS_ERR,                         AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_LOF_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_RAI_ERR,                         AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_SDI_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_LOS_ERR,                                   AIF2_EE_LK_EN_A_EV1_RM_EE_LOF_ERR,
 *         AIF2_EE_LK_EN_A_EV1_RM_EE_HFNSYNC_STATE_ERR,                AIF2_EE_LK_EN_A_EV1_RM_EE_LOF_STATE_ERR,
 *         AIF2_EE_LK_EN_A_EV1_TM_EE_FRM_MISALIGN_ERR,                   AIF2_EE_LK_EN_A_EV1_TM_EE_FIFO_STARVE_ERR;
 *
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR,      AIF2_EE_LK_EN_STS_A_EV0_RM_EE_NUM_LOS_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LCV_DET_ERR,                            AIF2_EE_LK_EN_STS_A_EV0_RM_EE_FRAME_BNDRY_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_BLOCK_BNDRY_DET_ERR,            AIF2_EE_LK_EN_STS_A_EV0_RM_EE_MISSING_K28P5_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_MISSING_K28P7_ERR,                 AIF2_EE_LK_EN_STS_A_EV0_RM_EE_K30P7_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOC_DET_ERR,                           AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RX_FIFO_OVF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_LOS_ERR,                         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_LOF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_RAI_ERR,                         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_SDI_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOS_ERR,                                   AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_RM_EE_HFNSYNC_STATE_ERR,                AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOF_STATE_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV0_TM_EE_FRM_MISALIGN_ERR,                   AIF2_EE_LK_EN_STS_A_EV0_TM_EE_FIFO_STARVE_ERR;
 *
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR,      AIF2_EE_LK_EN_STS_A_EV1_RM_EE_NUM_LOS_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LCV_DET_ERR,                            AIF2_EE_LK_EN_STS_A_EV1_RM_EE_FRAME_BNDRY_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_BLOCK_BNDRY_DET_ERR,            AIF2_EE_LK_EN_STS_A_EV1_RM_EE_MISSING_K28P5_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_MISSING_K28P7_ERR,                 AIF2_EE_LK_EN_STS_A_EV1_RM_EE_K30P7_DET_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOC_DET_ERR,                           AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RX_FIFO_OVF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_LOS_ERR,                         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_LOF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_RAI_ERR,                         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_SDI_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOS_ERR,                                   AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOF_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_RM_EE_HFNSYNC_STATE_ERR,                AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOF_STATE_ERR,
 *         AIF2_EE_LK_EN_STS_A_EV1_TM_EE_FRM_MISALIGN_ERR,                   AIF2_EE_LK_EN_STS_A_EV1_TM_EE_FIFO_STARVE_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeLinkAInt  LinkAInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        hAif2->arg_link = 0;//link0
        
        Aif2Fl_getEeLinkAIntStatus(hAif2,&LinkAInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeLinkAIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeLinkAInt*     LinkAInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 LinkAInt->rm_ee_sync_status_change_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_SYNC_STATUS_CHANGE_ERR);
	 LinkAInt->rm_ee_num_los_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_NUM_LOS_DET_ERR);
	 LinkAInt->rm_ee_lcv_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_LCV_DET_ERR);
	 LinkAInt->rm_ee_frame_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_FRAME_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_block_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_BLOCK_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_missing_k28p5_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_MISSING_K28P5_ERR);
	 LinkAInt->rm_ee_missing_k28p7_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_MISSING_K28P7_ERR);
	 LinkAInt->rm_ee_k30p7_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_K30P7_DET_ERR);
	 LinkAInt->rm_ee_loc_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_LOC_DET_ERR);
	 LinkAInt->rm_ee_rx_fifo_ovf_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_RX_FIFO_OVF_ERR);
	 LinkAInt->rm_ee_rcvd_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_RCVD_LOS_ERR);
	 LinkAInt->rm_ee_rcvd_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_RCVD_LOF_ERR);
	 LinkAInt->rm_ee_rcvd_rai_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_RCVD_RAI_ERR);
	 LinkAInt->rm_ee_rcvd_sdi_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_RCVD_SDI_ERR);
	 LinkAInt->rm_ee_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_LOS_ERR);
	 LinkAInt->rm_ee_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_LOF_ERR);
	 LinkAInt->rm_ee_hfnsync_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_HFNSYNC_STATE_ERR);
	 LinkAInt->rm_ee_lof_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_RM_EE_LOF_STATE_ERR);
	 LinkAInt->tm_ee_frm_misalign_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_TM_EE_FRM_MISALIGN_ERR);
	 LinkAInt->tm_ee_fifo_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_A, AIF2_EE_LK_IRS_A_TM_EE_FIFO_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 LinkAInt->rm_ee_sync_status_change_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR);
	 LinkAInt->rm_ee_num_los_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_NUM_LOS_DET_ERR);
	 LinkAInt->rm_ee_lcv_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_LCV_DET_ERR);
	 LinkAInt->rm_ee_frame_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_FRAME_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_block_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_BLOCK_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_missing_k28p5_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_MISSING_K28P5_ERR);
	 LinkAInt->rm_ee_missing_k28p7_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_MISSING_K28P7_ERR);
	 LinkAInt->rm_ee_k30p7_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_K30P7_DET_ERR);
	 LinkAInt->rm_ee_loc_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_LOC_DET_ERR);
	 LinkAInt->rm_ee_rx_fifo_ovf_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_RX_FIFO_OVF_ERR);
	 LinkAInt->rm_ee_rcvd_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_LOS_ERR);
	 LinkAInt->rm_ee_rcvd_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_LOF_ERR);
	 LinkAInt->rm_ee_rcvd_rai_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_RAI_ERR);
	 LinkAInt->rm_ee_rcvd_sdi_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_RCVD_SDI_ERR);
	 LinkAInt->rm_ee_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_LOS_ERR);
	 LinkAInt->rm_ee_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_LOF_ERR);
	 LinkAInt->rm_ee_hfnsync_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_HFNSYNC_STATE_ERR);
	 LinkAInt->rm_ee_lof_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_RM_EE_LOF_STATE_ERR);
	 LinkAInt->tm_ee_frm_misalign_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_TM_EE_FRM_MISALIGN_ERR);
	 LinkAInt->tm_ee_fifo_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV0, AIF2_EE_LK_EN_A_EV0_TM_EE_FIFO_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 LinkAInt->rm_ee_sync_status_change_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR);
	 LinkAInt->rm_ee_num_los_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_NUM_LOS_DET_ERR);
	 LinkAInt->rm_ee_lcv_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_LCV_DET_ERR);
	 LinkAInt->rm_ee_frame_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_FRAME_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_block_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_BLOCK_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_missing_k28p5_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_MISSING_K28P5_ERR);
	 LinkAInt->rm_ee_missing_k28p7_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_MISSING_K28P7_ERR);
	 LinkAInt->rm_ee_k30p7_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_K30P7_DET_ERR);
	 LinkAInt->rm_ee_loc_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_LOC_DET_ERR);
	 LinkAInt->rm_ee_rx_fifo_ovf_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_RX_FIFO_OVF_ERR);
	 LinkAInt->rm_ee_rcvd_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_LOS_ERR);
	 LinkAInt->rm_ee_rcvd_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_LOF_ERR);
	 LinkAInt->rm_ee_rcvd_rai_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_RAI_ERR);
	 LinkAInt->rm_ee_rcvd_sdi_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_RCVD_SDI_ERR);
	 LinkAInt->rm_ee_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_LOS_ERR);
	 LinkAInt->rm_ee_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_LOF_ERR);
	 LinkAInt->rm_ee_hfnsync_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_HFNSYNC_STATE_ERR);
	 LinkAInt->rm_ee_lof_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_RM_EE_LOF_STATE_ERR);
	 LinkAInt->tm_ee_frm_misalign_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_TM_EE_FRM_MISALIGN_ERR);
	 LinkAInt->tm_ee_fifo_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_EV1, AIF2_EE_LK_EN_A_EV1_TM_EE_FIFO_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 LinkAInt->rm_ee_sync_status_change_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR);
	 LinkAInt->rm_ee_num_los_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_NUM_LOS_DET_ERR);
	 LinkAInt->rm_ee_lcv_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LCV_DET_ERR);
	 LinkAInt->rm_ee_frame_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_FRAME_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_block_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_BLOCK_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_missing_k28p5_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_MISSING_K28P5_ERR);
	 LinkAInt->rm_ee_missing_k28p7_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_MISSING_K28P7_ERR);
	 LinkAInt->rm_ee_k30p7_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_K30P7_DET_ERR);
	 LinkAInt->rm_ee_loc_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOC_DET_ERR);
	 LinkAInt->rm_ee_rx_fifo_ovf_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RX_FIFO_OVF_ERR);
	 LinkAInt->rm_ee_rcvd_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_LOS_ERR);
	 LinkAInt->rm_ee_rcvd_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_LOF_ERR);
	 LinkAInt->rm_ee_rcvd_rai_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_RAI_ERR);
	 LinkAInt->rm_ee_rcvd_sdi_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_RCVD_SDI_ERR);
	 LinkAInt->rm_ee_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOS_ERR);
	 LinkAInt->rm_ee_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOF_ERR);
	 LinkAInt->rm_ee_hfnsync_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_HFNSYNC_STATE_ERR);
	 LinkAInt->rm_ee_lof_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_RM_EE_LOF_STATE_ERR);
	 LinkAInt->tm_ee_frm_misalign_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_TM_EE_FRM_MISALIGN_ERR);
	 LinkAInt->tm_ee_fifo_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV0, AIF2_EE_LK_EN_STS_A_EV0_TM_EE_FIFO_STARVE_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 LinkAInt->rm_ee_sync_status_change_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR);
	 LinkAInt->rm_ee_num_los_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_NUM_LOS_DET_ERR);
	 LinkAInt->rm_ee_lcv_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LCV_DET_ERR);
	 LinkAInt->rm_ee_frame_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_FRAME_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_block_bndry_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_BLOCK_BNDRY_DET_ERR);
	 LinkAInt->rm_ee_missing_k28p5_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_MISSING_K28P5_ERR);
	 LinkAInt->rm_ee_missing_k28p7_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_MISSING_K28P7_ERR);
	 LinkAInt->rm_ee_k30p7_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_K30P7_DET_ERR);
	 LinkAInt->rm_ee_loc_det_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOC_DET_ERR);
	 LinkAInt->rm_ee_rx_fifo_ovf_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RX_FIFO_OVF_ERR);
	 LinkAInt->rm_ee_rcvd_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_LOS_ERR);
	 LinkAInt->rm_ee_rcvd_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_LOF_ERR);
	 LinkAInt->rm_ee_rcvd_rai_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_RAI_ERR);
	 LinkAInt->rm_ee_rcvd_sdi_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_RCVD_SDI_ERR);
	 LinkAInt->rm_ee_los_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOS_ERR);
	 LinkAInt->rm_ee_lof_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOF_ERR);
	 LinkAInt->rm_ee_hfnsync_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_HFNSYNC_STATE_ERR);
	 LinkAInt->rm_ee_lof_state_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_RM_EE_LOF_STATE_ERR);
	 LinkAInt->tm_ee_frm_misalign_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_TM_EE_FRM_MISALIGN_ERR);
	 LinkAInt->tm_ee_fifo_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_A_EV1, AIF2_EE_LK_EN_STS_A_EV1_TM_EE_FIFO_STARVE_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeLinkBIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE Link B interrupt  status value 
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function and hAif2->arg_link to select link
           Aif2Fl_EeLinkBInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_LK_IRS_B_PD_EE_EOP_ERR,                           AIF2_EE_LK_IRS_B_PD_EE_CRC_ERR,
 *         AIF2_EE_LK_IRS_B_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_IRS_B_PD_EE_AXC_FAIL_ERR,
 *         AIF2_EE_LK_IRS_B_PD_EE_SOP_ERR,                           AIF2_EE_LK_IRS_B_PD_EE_OBSAI_FRM_ERR,
 *         AIF2_EE_LK_IRS_B_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_IRS_B_PE_EE_MODRULE_ERR,
 *         AIF2_EE_LK_IRS_B_PE_EE_SYM_ERR,                           AIF2_EE_LK_IRS_B_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *         AIF2_EE_LK_IRS_B_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_IRS_B_PE_EE_DB_STARVE_ERR,
 *         AIF2_EE_LK_IRS_B_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_IRS_B_PE_EE_PKT_STARVE_ERR,
 *         AIF2_EE_LK_IRS_B_RT_EE_FRM_ERR,                           AIF2_EE_LK_IRS_B_RT_EE_OVFL_ERR,
 *         AIF2_EE_LK_IRS_B_RT_EE_UNFL_ERR,                          AIF2_EE_LK_IRS_B_RT_EE_EM_ERR,
 *         AIF2_EE_LK_IRS_B_RT_EE_HDR_ERR;                                 
 *
 *         AIF2_EE_LK_EN_B_EV0_PD_EE_EOP_ERR,                          AIF2_EE_LK_EN_B_EV0_PD_EE_CRC_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PD_EE_CPRI_FRAME_ERR,             AIF2_EE_LK_EN_B_EV0_PD_EE_AXC_FAIL_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PD_EE_SOP_ERR,                          AIF2_EE_LK_EN_B_EV0_PD_EE_OBSAI_FRM_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PD_EE_WR2DB_ERR,                     AIF2_EE_LK_EN_B_EV0_PE_EE_MODRULE_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_EV0_PE_EE_DB_STARVE_ERR,
 *         AIF2_EE_LK_EN_B_EV0_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_B_EV0_PE_EE_PKT_STARVE_ERR,
 *         AIF2_EE_LK_EN_B_EV0_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_B_EV0_RT_EE_OVFL_ERR,
 *         AIF2_EE_LK_EN_B_EV0_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_B_EV0_RT_EE_EM_ERR,
 *         AIF2_EE_LK_EN_B_EV0_RT_EE_HDR_ERR;                                 
 *
 *         AIF2_EE_LK_EN_B_EV1_PD_EE_EOP_ERR,                           AIF2_EE_LK_EN_B_EV1_PD_EE_CRC_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_B_EV1_PD_EE_AXC_FAIL_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PD_EE_SOP_ERR,                           AIF2_EE_LK_EN_B_EV1_PD_EE_OBSAI_FRM_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_EN_B_EV1_PE_EE_MODRULE_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_EV1_PE_EE_DB_STARVE_ERR,
 *         AIF2_EE_LK_EN_B_EV1_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_B_EV1_PE_EE_PKT_STARVE_ERR,
 *         AIF2_EE_LK_EN_B_EV1_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_B_EV1_RT_EE_OVFL_ERR,
 *         AIF2_EE_LK_EN_B_EV1_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_B_EV1_RT_EE_EM_ERR,
 *         AIF2_EE_LK_EN_B_EV1_RT_EE_HDR_ERR;      
 *
 *         AIF2_EE_LK_EN_STS_B_EV0_PD_EE_EOP_ERR,                           AIF2_EE_LK_EN_STS_B_EV0_PD_EE_CRC_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_STS_B_EV0_PD_EE_AXC_FAIL_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PD_EE_SOP_ERR,                           AIF2_EE_LK_EN_STS_B_EV0_PD_EE_OBSAI_FRM_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MODRULE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_DB_STARVE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_STS_B_EV0_PE_EE_PKT_STARVE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_STS_B_EV0_RT_EE_OVFL_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_STS_B_EV0_RT_EE_EM_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV0_RT_EE_HDR_ERR;                                 
 *
 *         AIF2_EE_LK_EN_STS_B_EV1_PD_EE_EOP_ERR,                           AIF2_EE_LK_EN_STS_B_EV1_PD_EE_CRC_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_STS_B_EV1_PD_EE_AXC_FAIL_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PD_EE_SOP_ERR,                           AIF2_EE_LK_EN_STS_B_EV1_PD_EE_OBSAI_FRM_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MODRULE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_DB_STARVE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_STS_B_EV1_PE_EE_PKT_STARVE_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_STS_B_EV1_RT_EE_OVFL_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_STS_B_EV1_RT_EE_EM_ERR,
 *         AIF2_EE_LK_EN_STS_B_EV1_RT_EE_HDR_ERR;      
 *   @b Example
 *   @verbatim
        Aif2Fl_EeLinkBInt  LinkBInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        hAif2->arg_link = 0;//link0
        
        Aif2Fl_getEeLinkBIntStatus(hAif2,&LinkBInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeLinkBIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeLinkBInt*     LinkBInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 LinkBInt->pd_ee_eop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_EOP_ERR);
	 LinkBInt->pd_ee_crc_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_CRC_ERR);
	 LinkBInt->pd_ee_cpri_frame_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_CPRI_FRAME_ERR);
	 LinkBInt->pd_ee_axc_fail_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_AXC_FAIL_ERR);
	 LinkBInt->pd_ee_sop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_SOP_ERR);
	 LinkBInt->pd_ee_obsai_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_OBSAI_FRM_ERR);
	 LinkBInt->pd_ee_wr2db_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PD_EE_WR2DB_ERR);
	 LinkBInt->pe_ee_modrule_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_MODRULE_ERR);
	 LinkBInt->pe_ee_sym_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_SYM_ERR);
	 LinkBInt->pe_ee_mf_fifo_overflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_MF_FIFO_OVERFLOW_ERR);
	 LinkBInt->pe_ee_mf_fifo_underflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_MF_FIFO_UNDERFLOW_ERR);
	 LinkBInt->pe_ee_db_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_DB_STARVE_ERR);
	 LinkBInt->pe_ee_rt_if_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_RT_IF_ERR);
	 LinkBInt->pe_ee_pkt_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_PE_EE_PKT_STARVE_ERR);
	 LinkBInt->rt_ee_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_RT_EE_FRM_ERR);
	 LinkBInt->rt_ee_ovfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_RT_EE_OVFL_ERR);
	 LinkBInt->rt_ee_unfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_RT_EE_UNFL_ERR);
	 LinkBInt->rt_ee_em_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_RT_EE_EM_ERR);
	 LinkBInt->rt_ee_hdr_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_B, AIF2_EE_LK_IRS_B_RT_EE_HDR_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 LinkBInt->pd_ee_eop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_EOP_ERR);
	 LinkBInt->pd_ee_crc_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_CRC_ERR);
	 LinkBInt->pd_ee_cpri_frame_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_CPRI_FRAME_ERR);
	 LinkBInt->pd_ee_axc_fail_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_AXC_FAIL_ERR);
	 LinkBInt->pd_ee_sop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_SOP_ERR);
	 LinkBInt->pd_ee_obsai_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_OBSAI_FRM_ERR);
	 LinkBInt->pd_ee_wr2db_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PD_EE_WR2DB_ERR);
	 LinkBInt->pe_ee_modrule_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_MODRULE_ERR);
	 LinkBInt->pe_ee_sym_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_SYM_ERR);
	 LinkBInt->pe_ee_mf_fifo_overflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR);
	 LinkBInt->pe_ee_mf_fifo_underflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR);
	 LinkBInt->pe_ee_db_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_DB_STARVE_ERR);
	 LinkBInt->pe_ee_rt_if_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_RT_IF_ERR);
	 LinkBInt->pe_ee_pkt_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_PE_EE_PKT_STARVE_ERR);
	 LinkBInt->rt_ee_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_RT_EE_FRM_ERR);
	 LinkBInt->rt_ee_ovfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_RT_EE_OVFL_ERR);
	 LinkBInt->rt_ee_unfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_RT_EE_UNFL_ERR);
	 LinkBInt->rt_ee_em_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_RT_EE_EM_ERR);
	 LinkBInt->rt_ee_hdr_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV0, AIF2_EE_LK_EN_B_EV0_RT_EE_HDR_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 LinkBInt->pd_ee_eop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_EOP_ERR);
	 LinkBInt->pd_ee_crc_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_CRC_ERR);
	 LinkBInt->pd_ee_cpri_frame_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_CPRI_FRAME_ERR);
	 LinkBInt->pd_ee_axc_fail_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_AXC_FAIL_ERR);
	 LinkBInt->pd_ee_sop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_SOP_ERR);
	 LinkBInt->pd_ee_obsai_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_OBSAI_FRM_ERR);
	 LinkBInt->pd_ee_wr2db_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PD_EE_WR2DB_ERR);
	 LinkBInt->pe_ee_modrule_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_MODRULE_ERR);
	 LinkBInt->pe_ee_sym_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_SYM_ERR);
	 LinkBInt->pe_ee_mf_fifo_overflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR);
	 LinkBInt->pe_ee_mf_fifo_underflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR);
	 LinkBInt->pe_ee_db_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_DB_STARVE_ERR);
	 LinkBInt->pe_ee_rt_if_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_RT_IF_ERR);
	 LinkBInt->pe_ee_pkt_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_PE_EE_PKT_STARVE_ERR);
	 LinkBInt->rt_ee_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_RT_EE_FRM_ERR);
	 LinkBInt->rt_ee_ovfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_RT_EE_OVFL_ERR);
	 LinkBInt->rt_ee_unfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_RT_EE_UNFL_ERR);
	 LinkBInt->rt_ee_em_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_RT_EE_EM_ERR);
	 LinkBInt->rt_ee_hdr_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_EV1, AIF2_EE_LK_EN_B_EV1_RT_EE_HDR_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 LinkBInt->pd_ee_eop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_EOP_ERR);
	 LinkBInt->pd_ee_crc_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_CRC_ERR);
	 LinkBInt->pd_ee_cpri_frame_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_CPRI_FRAME_ERR);
	 LinkBInt->pd_ee_axc_fail_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_AXC_FAIL_ERR);
	 LinkBInt->pd_ee_sop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_SOP_ERR);
	 LinkBInt->pd_ee_obsai_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_OBSAI_FRM_ERR);
	 LinkBInt->pd_ee_wr2db_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PD_EE_WR2DB_ERR);
	 LinkBInt->pe_ee_modrule_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MODRULE_ERR);
	 LinkBInt->pe_ee_sym_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_SYM_ERR);
	 LinkBInt->pe_ee_mf_fifo_overflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR);
	 LinkBInt->pe_ee_mf_fifo_underflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR);
	 LinkBInt->pe_ee_db_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_DB_STARVE_ERR);
	 LinkBInt->pe_ee_rt_if_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_RT_IF_ERR);
	 LinkBInt->pe_ee_pkt_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_PE_EE_PKT_STARVE_ERR);
	 LinkBInt->rt_ee_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_RT_EE_FRM_ERR);
	 LinkBInt->rt_ee_ovfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_RT_EE_OVFL_ERR);
	 LinkBInt->rt_ee_unfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_RT_EE_UNFL_ERR);
	 LinkBInt->rt_ee_em_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_RT_EE_EM_ERR);
	 LinkBInt->rt_ee_hdr_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV0, AIF2_EE_LK_EN_STS_B_EV0_RT_EE_HDR_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 LinkBInt->pd_ee_eop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_EOP_ERR);
	 LinkBInt->pd_ee_crc_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_CRC_ERR);
	 LinkBInt->pd_ee_cpri_frame_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_CPRI_FRAME_ERR);
	 LinkBInt->pd_ee_axc_fail_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_AXC_FAIL_ERR);
	 LinkBInt->pd_ee_sop_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_SOP_ERR);
	 LinkBInt->pd_ee_obsai_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_OBSAI_FRM_ERR);
	 LinkBInt->pd_ee_wr2db_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PD_EE_WR2DB_ERR);
	 LinkBInt->pe_ee_modrule_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MODRULE_ERR);
	 LinkBInt->pe_ee_sym_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_SYM_ERR);
	 LinkBInt->pe_ee_mf_fifo_overflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR);
	 LinkBInt->pe_ee_mf_fifo_underflow_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR);
	 LinkBInt->pe_ee_db_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_DB_STARVE_ERR);
	 LinkBInt->pe_ee_rt_if_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_RT_IF_ERR);
	 LinkBInt->pe_ee_pkt_starve_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_PE_EE_PKT_STARVE_ERR);
	 LinkBInt->rt_ee_frm_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_RT_EE_FRM_ERR);
	 LinkBInt->rt_ee_ovfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_RT_EE_OVFL_ERR);
	 LinkBInt->rt_ee_unfl_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_RT_EE_UNFL_ERR);
	 LinkBInt->rt_ee_em_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_RT_EE_EM_ERR);
	 LinkBInt->rt_ee_hdr_err=  CSL_FEXT(hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_STS_B_EV1, AIF2_EE_LK_EN_STS_B_EV1_RT_EE_HDR_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEeAtIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE AT interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EeAtInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_IRS_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_IRS_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_PI0_ERR,AIF2_EE_AT_IRS_AT_EE_PI1_ERR,AIF2_EE_AT_IRS_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_PI3_ERR,AIF2_EE_AT_IRS_AT_EE_PI4_ERR,AIF2_EE_AT_IRS_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_IRS_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_IRS_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_EV0_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_EV0_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_PI0_ERR,AIF2_EE_AT_EN_EV0_AT_EE_PI1_ERR,AIF2_EE_AT_EN_EV0_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_PI3_ERR,AIF2_EE_AT_EN_EV0_AT_EE_PI4_ERR,AIF2_EE_AT_EN_EV0_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_EV0_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_EV0_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_EV1_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_EV1_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_PI0_ERR,AIF2_EE_AT_EN_EV1_AT_EE_PI1_ERR,AIF2_EE_AT_EN_EV1_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_PI3_ERR,AIF2_EE_AT_EN_EV1_AT_EE_PI4_ERR,AIF2_EE_AT_EN_EV1_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_EV1_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_EV1_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_PI0_ERR,AIF2_EE_AT_EN_STS_EV0_AT_EE_PI1_ERR,AIF2_EE_AT_EN_STS_EV0_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_PI3_ERR,AIF2_EE_AT_EN_STS_EV0_AT_EE_PI4_ERR,AIF2_EE_AT_EN_STS_EV0_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_STS_EV0_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_STS_EV0_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_PI0_ERR,AIF2_EE_AT_EN_STS_EV1_AT_EE_PI1_ERR,AIF2_EE_AT_EN_STS_EV1_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_PI3_ERR,AIF2_EE_AT_EN_STS_EV1_AT_EE_PI4_ERR,AIF2_EE_AT_EN_STS_EV1_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_STS_EV1_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_STS_EV1_AT_EE_RADT_SYNC_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAtInt  AtInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEeAtIntStatus(hAif2,&AtInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEeAtIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EeAtInt*     AtInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 AtInt->at_ee_rp1_type_sys_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_SYS_RCVD_ERR);
	 AtInt->at_ee_rp1_type_rp3_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_RP3_RCVD_ERR);
	 AtInt->at_ee_rp1_type_tod_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_TOD_RCVD_ERR);
	 AtInt->at_ee_rp1_type_unsel_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_UNSEL_ERR);
	 AtInt->at_ee_rp1_type_spare_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_SPARE_ERR);
	 AtInt->at_ee_rp1_type_rsvd_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_TYPE_RSVD_ERR);
	 AtInt->at_ee_rp1_bit_width_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_BIT_WIDTH_ERR);
	 AtInt->at_ee_rp1_crc_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_CRC_ERR);
	 AtInt->at_ee_rp1_rp3_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_RP3_ERR);
	 AtInt->at_ee_rp1_sys_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RP1_SYS_ERR);
	 AtInt->at_ee_pi0_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI0_ERR);
	 AtInt->at_ee_pi1_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI1_ERR);
	 AtInt->at_ee_pi2_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI2_ERR);
	 AtInt->at_ee_pi3_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI3_ERR);
	 AtInt->at_ee_pi4_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI4_ERR);
	 AtInt->at_ee_pi5_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PI5_ERR);
	 AtInt->at_ee_phyt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_PHYT_SYNC_ERR);
	 AtInt->at_ee_radt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_IRS, AIF2_EE_AT_IRS_AT_EE_RADT_SYNC_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 AtInt->at_ee_rp1_type_sys_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR);
	 AtInt->at_ee_rp1_type_rp3_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR);
	 AtInt->at_ee_rp1_type_tod_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR);
	 AtInt->at_ee_rp1_type_unsel_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_UNSEL_ERR);
	 AtInt->at_ee_rp1_type_spare_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_SPARE_ERR);
	 AtInt->at_ee_rp1_type_rsvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_TYPE_RSVD_ERR);
	 AtInt->at_ee_rp1_bit_width_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_BIT_WIDTH_ERR);
	 AtInt->at_ee_rp1_crc_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_CRC_ERR);
	 AtInt->at_ee_rp1_rp3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_RP3_ERR);
	 AtInt->at_ee_rp1_sys_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RP1_SYS_ERR);
	 AtInt->at_ee_pi0_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI0_ERR);
	 AtInt->at_ee_pi1_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI1_ERR);
	 AtInt->at_ee_pi2_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI2_ERR);
	 AtInt->at_ee_pi3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI3_ERR);
	 AtInt->at_ee_pi4_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI4_ERR);
	 AtInt->at_ee_pi5_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PI5_ERR);
	 AtInt->at_ee_phyt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_PHYT_SYNC_ERR);
	 AtInt->at_ee_radt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV0, AIF2_EE_AT_EN_EV0_AT_EE_RADT_SYNC_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 AtInt->at_ee_rp1_type_sys_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR);
	 AtInt->at_ee_rp1_type_rp3_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR);
	 AtInt->at_ee_rp1_type_tod_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR);
	 AtInt->at_ee_rp1_type_unsel_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_UNSEL_ERR);
	 AtInt->at_ee_rp1_type_spare_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_SPARE_ERR);
	 AtInt->at_ee_rp1_type_rsvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_TYPE_RSVD_ERR);
	 AtInt->at_ee_rp1_bit_width_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_BIT_WIDTH_ERR);
	 AtInt->at_ee_rp1_crc_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_CRC_ERR);
	 AtInt->at_ee_rp1_rp3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_RP3_ERR);
	 AtInt->at_ee_rp1_sys_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RP1_SYS_ERR);
	 AtInt->at_ee_pi0_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI0_ERR);
	 AtInt->at_ee_pi1_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI1_ERR);
	 AtInt->at_ee_pi2_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI2_ERR);
	 AtInt->at_ee_pi3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI3_ERR);
	 AtInt->at_ee_pi4_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI4_ERR);
	 AtInt->at_ee_pi5_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PI5_ERR);
	 AtInt->at_ee_phyt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_PHYT_SYNC_ERR);
	 AtInt->at_ee_radt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_EV1, AIF2_EE_AT_EN_EV1_AT_EE_RADT_SYNC_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 AtInt->at_ee_rp1_type_sys_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR);
	 AtInt->at_ee_rp1_type_rp3_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR);
	 AtInt->at_ee_rp1_type_tod_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR);
	 AtInt->at_ee_rp1_type_unsel_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_UNSEL_ERR);
	 AtInt->at_ee_rp1_type_spare_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_SPARE_ERR);
	 AtInt->at_ee_rp1_type_rsvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_TYPE_RSVD_ERR);
	 AtInt->at_ee_rp1_bit_width_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_BIT_WIDTH_ERR);
	 AtInt->at_ee_rp1_crc_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_CRC_ERR);
	 AtInt->at_ee_rp1_rp3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_RP3_ERR);
	 AtInt->at_ee_rp1_sys_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RP1_SYS_ERR);
	 AtInt->at_ee_pi0_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI0_ERR);
	 AtInt->at_ee_pi1_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI1_ERR);
	 AtInt->at_ee_pi2_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI2_ERR);
	 AtInt->at_ee_pi3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI3_ERR);
	 AtInt->at_ee_pi4_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI4_ERR);
	 AtInt->at_ee_pi5_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PI5_ERR);
	 AtInt->at_ee_phyt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_PHYT_SYNC_ERR);
	 AtInt->at_ee_radt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV0, AIF2_EE_AT_EN_STS_EV0_AT_EE_RADT_SYNC_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 AtInt->at_ee_rp1_type_sys_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR);
	 AtInt->at_ee_rp1_type_rp3_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR);
	 AtInt->at_ee_rp1_type_tod_rcvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR);
	 AtInt->at_ee_rp1_type_unsel_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_UNSEL_ERR);
	 AtInt->at_ee_rp1_type_spare_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_SPARE_ERR);
	 AtInt->at_ee_rp1_type_rsvd_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_TYPE_RSVD_ERR);
	 AtInt->at_ee_rp1_bit_width_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_BIT_WIDTH_ERR);
	 AtInt->at_ee_rp1_crc_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_CRC_ERR);
	 AtInt->at_ee_rp1_rp3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_RP3_ERR);
	 AtInt->at_ee_rp1_sys_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RP1_SYS_ERR);
	 AtInt->at_ee_pi0_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI0_ERR);
	 AtInt->at_ee_pi1_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI1_ERR);
	 AtInt->at_ee_pi2_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI2_ERR);
	 AtInt->at_ee_pi3_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI3_ERR);
	 AtInt->at_ee_pi4_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI4_ERR);
	 AtInt->at_ee_pi5_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PI5_ERR);
	 AtInt->at_ee_phyt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_PHYT_SYNC_ERR);
	 AtInt->at_ee_radt_sync_err=  CSL_FEXT(hAif2->regs->EE_AT_EN_STS_EV1, AIF2_EE_AT_EN_STS_EV1_AT_EE_RADT_SYNC_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEePdIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE PD common interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EePdInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_PD_COMMON_IRS_PD_EE_TS_WDOG_ERR;AIF2_EE_PD_COMMON_EN_EV0_PD_EE_TS_WDOG_ERR;
 *         AIF2_EE_PD_COMMON_EN_EV1_PD_EE_TS_WDOG_ERR;AIF2_EE_PD_COMMON_EN_STS_EV0_PD_EE_TS_WDOG_ERR;
 *         AIF2_EE_PD_COMMON_EN_STS_EV1_PD_EE_TS_WDOG_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EePdInt  PdInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEePdIntStatus(hAif2,&PdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEePdIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EePdInt*     PdInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 PdInt->pd_ee_ts_wdog_err=  CSL_FEXT(hAif2->regs->EE_PD_COMMON_IRS, AIF2_EE_PD_COMMON_IRS_PD_EE_TS_WDOG_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 PdInt->pd_ee_ts_wdog_err=  CSL_FEXT(hAif2->regs->EE_PD_COMMON_EN_EV0, AIF2_EE_PD_COMMON_EN_EV0_PD_EE_TS_WDOG_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 PdInt->pd_ee_ts_wdog_err=  CSL_FEXT(hAif2->regs->EE_PD_COMMON_EN_EV1, AIF2_EE_PD_COMMON_EN_EV1_PD_EE_TS_WDOG_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 PdInt->pd_ee_ts_wdog_err=  CSL_FEXT(hAif2->regs->EE_PD_COMMON_EN_STS_EV0, AIF2_EE_PD_COMMON_EN_STS_EV0_PD_EE_TS_WDOG_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 PdInt->pd_ee_ts_wdog_err=  CSL_FEXT(hAif2->regs->EE_PD_COMMON_EN_STS_EV1, AIF2_EE_PD_COMMON_EN_STS_EV1_PD_EE_TS_WDOG_ERR);
        }
	 
}


/** ============================================================================
 *   @n@b Aif2Fl_getEePeIntStatus
 *
 *   @b Description
 *   @n Get Aif2 EE PE common interrupt  status value
 *
 *   @b Arguments
 *   @verbatim
           hAif2      Handle to the aif2 instance     use hAif2->ee_arg to select function
           Aif2Fl_EePeInt     
      @endverbatim
 *
 *   <b> Return Value </b>  void
 *   <b> Pre Condition </b>
 *   @n  Both @a Aif2Fl_init() and @a Aif2Fl_open() must be called successfully
 *   in that order before calling this function.
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Reads
 *   @n  AIF2_EE_PE_COMMON_IRS_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_IRS_PE_EE_TOKEN_REQ_OVFL_ERR,
 *         AIF2_EE_PE_COMMON_IRS_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_IRS_PE_EE_DAT_REQ_OVFL_ERR;
 *         AIF2_EE_PE_COMMON_EN_EV0_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_EV0_PE_EE_TOKEN_REQ_OVFL_ERR,
 *         AIF2_EE_PE_COMMON_EN_EV0_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_EV0_PE_EE_DAT_REQ_OVFL_ERR;
 *         AIF2_EE_PE_COMMON_EN_EV1_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_EV1_PE_EE_TOKEN_REQ_OVFL_ERR,
 *         AIF2_EE_PE_COMMON_EN_EV1_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_EV1_PE_EE_DAT_REQ_OVFL_ERR;         
 *        AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_DAT_REQ_OVFL_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EePeInt  PeInt;
        hAif2->ee_arg = AIF2FL_EE_INT_RAW_STATUS;
        
        Aif2Fl_getEePeIntStatus(hAif2,&PeInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_getEePeIntStatus (
        Aif2Fl_Handle   hAif2,
        Aif2Fl_EePeInt*     PeInt
)
{ 
        if(hAif2->ee_arg == AIF2FL_EE_INT_RAW_STATUS){
	 PeInt->pe_ee_rd2db_err =               CSL_FEXT(hAif2->regs->EE_PE_COMMON_IRS, AIF2_EE_PE_COMMON_IRS_PE_EE_RD2DB_ERR);
	 PeInt->pe_ee_token_req_ovfl_err =  CSL_FEXT(hAif2->regs->EE_PE_COMMON_IRS, AIF2_EE_PE_COMMON_IRS_PE_EE_TOKEN_REQ_OVFL_ERR);
	 PeInt->pe_ee_token_wr_err =          CSL_FEXT(hAif2->regs->EE_PE_COMMON_IRS, AIF2_EE_PE_COMMON_IRS_PE_EE_TOKEN_WR_ERR);
	 PeInt->pe_ee_dat_req_ovfl_err =     CSL_FEXT(hAif2->regs->EE_PE_COMMON_IRS, AIF2_EE_PE_COMMON_IRS_PE_EE_DAT_REQ_OVFL_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV0){
	 PeInt->pe_ee_rd2db_err =               CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV0, AIF2_EE_PE_COMMON_EN_EV0_PE_EE_RD2DB_ERR);
	 PeInt->pe_ee_token_req_ovfl_err =  CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV0, AIF2_EE_PE_COMMON_EN_EV0_PE_EE_TOKEN_REQ_OVFL_ERR);
	 PeInt->pe_ee_token_wr_err =          CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV0, AIF2_EE_PE_COMMON_EN_EV0_PE_EE_TOKEN_WR_ERR);
	 PeInt->pe_ee_dat_req_ovfl_err =     CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV0, AIF2_EE_PE_COMMON_EN_EV0_PE_EE_DAT_REQ_OVFL_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_EV1){
	 PeInt->pe_ee_rd2db_err =               CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV1, AIF2_EE_PE_COMMON_EN_EV1_PE_EE_RD2DB_ERR);
	 PeInt->pe_ee_token_req_ovfl_err =  CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV1, AIF2_EE_PE_COMMON_EN_EV1_PE_EE_TOKEN_REQ_OVFL_ERR);
	 PeInt->pe_ee_token_wr_err =          CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV1, AIF2_EE_PE_COMMON_EN_EV1_PE_EE_TOKEN_WR_ERR);
	 PeInt->pe_ee_dat_req_ovfl_err =     CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_EV1, AIF2_EE_PE_COMMON_EN_EV1_PE_EE_DAT_REQ_OVFL_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV0){
	 PeInt->pe_ee_rd2db_err =               CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV0, AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_RD2DB_ERR);
	 PeInt->pe_ee_token_req_ovfl_err =  CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV0, AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_TOKEN_REQ_OVFL_ERR);
	 PeInt->pe_ee_token_wr_err =          CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV0, AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_TOKEN_WR_ERR);
	 PeInt->pe_ee_dat_req_ovfl_err =     CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV0, AIF2_EE_PE_COMMON_EN_STS_EV0_PE_EE_DAT_REQ_OVFL_ERR);
        }
	 else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_STATUS_EV1){
	 PeInt->pe_ee_rd2db_err =               CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV1, AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_RD2DB_ERR);
	 PeInt->pe_ee_token_req_ovfl_err =  CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV1, AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_TOKEN_REQ_OVFL_ERR);
	 PeInt->pe_ee_token_wr_err =          CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV1, AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_TOKEN_WR_ERR);
	 PeInt->pe_ee_dat_req_ovfl_err =     CSL_FEXT(hAif2->regs->EE_PE_COMMON_EN_STS_EV1, AIF2_EE_PE_COMMON_EN_STS_EV1_PE_EE_DAT_REQ_OVFL_ERR);
        }
	 
}



#ifdef __cplusplus
}
#endif
#endif /* Aif2Fl_getHwStatusAUX_H_ */


