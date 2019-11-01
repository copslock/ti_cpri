/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008,  2009
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
 *   @file  aif2fl_getHwStatus.c
 *
 *
 *   @brief  Antenna Interface 2 get hardware status function
 *
 */


/* =============================================================================
 * Revision History
 * ===============
 *  03-Jun-2009  Albert  File Created.
 *  06-June-2015 Seb    File imported in the driver
 *  
 *
 * =============================================================================
 */

#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_getHwStatusAux.h>

/** ============================================================================
 *   @n@b Aif2Fl_getHwStatus
 *
 *   @b Description
 *   @n This function is used to get the value of various parameters of the
 *      aif2 instance. The value returned depends on the query passed.
 *
 *   @b Arguments
 *   @verbatim
            hAif2      Handle to the aif2 instance
 
            Query         Query to be performed 
 
            response      Pointer to buffer to return the data requested by
                          the query passed
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *   @li                    AIF2FL_SOK            - Successful completion of the
 *                                               query
 *
 *   @li                    AIF2FL_BADHANDLE - Invalid handle
 *
 *   @li                    AIF2FL_INVQUERY  - Query command not supported
 *
 *   @li                    AIF2FL_FAIL      - Generic failure
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *       Data requested by query is returned through the variable "response"
 *
 *   @b Writes
 *   @n The input argument "response" is modified
 *
 *   @b Example
 *   @verbatim
        
        #define AIF2FL_MAX_RX_MASTER_FRAME_OFFSET 		256   
 
        Aif2Fl_Handle hAif2;
        // other link related declarations
        ...
        // ctrl argument for hw command
        uint16_t ctrlArg;
        // query response
        uint16_t response;

        // Open handle  - for use 
        hAif2 = Aif2Fl_open(&Aif2Obj, 0, &aif2Param, &status);

        if ((hAif2 == NULL) || (status != AIF2FL_SOK)) 
        {
           printf ("\nError opening AIF2FL");
           exit(1);
        }

        // Do config for link 0 
        Config.globalSetup = &gblCfg;
        ...
         
        //Do setup for link - 0
        Aif2Fl_hwSetup(hAif2, &Config);

        ctrlArg = AIF2FL_CTRL_RX_LINK_ENABLE;
        hAif2->arg_link = AIF2FL_LINK_0; //link 0 enable
        
        // Send hw control command to enable Tx/Rx of link 0 
        Aif2Fl_hwControl(hAif2, AIF2FL_CMD_ENABLE_DISABLE_RX_LINK, (void *)&ctrlArg);
        ...
        wait(100); //wait for a sufficient length of time, so Rx link has enough 
                     time to sync; 100 cycles wait time is arbitrarily chosen 
    
        // Get status of Rx master frame offset  
        hAif2->arg_link = AIF2FL_LINK_0; //get link 0 status
        Aif2Fl_getHwStatus(hAif2, AIF2FL_QUERY_RM_LINK_RCVD_MSTR_FRAME_OFFSET, (void *)&response);
   
        if (response > AIF2FL_MAX_RX_MASTER_FRAME_OFFSET)
        {
           printf("\nRx Master Frame Offset exceeds bounds");
        }

     @endverbatim
 * =============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_getHwStatus, ".text:aif2fl");
#endif
Aif2Fl_Status  Aif2Fl_getHwStatus(
   Aif2Fl_Handle                      hAif2,
   Aif2Fl_HwStatusQuery               Query,
   void                               *response
)
{
    Aif2Fl_Status status = AIF2FL_SOK;
    
    if (hAif2 == NULL)
        return AIF2FL_BADHANDLE;
    
    switch (Query) {
    
    /** returns VC AIF2 version */
    case AIF2FL_QUERY_VERSION:
        Aif2Fl_getVersion(hAif2, (Aif2Fl_PidStatus *)response);
        break;

     /** returns VC Emu status */
    case AIF2FL_QUERY_VC_STAT:
        *((uint16_t *)response) = Aif2Fl_getVcStat(hAif2);
        break;
		
    /* SERDES Rx link Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_SdRxStatus *) */
    case AIF2FL_QUERY_SD_RX_LINK_STATUS:
    	 Aif2Fl_getSdRxLinkStatus (hAif2, (Aif2Fl_SdRxStatus *)response);  
        break;
#ifndef K2
	/* SERDES Tx link Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_SdTxStatus *)*/
	case AIF2FL_QUERY_SD_TX_LINK_STATUS:
		 Aif2Fl_getSdTxLinkStatus (hAif2, (Aif2Fl_SdTxStatus *)response);
		break;
#endif
    /* Return the status of SERDES B8 PLL lock.   @param (uint8_t *)*/
    case AIF2FL_QUERY_SD_B8_PLL_LOCK:
        *((uint8_t *)response) = Aif2Fl_getSdB8PllLock (hAif2);  
        break;
		
    /* Return the status of SERDES B4 PLL lock.   @param (uint8_t *)*/
    case AIF2FL_QUERY_SD_B4_PLL_LOCK:
        *((uint8_t *)response) = Aif2Fl_getSdB4PllLock (hAif2);  
        break;

    /** RM link Status 0. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus0 *)*/
    case AIF2FL_QUERY_RM_LINK_STATUS_0:
    	Aif2Fl_getRmLinkStatus0  (hAif2, (Aif2Fl_RmStatus0 *)response);
        break;

    /** RM link Status 1. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus1 *)*/
    case AIF2FL_QUERY_RM_LINK_STATUS_1:
    	Aif2Fl_getRmLinkStatus1  (hAif2, (Aif2Fl_RmStatus1 *)response);
        break;

    /** RM link Status 2. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus2 *)*/
    case AIF2FL_QUERY_RM_LINK_STATUS_2:
    	Aif2Fl_getRmLinkStatus2  (hAif2, (Aif2Fl_RmStatus2 *)response);
        break;

     /** RM link Status 3. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus3 *)*/
    case AIF2FL_QUERY_RM_LINK_STATUS_3:
    	Aif2Fl_getRmLinkStatus3  (hAif2, (Aif2Fl_RmStatus3 *)response);
        break;

    /** RM link Status 4. use hAif2->arg_link to choose link.  @param (Aif2Fl_RmStatus4 *)*/
    case AIF2FL_QUERY_RM_LINK_STATUS_4:
    	Aif2Fl_getRmLinkStatus4  (hAif2, (Aif2Fl_RmStatus4 *)response);
        break;
		
    /** Return TM link CPRI HFN Status. use hAif2->arg_link to choose link.  @param (uint8_t *)*/
    case AIF2FL_QUERY_TM_LINK_CPRI_HFN:
    	*((uint8_t *)response) = Aif2Fl_getTmCpriHfnStatus (hAif2);  
        break;
		
     /** TM link Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_TmStatus *)*/
    case AIF2FL_QUERY_TM_LINK_STATUS:
    	Aif2Fl_getTmLinkStatus  (hAif2, (Aif2Fl_TmStatus *)response);
        break;

    /** RT Internal FIFO depth Status. use hAif2->arg_link to choose link.  @param (uint8_t *)*/
    case AIF2FL_QUERY_RT_FIFO_DEPTH_STATUS:
    	*((uint8_t *)response) = Aif2Fl_getRtFifoDepthStatus (hAif2);
        break;

    /** RT Header Error Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_RtHeaderStatus *)*/
    case AIF2FL_QUERY_RT_HEADER_ERROR_STATUS:
    	Aif2Fl_getRtHeaderStatus (hAif2, (Aif2Fl_RtHeaderStatus *)response);
        break;
		
    /** RT link Status. use hAif2->arg_link to choose link.  @param (Aif2Fl_RtStatus *)*/
    case AIF2FL_QUERY_RT_LINK_STATUS:
    	Aif2Fl_getRtLinkStatus (hAif2, (Aif2Fl_RtStatus *)response);
        break;
		
     /** PD 128 Channel Status.  @param (uint32_t *)*/
    case AIF2FL_QUERY_PD_CHANNEL_STATUS:
    	Aif2Fl_getPdChannelStatus  (hAif2, (uint32_t *)response);
        break;

     /** PD Packet Status for 128 channels if it is in or out of packet.  @param (uint32_t *)*/
    case AIF2FL_QUERY_PD_PACKET_STATUS:
    	Aif2Fl_getPdPacketStatus  (hAif2, (uint32_t *)response);
        break;

    /** PE 128 Channel Status.  @param (uint32_t *)*/
    case AIF2FL_QUERY_PE_CHANNEL_STATUS:
    	Aif2Fl_getPeChannelStatus  (hAif2, (uint32_t *)response);
        break;

     /** PE Packet Status for 128 channels if it is in or out of packet.  @param (uint32_t *)*/
    case AIF2FL_QUERY_PE_PACKET_STATUS:
    	Aif2Fl_getPePacketStatus  (hAif2, (uint32_t *)response);
        break;
		
    /* Get Write and Read Offset Value at address in DB_IDB_DEBUG_OFS @param (uint8_t *) */
    case AIF2FL_QUERY_DB_IN_DEBUG_OFFSET_DATA:
        Aif2Fl_getInDbDebugOffsetData (hAif2, (uint8_t *)response);
        break;

    /** Debug data written to bits 128:0 of Egress DB RAM  @param ( uint32_t *) */
    case AIF2FL_QUERY_DB_E_DEBUG_DATA:
        Aif2Fl_getEDbDebugData(hAif2, (uint32_t *)response);
        break;
		
    /** Egress DB debug side band data setup  @param ( Aif2Fl_DbSideData *)  */
    case AIF2FL_QUERY_DB_E_DEBUG_SIDE_DATA:
        Aif2Fl_getEDbDebugSideData(hAif2, (Aif2Fl_DbSideData *)response);
        break;
		
    /* Get Write and Read Offset Value at address in DB_EDB_DEBUG_OFS  @param (uint8_t *) */
    case AIF2FL_QUERY_DB_E_DEBUG_OFFSET_DATA:
        Aif2Fl_getEDbDebugOffsetData (hAif2,(uint8_t *)response);
        break;

    /* Get Egress EOP count value   @param (uint32_t *) */
    case AIF2FL_QUERY_DB_E_EOP_COUNT:
        *((uint32_t *)response) = Aif2Fl_getEgrEopCount(hAif2);
        break;

    /* Get Inress EOP count value   @param (uint32_t *) */
    case AIF2FL_QUERY_AD_I_EOP_COUNT:
        *((uint32_t *)response) = Aif2Fl_getIngrEopCount(hAif2);
        break;

    /* Get Aif2 timer link Pi captured value. use hAif2->arg_link to choose link   @param (uint32_t *) */
    case AIF2FL_QUERY_AT_LINK_PI_CAPTURE:
        *((uint32_t *)response) = Aif2Fl_getAtLinkPiCapture (hAif2);
        break;

   /* Get Aif2 timer capture Radt values @param (Aif2Fl_AtCaptRadt  *) */
    case AIF2FL_QUERY_AT_RADT_CAPTURE:
        Aif2Fl_getAtRadtCapture (hAif2, (Aif2Fl_AtCaptRadt *)response);
        break;
		
    /* Get Aif2 timer RP1 type capture value @param (uint8_t *) */
    case AIF2FL_QUERY_AT_RP1_TYPE_CAPTURE:
        *((uint8_t *)response) = Aif2Fl_getAtRp1TypeCapture (hAif2);
        break;

    /* Get Aif2 timer RP1 tod capture lsb value    @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1TodCaptureLsb (hAif2);
        break;

    /* Get Aif2 timer RP1 tod capture msb value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1TodCaptureMsb (hAif2);
        break;

    /* Get Aif2 timer RP1 rp3 capture lsb value    @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1Rp3CaptureLsb (hAif2);
        break;

    /* Get Aif2 timer RP1 rp3 capture msb value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1Rp3CaptureMsb (hAif2);
        break;

    /* Get Aif2 timer RP1 rad capture lsb value    @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1RadCaptureLsb (hAif2);
        break;

    /* Get Aif2 timer RP1 rad capture msb value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtRp1RadCaptureMsb (hAif2);
        break;
    /* Get Aif2 Phy timer clock count value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_PHY_CLOCK_COUNT:
        *((uint32_t *)response) = Aif2Fl_getAtPhytClkCount (hAif2);
        break;
		
    /* Get Aif2 Phy timer frame count value lsb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_PHY_FRAME_COUNT_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtPhytFrameLsb (hAif2);
        break;
		
    /* Get Aif2 Phy timer frame count value msb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_PHY_FRAME_COUNT_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtPhytFrameMsb (hAif2);
        break;

    /* Get Aif2 Rad timer clock count value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RAD_CLOCK_COUNT:
        *((uint32_t *)response) = Aif2Fl_getAtRadtClkCount (hAif2);
        break;

    /* Get Aif2 Rad timer symbol count value  @param (uint8_t *) */
    case AIF2FL_QUERY_AT_RAD_SYMBOL_COUNT:
        *((uint8_t *)response) = Aif2Fl_getAtRadtSymCount (hAif2);
        break;
		
    /* Get Aif2 Rad timer frame count value lsb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RAD_FRAME_COUNT_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtRadtFrameLsb (hAif2);
        break;
		
    /* Get Aif2 Rad timer frame count value msb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RAD_FRAME_COUNT_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtRadtFrameMsb (hAif2);
        break;

    /* Get Aif2 Ul Rad timer clock count value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_ULRAD_CLOCK_COUNT:
        *((uint32_t *)response) = Aif2Fl_getAtUlRadtClkCount (hAif2);
        break;

    /* Get Aif2 Ul Rad timer symbol count value  @param (uint8_t *) */
    case AIF2FL_QUERY_AT_ULRAD_SYMBOL_COUNT:
        *((uint8_t *)response) = Aif2Fl_getAtUlRadtSymCount (hAif2);
        break;
		
    /* Get Aif2 Ul Rad timer frame count value lsb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtUlRadtFrameLsb (hAif2);
        break;
		
    /* Get Aif2 Ul Rad timer frame count value msb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtUlRadtFrameMsb (hAif2);
        break;
		
    /* Get Aif2 Dl Rad timer clock count value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_DLRAD_CLOCK_COUNT:
        *((uint32_t *)response) = Aif2Fl_getAtDlRadtClkCount (hAif2);
        break;

    /* Get Aif2 Dl Rad timer symbol count value  @param (uint8_t *) */
    case AIF2FL_QUERY_AT_DLRAD_SYMBOL_COUNT:
        *((uint8_t *)response) = Aif2Fl_getAtDlRadtSymCount (hAif2);
        break;
		
    /* Get Aif2 Dl Rad timer frame count value lsb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_LSB:
        *((uint32_t *)response) = Aif2Fl_getAtDlRadtFrameLsb (hAif2);
        break;
		
    /* Get Aif2 Dl Rad timer frame count value msb @param (uint32_t *) */
    case AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_MSB:
        *((uint32_t *)response) = Aif2Fl_getAtDlRadtFrameMsb (hAif2);
        break;

    /* Get Aif2 Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
    case AIF2FL_QUERY_AT_RAD_WCDMA_VALUE:
        Aif2Fl_getAtRadtWcdmaValue (hAif2, (Aif2Fl_AtWcdmaCount *)response);
        break;

    /* Get Aif2 Ul Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
    case AIF2FL_QUERY_AT_ULRAD_WCDMA_VALUE:
        Aif2Fl_getAtUlRadtWcdmaValue (hAif2, (Aif2Fl_AtWcdmaCount *)response);
        break;

    /* Get Aif2 Dl Rad timer WCDMA chip count value  @param (Aif2Fl_AtWcdmaCount *) */
    case AIF2FL_QUERY_AT_DLRAD_WCDMA_VALUE:
        Aif2Fl_getAtDlRadtWcdmaValue (hAif2, (Aif2Fl_AtWcdmaCount *)response);
        break;

    /* Get Aif2 Rad timer time stamp clock count value  @param (uint32_t *) */
    case AIF2FL_QUERY_AT_RAD_TSTAMP_CLOCK_COUNT:
        *((uint32_t *)response) = Aif2Fl_getAtRadtTstampClkCount (hAif2);
        break;

    /* Get Aif2 GSM Tcount  value  @param (Aif2Fl_AtGsmTCount *) */
    case AIF2FL_QUERY_AT_GSM_TCOUNT_VALUE:
        Aif2Fl_getAtGsmTcount (hAif2, (Aif2Fl_AtGsmTCount *)response);
        break;

    /* Get Aif2 EE DB interrupt  status value  @param (Aif2Fl_EeDbInt *) use hAif2->ee_arg to select function  */
    case AIF2FL_QUERY_EE_DB_INT_STATUS:
        Aif2Fl_getEeDbIntStatus (hAif2, (Aif2Fl_EeDbInt *)response);
        break;

    /* Get Aif2 EE AD interrupt  status value  @param (Aif2Fl_EeAdInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_AD_INT_STATUS:
        Aif2Fl_getEeAdIntStatus (hAif2, (Aif2Fl_EeAdInt *)response);
        break;

    /* Get Aif2 EE CD(PKTDMA) interrupt  status value  @param (Aif2Fl_EeCdInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_CD_INT_STATUS:
        Aif2Fl_getEeCdIntStatus (hAif2, (Aif2Fl_EeCdInt *)response);
        break;

    /* Get Aif2 EE SD interrupt  status value  @param (Aif2Fl_EeSdInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_SD_INT_STATUS:
        Aif2Fl_getEeSdIntStatus (hAif2, (Aif2Fl_EeSdInt *)response);
        break;

    /* Get Aif2 EE VC interrupt  status value  @param (Aif2Fl_EeVcInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_VC_INT_STATUS:
        Aif2Fl_getEeVcIntStatus (hAif2, (Aif2Fl_EeVcInt *)response);
        break;

    /* Get Aif2 EE AIF2 run status value  @param (Aif2Fl_EeAif2Run *)  */
    case AIF2FL_QUERY_EE_AIF2_RUN_STATUS:
        Aif2Fl_getEeAif2RunStatus (hAif2, (Aif2Fl_EeAif2Run*)response);
        break;

    /* Get Aif2 EE error or alarm origination status  @param (Aif2Fl_EeAif2Run *)  */
    case AIF2FL_QUERY_EE_AIF2_ORIGINATION:
        Aif2Fl_getEeAif2Origination (hAif2, (Aif2Fl_EeOrigin*)response);
        break;

    /* Get Aif2 EE Link A interrupt  status value  @param (Aif2Fl_EeLinkAInt *) use hAif2->ee_arg to select function and hAif2->arg_link to select link */
    case AIF2FL_QUERY_EE_LINKA_INT_STATUS:
        Aif2Fl_getEeLinkAIntStatus (hAif2, (Aif2Fl_EeLinkAInt *)response);
        break;		

    /* Get Aif2 EE Link B interrupt  status value  @param (Aif2Fl_EeLinkBInt *) use hAif2->ee_arg to select function and hAif2->arg_link to select link */
    case AIF2FL_QUERY_EE_LINKB_INT_STATUS:
        Aif2Fl_getEeLinkBIntStatus (hAif2, (Aif2Fl_EeLinkBInt *)response);
        break;	

    /* Get Aif2 EE AT interrupt  status value  @param (Aif2Fl_EeAtInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_AT_INT_STATUS:
        Aif2Fl_getEeAtIntStatus (hAif2, (Aif2Fl_EeAtInt *)response);
        break;

    /* Get Aif2 EE PD common interrupt  status value  @param (Aif2Fl_EePdInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_PD_INT_STATUS:
        Aif2Fl_getEePdIntStatus (hAif2, (Aif2Fl_EePdInt *)response);
        break;

    /* Get Aif2 EE PE common interrupt  status value  @param (Aif2Fl_EePeInt *) use hAif2->ee_arg to select function */
    case AIF2FL_QUERY_EE_PE_INT_STATUS:
        Aif2Fl_getEePeIntStatus (hAif2, (Aif2Fl_EePeInt *)response);
        break;

    default:
        status = AIF2FL_INVQUERY;
	}

return status;
}

