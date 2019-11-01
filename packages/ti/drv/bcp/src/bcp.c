/** 
 *   @file  bcp.c
 *
 *   @brief  
 *      This file contains the BCP driver's higher layer API implementations.
 *
 *      The BCP Higher Layer presents a useful set of APIs to send/receive
 *      data from BCP.
 * 
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
 *  ============================================================================
 *
*/
/* BCP types include */
#include <bcp_types.h>

/* BCP OSAL include */
#include <bcp_osal.h>

/* BCP include */
#include <ti/drv/bcp/bcp.h>

/* BCP private definitions include file */
#include <ti/drv/bcp/include/bcp_pvt.h>

/* Standard C definitions include */
#include <string.h>

/***************************************************************
 ******************** GLOBAL VARIABLES *************************
 **************************************************************/

/** Global variable to hold the BCP driver' state info */
extern Bcp_InstanceInfo         gBcp_instanceInfo [BCP_MAX_NUM_INSTANCES];

/** @addtogroup BCP_FUNCTION
 @{ */

/***************************************************************
 ******************** BCP Internal APIs ************************
 **************************************************************/

/**
* ============================================================================
*  @n@b Bcp_txQueueOpen
*
*  @b   brief
*  @n   If not already open, this API sets up the BCP Transmit queue corresponding 
*       to the queue number specified in 'bcpTxQNum' (valid values 0-8) and the
*       BCP instance specified by 'pBcpInstInfo' handle. If open already, the 
*       queue configuration is skipped.
*
*       As part of the queue configuration this API opens the CPPI queue, and the
*       corresponding Tx, Rx channels.
*
*  @param[in]
*       pBcpInstInfo        BCP instance information handle.
*
*  @param[in]
*       bcpTxQNum           BCP Transmit queue number for which the setup needs 
*                           to be performed. Valid values are between 0-8.
*
*  @return     Bcp_TxQInfo*
*  @li         NULL         -   Tx queue setup failed as per input provided.
*  @li         Valid Handle -   Success. Valid Tx queue handle returned.
*
*  @pre
*  @n   @a Bcp_open () API must have been called to initialize the driver's
*       global state before calling this API.
*
*  @post
*  @n   BCP Queue 'bcpTxQNum' opened and the corresponding CPPI channels are 
*       opened too if called for the first time. Reference counter for the 
*       queue is incremented to keep track of all the application threads 
*       using this queue.
* ============================================================================
*/
static Bcp_TxQInfo* Bcp_txQueueOpen
(
    Bcp_InstanceInfo*           pBcpInstInfo,
    Bcp_QueueId                 bcpTxQNum
)
{
    uint8_t                     isAllocated;        
    Cppi_TxChInitCfg            txChCfg;
    Qmss_QueueHnd               hQmssTxQ = (Qmss_QueueHnd)NULL;
    Cppi_ChHnd                  hCppiTxChan = NULL;  
    void*                      	hBcpTxQueueInfo;
    uint8_t                     destSel [8], prioVal [8];

    /* If this is the first time we are initializing this the BCP transmit queue,
     * setup the configuration, otherwise just increment the reference count
     * and return the queue handle.
     */
    if (!pBcpInstInfo->Bcp_txQInfo[bcpTxQNum].refCnt)
    {
        /* Setup and open the Tx channel corresponding to this queue.
         * In BCP, Tx queues are mapped one to one with Tx channels.
         *
         * Get the priority for the channel (Tx QFIFO) from the BCP MMRs 
         * configured by the application.
         */
        Bcp_getTxQfifoReadDestSelReg (&pBcpInstInfo->bcpLldObj, destSel, prioVal);

        txChCfg.channelNum      =   bcpTxQNum;                      
        txChCfg.txEnable        =   Cppi_ChState_CHANNEL_DISABLE;   
        txChCfg.filterEPIB      =   0;
        txChCfg.filterPS        =   0;
        txChCfg.aifMonoMode     =   0;
        txChCfg.priority        =   prioVal [bcpTxQNum];  
        if ((hCppiTxChan = Cppi_txChannelOpen (pBcpInstInfo->hCppi, &txChCfg, &isAllocated)) == NULL)
        {
            Bcp_osalLog ("Error opening Tx channel corresponding to BCP queue : %d\n", bcpTxQNum);
            goto return_error;
        }

        /* Open the BCP Tx queue */
        if ((hQmssTxQ = Qmss_queueOpen ((Qmss_QueueType) QMSS_PARAM_NOT_SPECIFIED, pBcpInstInfo->baseTxQueueNum + bcpTxQNum, &isAllocated)) < 0)
        {
            Bcp_osalLog ("Error opening BCP Tx Queue Number %d \n", bcpTxQNum);
            goto return_error;
        }

        /* Finally, enable the Tx channel so that we can start sending data to BCP */
        Cppi_channelEnable (hCppiTxChan);

        /* Save the channel setup info */
        pBcpInstInfo->Bcp_txQInfo[bcpTxQNum].qNum           =   bcpTxQNum;
        pBcpInstInfo->Bcp_txQInfo[bcpTxQNum].hCppiTxChan    =   hCppiTxChan;
        pBcpInstInfo->Bcp_txQInfo[bcpTxQNum].hQmssTxQ       =   hQmssTxQ;
    }

    /* Increment the reference count on this queue */
    pBcpInstInfo->Bcp_txQInfo[bcpTxQNum].refCnt ++;                    
    
    hBcpTxQueueInfo = &pBcpInstInfo->Bcp_txQInfo[bcpTxQNum];

    /* Return success. Return the handle to the Tx queue */
    return hBcpTxQueueInfo;

return_error:
    if (hCppiTxChan)
    {
        Cppi_channelClose (hCppiTxChan);
    }
    if (hQmssTxQ)
    {
        Qmss_queueClose (hQmssTxQ);       
    }

    /* Return NULL handle to indicate that the setup failed */
    return NULL;
}

/**
* ============================================================================
*  @n@b Bcp_txQueueClose
*
*  @b   brief
*  @n   This API decrements the reference count on the transmit queue handle
*       provided and when the reference count reaches zero, i.e., when no
*       application thread is no longer using this transmit queue handle it
*       frees up the BCP transmit queue handle and all associated configuration.
*       If an application wishes to use the transmit queue, it would have to do
*       so by opening the BCP transmit queue with appropriate configuration
*       using the API @a Bcp_txQueueOpen ().
*
*  @param[in]
*       pBcpTxQInfo         Handle to the transmit queue obtained using
*                           @a  Bcp_txQueueOpen () API.
*
*  @return     Bcp_RetVal
*  @li         BCP_RETVAL_EBAD_HANDLE  -   Invalid transmit queue handle provided.
*  @li         BCP_RETVAL_SUCCESS      -   Succesfully closed the queue.
*
*  @pre
*  @n   @a Bcp_txQueueOpen () must have been called succesfully to setup the 
*       BCP transmit queue before calling this API.
*
*  @post
*  @n   Reference counter on the queue handle decremented. If reference counter
*       reaches zero, queue configuration cleaned up and all associated CPPI
*       handles released.
* ============================================================================
*/
static Bcp_RetVal Bcp_txQueueClose 
(
    Bcp_TxQInfo*               pBcpTxQInfo
)
{
    /* Validate input */
    if (!pBcpTxQInfo)
    {
        Bcp_osalLog ("Invalid Transmit queue handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;	
    }

    if (!pBcpTxQInfo->refCnt)
    {
        return BCP_RETVAL_SUCCESS;	
    }

    /* We can close the queue and clean up all its associated CPPI
     * handles only if there is no other application/driver using
     * this queue other than us, i.e. refCnt is 1.
     */
    if (pBcpTxQInfo->refCnt == 1)
    {
        /* Disable the Tx channel */            
        Cppi_channelDisable (pBcpTxQInfo->hCppiTxChan);

        /* Close the CPPI/QMSS channels and queues associated with 
         * the transmit queue specified.
         */
        Cppi_channelClose (pBcpTxQInfo->hCppiTxChan);
        Qmss_queueClose (pBcpTxQInfo->hQmssTxQ);
    
        /* Update our database of tx queue state info */
        pBcpTxQInfo->qNum           =   0;
        pBcpTxQInfo->hCppiTxChan    =   NULL;
        pBcpTxQInfo->hQmssTxQ       =   (Qmss_QueueHnd) NULL;
    }

    /* Decrement the reference count to reflect the successful
     * close.
     */
    pBcpTxQInfo->refCnt --;

    /* Return Success */
    return BCP_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Bcp_getNextAvailRxObjId
 *
 *  @b  brief
 *  @n  Given a BCP peripheral instance number, this API searches through
 *      the instance's global Rx object info database to find the next 
 *      available empty slot to store a new Rx object's info. Each slot in this 
 *      global Rx object info database stores Rx object information that needs
 *      to be globally accessible, i.e., accessible from all cores of the 
 *      device.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]    
        instNum             BCP instance number from which an empty Rx object
                            slot must be found.
 *
 *  @return     int32_t
 *  @li         <0  -   Error finding an empty slot in the
 *                      Rx object global database.
 *                                  
 *  @li         >=0 -   Valid Rx object slot id from the database returned 
 *                      to store the new Rx object info.
 *
 *  @pre
 *  @n  @a Bcp_open () must have been called to initialize the driver before 
 *      calling this API. Also a valid instance number must be specified to 
 *      this API in 'instNum' parameter.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        ...

        // Get the next available Rx object slot from the database.
        if (Bcp_getNextAvailRxObjId (CSL_BCP) < 0)
        {
            // Number of Rx objects created exceeds maximum allowed.
        }        
    @endcode
 * ============================================================================
 */
static int32_t Bcp_getNextAvailRxObjId 
(
    uint8_t                   instNum
)
{
    int32_t                   i;

    for (i = 0; i < BCP_MAX_NUM_RXOBJECTS; i++)
    {
        if (!gBcp_instanceInfo[instNum].Bcp_rxObjGlobalInfo[i].bIsValid)
        {
            /* Found an empty Rx object slot. Return the slot number */                
            return i;                
        }
        /* Slot at "i" already taken by a Rx object. Continue searching */                
    }

    /* If we are here, indicates that we are done searching
     * through the entire Rx object global database and found
     * no slot empty.
     *
     * Return error. Maximum number of Rx objects reached.
     */
    return BCP_RETVAL_EFAILURE;
}


/***************************************************************
 ******************** BCP Exported APIs ************************
 **************************************************************/

/**
 * ============================================================================
 *  @n@b Bcp_getVersionID
 *
 *  @b  brief
 *  @n  This API returns the BCP driver's version Id.
 *      
 *  @return     
 *  @n  uint32_t    -   Returns the BCP driver version Id
 *
 *  @pre
 *  @n  None. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
uint32_t Bcp_getVersionID 
( 
    void
)
{
    return BCP_VERSION_ID;
}

const char  Bcp_versionStr[] = BCP_VERSION_STR ":" __DATE__  ":" __TIME__;

/**
 * ============================================================================
 *  @n@b Bcp_getVersionStr
 *
 *  @b  brief
 *  @n  This API returns the BCP driver's version information in string format.
 *      
 *  @return     
 *  @n  const char*     -   Returns the BCP driver build and version info in 
 *                          string format.
 *
 *  @pre
 *  @n  None. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
const char* Bcp_getVersionStr 
(
    void
)
{
    return (const char *) (Bcp_versionStr);
}

/**
 * ============================================================================
 *  @n@b Bcp_init
 *
 *  @b  brief
 *  @n  This API initializes the BCP peripheral with the configuration provided,
 *      and opens the BCP CDMA for use.
 *
 *      BCP driver can be initialized in 2 modes - "local" and "remote".
 *      In "remote" mode, applications can only send/receive data to/from BCP, 
 *      but cannot configure its registers or its CDMA. "Local" mode however
 *      gives access to entire BCP register and CDMA configuration space in 
 *      addition to the ability to send/receive packets from it. Application must
 *      initialize BCP in appropriate mode depending on whether BCP peripheral 
 *      is present on the device where driver is being executed or not.
 *
 *      This API MUST be called ONLY once in the system and at system startup
 *      for any given BCP peripheral instance before using any of the BCP 
 *      driver APIs.
 *
 *  @param[in]    
 *      instNum             BCP peripheral instance number.
 *
 *  @param[in]    
 *      mode                Indicates if BCP driver needs to execute in "local"
 *                          or "remote" modes. 
 *
 *  @param[in]    
 *      pBcpInitCfg         BCP initialization configuration.
 *
 *  @return     Bcp_RetVal
 *  @li         BCP_RETVAL_EINVALID_PARAMS -   Invalid instance number provided.
 *  @li         BCP_RETVAL_EBAD_HANDLE     -   Invalid init configuration
 *                                             handle provided.
 *  @li         BCP_RETVAL_EFAILURE        -   Error initializing driver for the
 *                                             BCP instance requested.
 *  @li         BCP_RETVAL_SUCCESS         -   BCP instance initialized succesfully.                                              
 *
 *  @pre
 *  @n  This API MUST be called only once during system startup. 
 *
 *  @post
 *  @n  BCP driver initialized for the instance specified.
 * ============================================================================
 */
Bcp_RetVal Bcp_init 
(
    uint8_t                     instNum, 
    Bcp_DrvMode                 mode,
    Bcp_InitCfg*                pBcpInitCfg
)
{
    Cppi_Handle                 hCppi;
    Cppi_CpDmaInitCfg           cpdmaCfg;
	
    /* Validate user input */
    if (instNum >= BCP_MAX_NUM_INSTANCES)
    {
        Bcp_osalLog ("Invalid instance number! Valid range 0 - %d \n", (BCP_MAX_NUM_INSTANCES - 1));
        return BCP_RETVAL_EINVALID_PARAMS;
    }

    if (!pBcpInitCfg)
    {
        Bcp_osalLog ("Invalid BCP Init configuration handle passed \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }

    if (!pBcpInitCfg->BcpTunnel_txOpen || !pBcpInitCfg->BcpTunnel_txClose ||
        !pBcpInitCfg->BcpTunnel_rxOpen || !pBcpInitCfg->BcpTunnel_rxClose ||
        !pBcpInitCfg->BcpTunnel_send || !pBcpInitCfg->BcpTunnel_recv ||
        !pBcpInitCfg->BcpTunnel_freeRecvBuffer)
    {
        Bcp_osalLog ("Invalid BCP transport layer function handlers passed \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();

    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));    

    /* Reset the driver's global info for this instance */            
    memset (&gBcp_instanceInfo [instNum], 0, sizeof (Bcp_InstanceInfo));

    /* Setup BCP peripheral/CDMA only if it is local to the device */
    if (mode == Bcp_DrvMode_LOCAL)
    {
        /* Set up the BCP CDMA configuration */
        memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
        cpdmaCfg.dmaNum     =   pBcpInitCfg->cpdmaNum;

        /* Initialize BCP CDMA */
        if ((hCppi = Cppi_open (&cpdmaCfg)) == NULL)
        {
            Bcp_osalLog ("Error opening BCP CDMA %d\n", cpdmaCfg.dmaNum);

            /* Exit Critical Section */
            Bcp_osalMultiCoreCsExit ();
            return BCP_RETVAL_EFAILURE;
        }

        /* Disable BCP CDMA loopback */
        if (Cppi_setCpdmaLoopback (hCppi, 0) != CPPI_SOK)
        {
            Bcp_osalLog ("Error disabling loopback for BCP CDMA %d\n", cpdmaCfg.dmaNum);

            /* Exit Critical Section */
            Bcp_osalMultiCoreCsExit ();
            return BCP_RETVAL_EFAILURE;
        }

        /* Open the BCP LLD for this instance. 
         *
         * Required to setup/read the BCP MMR for this instance.
         */
        if (Bcp_lldOpen (instNum, pBcpInitCfg->cfgRegsBaseAddress, &gBcp_instanceInfo[instNum].bcpLldObj) != 0)
        {
            /* BCP LLD Instance open failed */            
            Bcp_osalLog ("BCP LLD Instance %d open failed \n", instNum);

            /* Exit Critical Section */
            Bcp_osalMultiCoreCsExit ();
            return BCP_RETVAL_EFAILURE;
        }

        /* Save the CPPi driver handle and the Start Tx queue number for this instance. */
        gBcp_instanceInfo[instNum].hCppi                    =   hCppi;
        gBcp_instanceInfo[instNum].baseTxQueueNum           =   pBcpInitCfg->baseTxQueueNum;
    }

    /* Save the transport layer callbacks and other info */
    gBcp_instanceInfo[instNum].instNum                      =   instNum;
    gBcp_instanceInfo[instNum].mode                         =   mode;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_txOpen         =   pBcpInitCfg->BcpTunnel_txOpen;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_txClose        =   pBcpInitCfg->BcpTunnel_txClose;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxOpen         =   pBcpInitCfg->BcpTunnel_rxOpen;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxClose        =   pBcpInitCfg->BcpTunnel_rxClose;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_send           =   pBcpInitCfg->BcpTunnel_send;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_recv           =   pBcpInitCfg->BcpTunnel_recv;
    gBcp_instanceInfo[instNum].pFxnBcpTunnel_freeRecvBuffer =   pBcpInitCfg->BcpTunnel_freeRecvBuffer;

    /* Increment the reference count to indicate that this instance
     * has been initialized and is ready for use by an application.
     */
    gBcp_instanceInfo[instNum].refCnt ++;

    /* Writeback the shared Instance Info object */
    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* Exit Critical Section */
    Bcp_osalMultiCoreCsExit ();

    /* Successfully done with initialization. Return. */
    return BCP_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Bcp_deInit
 *
 *  @b  brief
 *  @n  This API closes the BCP peripheral instance opened earlier using 
 *      @a Bcp_init () API. When all the applications using this BCP
 *      instance have released it (reference count on the instance reaches zero),
 *      this API closes the BCP CDMA, closes the BCP LLD and finally initiates 
 *      a software reset on the BCP to reset all its registers, and 
 *      it's state machine. On success, it restores the BCP peripheral state 
 *      to 'un-initialized' state
 *
 *      If an application or driver would like to use any of the driver APIs again,
 *      it can do so only after calling @a Bcp_init() again.
 *
 *  @param[in]
 *       instNum             BCP peripheral instance number.
 *
 *  @return     Bcp_RetVal
 *  @li         BCP_RETVAL_SUCCESS      -   Successfully closed the BCP CDMA.
 *
 *  @pre
 *  @n  @a Bcp_init () must be called before calling this API. 
 *
 *  @post
 *  @n  Reference count decremented on the BCP instance in the driver. When
 *      reference count reaches zero, CDMA is closed, all BCP Tx queues closed 
 *      and deallocated. BCP software reset performed.
 * ============================================================================
 */
Bcp_RetVal Bcp_deInit 
(
    uint8_t                     instNum
)
{
    /* BCP setup cannot be undone unless initialized */
    if (!Bcp_isInitialized (instNum))
    {
        Bcp_osalLog ("BCP instance %d not open !! \n", instNum);
        return BCP_RETVAL_SUCCESS;
    }

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();

    /* Ensure all applications using this BCP instance are done
     * using before resetting the instance.
     */
    if (gBcp_instanceInfo[instNum].refCnt == 1)
    {
        /* Reset BCP peripheral only if deinit being done from local device */            
        if (gBcp_instanceInfo[instNum].mode == Bcp_DrvMode_LOCAL)
        {
            /* Close the CDMA */
            Cppi_close (gBcp_instanceInfo[instNum].hCppi);

            /* Issue a software reset to BCP to reset all its register values 
             * and state machines to its initial state.
             */
            Bcp_doSoftwareReset (&gBcp_instanceInfo[instNum].bcpLldObj);

            Bcp_lldClose (&gBcp_instanceInfo[instNum].bcpLldObj);

            /* BCP deinit successful. Update the status variables. */
            gBcp_instanceInfo[instNum].hCppi                    =   NULL;
            gBcp_instanceInfo[instNum].baseTxQueueNum           =   0;
        }
   
        gBcp_instanceInfo[instNum].instNum                      =   0;
        gBcp_instanceInfo[instNum].mode                         =   Bcp_DrvMode_LOCAL;
        gBcp_instanceInfo[instNum].refCnt                       =   0;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_txOpen         =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_txClose        =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxOpen         =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxClose        =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_send           =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_recv           =   NULL;
        gBcp_instanceInfo[instNum].pFxnBcpTunnel_freeRecvBuffer =   NULL;
    }

    /* De-init complete. Update the physical memory. */
    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* Exit Critical Section */
    Bcp_osalMultiCoreCsExit ();
    
    /* Return success. */
    return BCP_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Bcp_isInitialized
 *
 *  @b  brief
 *  @n  Given an BCP peripheral instance number, this API returns 1 to
 *      indicate that it is initialized and has been successfully setup, 0 otherwise.
 *
 *  @param[in]
 *      instNum             BCP peripheral instance number to check the status 
 *                          on.
 *
 *  @return     uint8_t
 *  @li         1       -   Driver initialized for the instance number passed.
 *  @li         0       -   Instance number invalid/not initialized.
 *
 *  @pre
 *  @n  None.
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
uint8_t Bcp_isInitialized 
(
    uint8_t                     instNum
)
{
    uint8_t                     retVal;

    /* Critical section start */
    Bcp_osalMultiCoreCsEnter ();

    /* Invalidate cache before reading shared Instance Info to
     * ensure we are in sync.
     */
    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* Validate input */
    if (instNum >= BCP_MAX_NUM_INSTANCES || !gBcp_instanceInfo[instNum].refCnt)
    {
        Bcp_osalLog ("Invalid instance number passed \n");
    
        /* critical section end */
        Bcp_osalMultiCoreCsExit ();
        return 0;
    }

    /* check if the BCP instance is setup and valid */
    if (gBcp_instanceInfo[instNum].refCnt)
    {
        /* BCP instance open */            
        retVal = 1;
    }
    else
    {
        /* BCP instance not open */            
        retVal = 0;
    }    

    /* critical section end */
    Bcp_osalMultiCoreCsExit ();

    return retVal;
}

/**
 * ============================================================================
 *  @n@b Bcp_open
 *
 *  @b  brief
 *  @n  This API MUST be called by all BCP driver users at least once to obtain
 *      an BCP driver handle.
 *
 *  @param[in]    
 *      instNum             BCP Instance number that the application would like
 *                          to use.
 *
 *  @param[in]    
 *      pBcpDrvCfg          Handle to BCP driver configuration structure.
 *
 *  @param[out]    
 *      pRetVal             Pointer to hold the return value returned by this API. 
 *                          Following are the possible return values for this API:
 *                            
 *  @li         BCP_RETVAL_EINVALID_PARAMS  -   Invalid instance number/driver 
 *                                              configuration provided.
 *  @li         BCP_RETVAL_EBAD_HANDLE      -   Invalid driver configuration
 *                                              handle provided.
 *  @li         BCP_RETVAL_ENO_MEM          -   Error allocating memory for driver's 
 *                                              internal book-keeping data.
 *  @li         BCP_RETVAL_SUCCESS          -   Successfully opened the driver instance.
 *
 *  @return     Bcp_DrvHandle
 *  @li         NULL                        -   Error. Appropriate error code is returned
 *                                              in the output parameter 'pRetVal'.
 *  @li         >0                          -   Success. Valid BCP driver handle is returned.
 *
 *  @pre
 *  @n  None
 *
 *  @post
 *  @n  None 
 * ============================================================================
 */
Bcp_DrvHandle Bcp_open 
(
    uint8_t                     instNum,
    Bcp_DrvCfg*                 pBcpDrvCfg,
    Bcp_RetVal*                 pRetVal
)
{
    Bcp_DrvInfo*                pBcpDrvInfo;

    /* Validate input */
    if (!pRetVal)
    {
        Bcp_osalLog ("Invalid Retrun value handle passed \n");
        return NULL;
    }

    /* Validate the instance number passed. Instance number passed is invalid if:
     *
     *  (1) Instance number exceeds the acceptable range.
     *  (2) Instance number is valid, but the driver's state info indicates that
     *      the instance has not been setup using Bcp_init () API.
     */
    if (!Bcp_isInitialized (instNum))
    {
        Bcp_osalLog ("Invalid instance number or instance not initialized yet. \n");
        *pRetVal    =   BCP_RETVAL_EINVALID_PARAMS;
        return NULL;
    }
	    
    if (!pBcpDrvCfg)
    {
        Bcp_osalLog ("Invalid BCP driver configuration handle passed \n");
        *pRetVal    =   BCP_RETVAL_EBAD_HANDLE;
        return NULL;
    }

    /* Allocate space to hold BCP descriptor configuration for this user */
    if ((pBcpDrvInfo = (Bcp_DrvInfo *) Bcp_osalMalloc (sizeof (Bcp_DrvInfo), FALSE)) == NULL)
    {
        Bcp_osalLog ("Error allocating memory for BCP driver info \n");
        *pRetVal    =   BCP_RETVAL_ENO_MEM;
        return NULL;
    }
    memset ((void *)pBcpDrvInfo, 0, sizeof (Bcp_DrvInfo));

    /* Save driver settings */
    pBcpDrvInfo->instNum                        =   instNum;   
    pBcpDrvInfo->mode                           =   gBcp_instanceInfo[instNum].mode;   
    pBcpDrvInfo->drvCfg                         =   *pBcpDrvCfg;
    pBcpDrvInfo->pFxnBcpTunnel_txOpen           =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_txOpen;
    pBcpDrvInfo->pFxnBcpTunnel_txClose          =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_txClose;
    pBcpDrvInfo->pFxnBcpTunnel_rxOpen           =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxOpen;
    pBcpDrvInfo->pFxnBcpTunnel_rxClose          =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_rxClose;
    pBcpDrvInfo->pFxnBcpTunnel_send             =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_send;
    pBcpDrvInfo->pFxnBcpTunnel_recv             =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_recv;
    pBcpDrvInfo->pFxnBcpTunnel_freeRecvBuffer   =   gBcp_instanceInfo[instNum].pFxnBcpTunnel_freeRecvBuffer;

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();

    /* Increment reference count to keep the instance info alive */
    gBcp_instanceInfo[instNum].refCnt ++;

    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* Exit Critical Section */
    Bcp_osalMultiCoreCsExit ();

    /* Return success */
    *pRetVal    =   BCP_RETVAL_SUCCESS;

    /* Return handle to the BCP instance info */ 
    return ((Bcp_DrvHandle) pBcpDrvInfo);
}

/**
 * ============================================================================
 *  @n@b Bcp_close
 *
 *  @b  brief
 *  @n  This API closes the BCP driver instance opened earlier using a call
 *      to @a Bcp_open () API. 
 *
 *  @param[in]
 *      hBcp                BCP driver handle obtained using @a Bcp_open () 
 *                          API.
 *
 *  @return     Bcp_RetVal
 *  @li         BCP_RETVAL_EBAD_HANDLE  -   Invalid input handle passed. Error.
 *  @li         BCP_RETVAL_SUCCESS      -   Successfully closed the BCP.
 *
 *  @pre
 *  @n  @a Bcp_open () must have been called to open the driver instance prior to
 *      calling this API. Also, all open Tx, Rx objects opened by an application
 *      must be closed by calling @a Bcp_txClose () and Bcp_rxClose () APIs 
 *      before calling this API to ensure all associated memory is freed cleanly.
 *
 *  @post
 *  @n  Reference count decremented on the BCP instance in the driver. 
 * ============================================================================
 */
Bcp_RetVal Bcp_close 
(
    Bcp_DrvHandle           hBcp
)
{
    Bcp_DrvInfo*            pBcpDrvInfo;

    /* Validate input */
    if (!hBcp)
    {
        Bcp_osalLog ("Invalid BCP driver handle passed! \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }

    pBcpDrvInfo   =   (Bcp_DrvInfo *) hBcp;

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();

    /* Invalidate the BCP instance in cache */
    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[pBcpDrvInfo->instNum], sizeof (Bcp_InstanceInfo));
    gBcp_instanceInfo[pBcpDrvInfo->instNum].refCnt --;            
    Bcp_osalEndMemAccess (&gBcp_instanceInfo[pBcpDrvInfo->instNum], sizeof (Bcp_InstanceInfo));

    /* Exit Critical Section */
    Bcp_osalMultiCoreCsExit ();
    
    /* Free the BCP state info handle */
    Bcp_osalFree ((void*) pBcpDrvInfo, sizeof (Bcp_DrvInfo), FALSE);

    /* Return success. */
    return BCP_RETVAL_SUCCESS;
}


/**
 * ============================================================================
 *  @n@b Bcp_getLLDHandle
 *
 *  @b  brief
 *  @n  This API returns the BCP LLD object handle corresponding to the BCP
 *      instance number provided. 
 *
 *  @param[in]
 *      instNum             BCP instance number.
 *
 *  @return     Bcp_LldObj*
 *  @li         NULL        -   Invalid instance number passed/Corresponding BCP 
 *                              instance not initialized yet. Error.
 *  @li         >0          -   BCP LLD object handle returned.
 *
 *  @pre
 *  @n  @a Bcp_init () must have been called to open the driver instance prior to
 *      calling this API. 
 *
 *  @post
 *  @n  None
 * ============================================================================
 */
Bcp_LldObj* Bcp_getLLDHandle 
(
    uint8_t                 instNum
)
{
    Bcp_LldObj*             pBcpLldObj = NULL;

    /* Validate input */
    if (!Bcp_isInitialized (instNum))
    {
        Bcp_osalLog ("Invalid BCP instance number passed \n");
        return NULL;
    }

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();

    /* Invalidate the BCP instance in cache */
    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    if (gBcp_instanceInfo[instNum].mode == Bcp_DrvMode_LOCAL)
        pBcpLldObj =   &gBcp_instanceInfo[instNum].bcpLldObj;            
    else
        pBcpLldObj =   NULL;

    /* Exit Critical Section */
    Bcp_osalMultiCoreCsExit ();
    
    /* Return LLD handle. */
    return pBcpLldObj;
}

/**
* ============================================================================
*  @n@b Bcp_txOpen
*
*  @b   brief
*  @n   This API sets up a transmit object for the calling application. A 
*       transmit object must be setup by any application that wishes to
*       send data for processing to BCP using the driver. 
*
*       On success, this API returns a valid, non-zero Tx handle and NULL 
*       otherwise to indicate an error.
*
*       On BCP local device, a valid Tx endpoint configuration must be passed
*       in 'pTxEndpointCfg' if the BCP Tx queue being opened needs to be
*       made accessible from remote device too. If no remote access is required
*       for this BCP Tx queue, then 'pTxEndpointCfg' parameter can be set
*       to NULL.
*
*       On BCP remote device, the BCP Tx queue configuration 'pBcpTxCfg'
*       must be set to NULL since remote device has no direct access to 
*       BCP CDMA. Configuration required to open a BCP Tx remote endpoint
*       must be passed in 'pTxEndpointCfg' and the driver in turn passes
*       this as-is to the registered Tx endpoint open API.
*
*  @param[in]
*       hBcp                BCP driver handle obtained using @a Bcp_open () 
*                           API.
*
*  @param[in]  
*       pBcpTxCfg           Input structure that holds configuration for the
*                           Tx object. MUST be set to NULL on remote device
*                           and MUST contain valid configuration on local
*                           device.
*
*  @param[in]  
*       pTxEndpointCfg      Configuration for BCP tunnel Tx endpoint object.
*                           MUST be valid for remote device. Can be NULL
*                           or a valid handle on local device based on whether
*                           remote access needs to be enabled for this Tx queue
*                           or not.
*
*  @return      Bcp_TxHandle
*  @li          NULL                -   Error. Invalid configuration passed. 
*  @li          Valid Tx handle     -   Success. 
*
*  @pre
*  @n   Valid BCP driver handle must be obtained by calling @a Bcp_open () API
*       by the application before calling this API. Valid Tx configuration must be 
*       specified in 'pBcpTxCfg' structure as input to this API on local device.
*
*  @post
*  @n   BCP Tx queue open and setup if this API being called for first time for
*       a given Tx queue number. Tx object is ready for use to send data to BCP.
*       BCP tunnel Tx endpoint is setup too using the callback registered at
*       BCP init time.
* ============================================================================
*/
Bcp_TxHandle Bcp_txOpen
(
    Bcp_DrvHandle               hBcp,
    Bcp_TxCfg*                  pBcpTxCfg,
    void*                       pTxEndpointCfg
)
{
    Bcp_DrvInfo*                pBcpDrvInfo;
    Bcp_TxQInfo*                pBcpTxQInfo = NULL;
    Bcp_TxInfo*                 pTxObjInfo;
    void*                       hTxEndpointInfo = NULL;
    uint8_t                     instNum;

    /* Validate application's BCP driver handle. */
    if (!hBcp)
    {
        Bcp_osalLog ("Invalid BCP driver handle passed! \n");
        return NULL;
    }

    pBcpDrvInfo     =   (Bcp_DrvInfo *) hBcp;
    instNum         =   pBcpDrvInfo->instNum;
    
    /* Validate input */
    if ((pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL && !pBcpTxCfg) ||
        (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE && !pTxEndpointCfg))
    {
        Bcp_osalLog ("Invalid inputs passed \n");            
        return NULL;
    }

    /* Enter Critical Section */
    Bcp_osalMultiCoreCsEnter ();            

    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        /* Open the BCP Tx queue if it is local to the device */
        if ((pBcpTxQInfo = Bcp_txQueueOpen (&gBcp_instanceInfo[instNum], pBcpTxCfg->txQNum)) == NULL)
        {
            Bcp_osalLog ("BCP Tx Queue Open failed \n");
            Bcp_osalMultiCoreCsExit ();
            return NULL;
        }

        /* Create a BCP tunnel Tx local endpoint for this queue if requested. */
        if (pTxEndpointCfg)
        {
            if ((hTxEndpointInfo = pBcpDrvInfo->pFxnBcpTunnel_txOpen (pTxEndpointCfg)) == NULL)
            {
                Bcp_osalLog ("Error opening BCP tunnel's local Tx endpoint \n");

                Bcp_txQueueClose (pBcpTxQInfo);
                Bcp_osalMultiCoreCsExit ();
                return NULL;
            }
        }
    }
    else
    {
        /* Create a BCP tunnel Tx remote endpoint. */
        if ((hTxEndpointInfo = pBcpDrvInfo->pFxnBcpTunnel_txOpen (pTxEndpointCfg)) == NULL)
        {
            Bcp_osalLog ("Error opening BCP tunnel's Tx remote endpoint \n");

            Bcp_osalMultiCoreCsExit ();
            return NULL;
        }
    }

    /* Allocate memory for holding the Tx object info */
    if ((pTxObjInfo = (Bcp_TxInfo *) Bcp_osalMalloc (sizeof (Bcp_TxInfo), FALSE)) == NULL)
    {
        Bcp_osalLog ("Error allocating memory for Tx object \n");

        if (hTxEndpointInfo)
            pBcpDrvInfo->pFxnBcpTunnel_txClose (hTxEndpointInfo);

        if (pBcpTxQInfo)
            Bcp_txQueueClose (pBcpTxQInfo);

        Bcp_osalMultiCoreCsExit ();
        Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));
        return NULL;
    }

    /* Initialize the Tx handle */
    memset ((void*) pTxObjInfo, 0, sizeof (Bcp_TxInfo));

    /* Save the Tx settings and state information */
    pTxObjInfo->pTxQInfo        =   pBcpTxQInfo;
    pTxObjInfo->hBcp            =   hBcp;
    pTxObjInfo->hTxEndpointInfo =   hTxEndpointInfo;

    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    Bcp_osalMultiCoreCsExit ();

    /* Return success. Return Tx object handle. */
    return ((Bcp_TxHandle) pTxObjInfo);
}

/**
* ============================================================================
*  @n@b Bcp_txClose
*
*  @b   brief
*  @n   This API closes the Tx object and frees memory allocated for it.
*
*       On local device, it also decrements the reference count on the Tx 
*       queue this object uses. When the reference count on the queue reaches 
*       zero, the Tx queue is closed. Additionally, if any tunnel Tx endpoint 
*       was created, its corresponding close function is closed to clean it
*       up.
*
*       On remote device, the Tx remote endpoint is closed by calling the
*       corresponding transport layer function.
*
*  @param[in]
*       hBcpTxInfo         BCP Tx handle obtained using @a Bcp_txOpen() API.
*
*  @return     Bcp_RetVal
*  @li         BCP_RETVAL_EBAD_HANDLE  -   Error. Invalid input passed.
*  @li         BCP_RETVAL_SUCCESS      -   Success. BCP Tx object closed.
*
*  @pre
*  @n   A valid Tx handle also must be obtained using @a Bcp_txOpen () 
*       before calling this API.
*
*  @post
*  @n   BCP Tx object successfully closed. The Tx queue the object uses is closed 
*       if reference count on it reaches zero.
* ============================================================================
*/
Bcp_RetVal Bcp_txClose 
(
    Bcp_TxHandle                hBcpTxInfo
)
{
    Bcp_TxInfo*                 pTxObjInfo;
    Bcp_DrvInfo*                pBcpDrvInfo;
    uint8_t                     instNum;

    /* Validate input handles */
    if (!hBcpTxInfo)
    {
        Bcp_osalLog ("Invalid Transmit Object Handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;	
    }

    pTxObjInfo      =   (Bcp_TxInfo *) hBcpTxInfo;
    pBcpDrvInfo     =   (Bcp_DrvInfo *) pTxObjInfo->hBcp;
    instNum         =   pBcpDrvInfo->instNum;

    /* Grab the driver's lock to access shared info */
    Bcp_osalMultiCoreCsEnter ();

    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

        /* Close the Tx queue used by the Tx object */
        Bcp_txQueueClose (pTxObjInfo->pTxQInfo);

        Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));
    }

    if (pTxObjInfo->hTxEndpointInfo)
        pBcpDrvInfo->pFxnBcpTunnel_txClose (pTxObjInfo->hTxEndpointInfo);

    /* Release the driver's lock. */
    Bcp_osalMultiCoreCsExit ();       

    /* Done cleaning up. Free the Tx object handle. */
    Bcp_osalFree (pTxObjInfo, sizeof (Bcp_TxInfo), FALSE);

    /* Return Success */
    return BCP_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Bcp_rxOpen
*
*  @b   brief
*  @n   This API sets up a Rx object for the calling application. A Rx object
*       is required by driver users to receive output from the BCP.
*
*       On Local device, a valid Rx object configuration MUST be provided in 
*       the input parameter 'pBcpRxCfg'. This API creates a BCP flow using the 
*       configuration provided. It also sets up the accumulator interrupts if 
*       interrupts are requested for this object. If any configuration needs
*       to be done to redirect output from BCP to a remote device, then a
*       valid configuration can be passed in 'pRxEndpointCfg' parameter and
*       the driver will invoke the BCP transport layer's Rx endpoint open
*       function. This parameter can be set to NULL if BCP output is to
*       be processed on the same device or if no additional transport layer
*       configuration is required.
*
*       On Remote device, a valid Rx endpoint configuration must be passed in
*       the 'pRxEndpointCfg' parameter. This API in turn calls the transport
*       layer's Rx endpoint open API to do the needful configuration to 
*       receive results from BCP on that device.
*
*       On success, this API returns a valid, non-zero Rx oject handle and returns 
*       NULL otherwise to indicate an error.
*
*  @param[in]
*       hBcp                BCP driver handle obtained using @a Bcp_open () 
*                           API.
*
*  @param[in]  
*       pBcpRxCfg           Input structure that holds configuration for the
*                           Rx object. MUST be a valid handle on local device.
*                           MUST be set to NULL on remote device.
*
*  @param[in]  
*       pRxEndpointCfg      Transport layer's Rx endpoint configuration. MUST be
*                           valid on remote device. Can be NULL or a valid handle
*                           on local device, depending on whether BCP output
*                           is going to be processed on local device or not.
*
*  @return      Bcp_RxHandle
*  @li          NULL                -   Error. Invalid Rx object configuration. 
*  @li          Valid Rx handle     -   Success. BCP Rx object, Rx queue successfully 
*                                       setup. 
*
*  @pre
*  @n   Valid BCP driver handle must be obtained by calling @a Bcp_open () API
*       by the application before calling this API. 
*
*  @post
*  @n   A Rx queue, flow successfully setup with input configuration specified on local 
*       device. On remote device, Rx remote endpoint is opened. The
*       Rx object can now be used to retrieve results from the BCP. 
* ============================================================================
*/
Bcp_RxHandle Bcp_rxOpen
(
    Bcp_DrvHandle               hBcp,
    Bcp_RxCfg*                  pBcpRxCfg,
    void*                       pRxEndpointCfg
)
{
    int32_t                     result;                
    Bcp_DrvInfo*                pBcpDrvInfo;
    Qmss_QueueHnd               hQmssRxQ = (Qmss_QueueHnd)NULL;
    Cppi_RxChInitCfg            rxChInitCfg;
    Cppi_ChHnd                  hCppiRxChan = NULL;  
    Cppi_FlowHnd                hCppiRxFlow;
    Qmss_QueueType              queueType=Qmss_QueueType_UNSPECIFIED;
    Bcp_RxInfo*                 pRxObjInfo = NULL;
    Bcp_RxGlobalInfo*           pRxGlobalObjInfo = NULL;
    int32_t                     instNum, flowId;
    Bcp_FlowInfo*               pFlowInfo;
    void*                       hRxEndpointInfo = NULL;
    Cppi_PSLoc                  psLocation;
    uint8_t                     isAllocated, bPSInfoPresent;

    /* Validate input. */
    if (!hBcp)
    {
        Bcp_osalLog ("Error! Invalid BCP driver handle passed! \n");
        return NULL;
    }

    pBcpDrvInfo   =   (Bcp_DrvInfo *) hBcp;
    instNum         =   pBcpDrvInfo->instNum;

    if ((pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL && !pBcpRxCfg) ||
        (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE && !pRxEndpointCfg)) 
    {
        Bcp_osalLog ("Error! Invalid Rx configuration handle provided! \n");
        return NULL;
    }

    /* Critical Section Start */
    Bcp_osalMultiCoreCsEnter ();            

    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        /************* Rx queue and Interrupt Setup. *****************/

        /* Open the Rx Queue for this Rx object based on input specified.
         *
         * We need to specify either a Rx queue number or a queue type.
         * If a queue is specified, assume that the queueType is also chosen and correct. 
         * If a queue is not specified, then set the queueType to be general purpose.
         */
        if (pBcpRxCfg->rxQNum == QMSS_PARAM_NOT_SPECIFIED)
        {
            /* No specific queue number specified. Lets pick next available 
             * Rx Queue from the General purpose queue pool since we do not
             * need any interrupt capability on it.
             */
            queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;            
        }
        if ((hQmssRxQ = Qmss_queueOpen (queueType, pBcpRxCfg->rxQNum, &isAllocated)) < 0)
        {
            Bcp_osalLog ("Error opening Rx Queue Number %d \n",pBcpRxCfg->rxQNum);
            goto return_error;
        }

        /* Get the receive queue number in case it wasnt specified */
        pBcpRxCfg->rxQNum       =   Qmss_getQIDFromHandle(hQmssRxQ);
    
        /* Configure interrupts using the application specified accumulator configuration. */
        if (pBcpRxCfg->bUseInterrupts)
        {
            /* Ensure that the accumulator channel is not already in use. */
            result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, pBcpRxCfg->accumCfg.channel);
            if (result != QMSS_ACC_SOK && result != QMSS_ACC_CHANNEL_NOT_ACTIVE)
            {
                Bcp_osalLog ("Error disabling high priority accumulator for channel : %d error code: %d\n",
                            pBcpRxCfg->accumCfg.channel, result);
                goto return_error;
            }

            /* Program the accumulator */
            if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &pBcpRxCfg->accumCfg)) != QMSS_ACC_SOK)
            {
                Bcp_osalLog ("Error programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                            pBcpRxCfg->accumCfg.channel, pBcpRxCfg->accumCfg.queMgrIndex, result);
                goto return_error;
            }
        }

        /************* Flow and Channel Setup. *****************/

        /* The Rx channel is mapped one to one with Rx QFIFO. */
        rxChInitCfg.channelNum          = pBcpRxCfg->tmFlowCfg.qfifo_out; 
        rxChInitCfg.rxEnable            = Cppi_ChState_CHANNEL_DISABLE; 
        if ((hCppiRxChan = Cppi_rxChannelOpen (gBcp_instanceInfo[instNum].hCppi, 
                                               &rxChInitCfg, 
                                               &isAllocated)) == NULL)
        {
            Bcp_osalLog ("Error opening Rx channel: %d \n", rxChInitCfg.channelNum);
            goto return_error;
        }
    
        /* Configure the Rx flow with the parameters specified by application */
        if ((hCppiRxFlow = Cppi_configureRxFlow (gBcp_instanceInfo[instNum].hCppi, 
                                                &pBcpRxCfg->flowCfg, 
                                                &isAllocated)) == NULL)
        {
            Bcp_osalLog ("Error configuring Rx flow \n");
            goto return_error;
        }

        /* Also enable Rx Channel */
        Cppi_channelEnable (hCppiRxChan);

        /* Setup TM flow entry corresponding to CPPI Rx flow just setup.
         *
         * CPPI Rx flows in BCP CDMA are mapped one to one with TM flows. 
         */
        flowId          =   Cppi_getFlowId (hCppiRxFlow);
        Bcp_setTmFlowEntry (&gBcp_instanceInfo[instNum].bcpLldObj, flowId, &pBcpRxCfg->tmFlowCfg);

        /* No Rx tunnel needed to receive packets from BCP if it is local. */
        hRxEndpointInfo =   NULL;

        /* PS Info read from Rx descriptors is no longer reliable because
         * of a CDMA H/W Bug. Hence save the settings so as to use on Rx
         * results.
         */    
        bPSInfoPresent  =   pBcpRxCfg->flowCfg.rx_psinfo_present;
        psLocation      =   (Cppi_PSLoc) pBcpRxCfg->flowCfg.rx_ps_location;

        if (pRxEndpointCfg)
        {
            if ((hRxEndpointInfo = pBcpDrvInfo->pFxnBcpTunnel_rxOpen (pRxEndpointCfg)) == NULL)
            {
                Bcp_osalLog ("Error creating BCP tunnel Rx local end point! \n");
                goto return_error;
            }
        }
    }
    else
    {
        /* Create a tunnel Rx endpoint only if the application is on remote device and wants to
         * receive data from BCP.
         */
        if ((hRxEndpointInfo = pBcpDrvInfo->pFxnBcpTunnel_rxOpen (pRxEndpointCfg)) == NULL)
        {
            Bcp_osalLog ("Error creating BCP tunnel Rx remote end point! \n");
            goto return_error;
        }
        bPSInfoPresent  =   0;
        psLocation      =   (Cppi_PSLoc)0;
    }

    /************* Save Info and Return. *****************/
    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        /* Get the next available slot from Rx object global database to store our info. */
        if ((result = Bcp_getNextAvailRxObjId (instNum)) == BCP_RETVAL_EFAILURE)
        {
            Bcp_osalLog ("Error! Max number of Rx objects exceeded. \n");
            goto return_error;
        }
        pRxGlobalObjInfo    =   &gBcp_instanceInfo[pBcpDrvInfo->instNum].Bcp_rxObjGlobalInfo[result];
        memset ((void*) pRxGlobalObjInfo, 0, sizeof (Bcp_RxGlobalInfo));
        pRxGlobalObjInfo->Id            =   result;
        pRxGlobalObjInfo->bIsValid      =   1;
        pRxGlobalObjInfo->flowId        =   flowId;   
        pRxGlobalObjInfo->rxQNum        =   pBcpRxCfg->rxQNum;

        /* Save the flow settings. */
        pFlowInfo   =   &gBcp_instanceInfo[instNum].Bcp_flowInfo[flowId];
        memset (pFlowInfo, 0, sizeof (Bcp_FlowInfo));  
        pFlowInfo->hCppiRxFlow          =   hCppiRxFlow;    
        pFlowInfo->flowId               =   flowId;   
        pFlowInfo->bPSInfoPresent       =   bPSInfoPresent;
        pFlowInfo->psLocation           =   psLocation;
    
        /* Increment the flow reference count to keep it around while in use by this Rx object. */
        pFlowInfo->refCnt ++;
    }
    else
    {
        /* In remote case, no BCP flow is created. Hence we dont need a BCP flow info            
         * object or a Rx global object.
         */
        result  =   -1;            
        flowId  =   -1;
    }

    /* Allocate memory for holding the Rx object info */
    if ((pRxObjInfo = Bcp_osalMalloc (sizeof (Bcp_RxInfo), FALSE)) == NULL)
    {
        Bcp_osalLog ("Error allocating memory for Rx object \n");
        goto return_error;
    }
    memset ((void*) pRxObjInfo, 0, sizeof (Bcp_RxInfo));

    pRxObjInfo->hBcp                =   hBcp;
    pRxObjInfo->hRxEndpointInfo     =   hRxEndpointInfo;
    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        pRxObjInfo->flowId          =   flowId;   
        pRxObjInfo->rxQNum          =   pBcpRxCfg->rxQNum;
        pRxObjInfo->cfg             =   *pBcpRxCfg;
        pRxObjInfo->hQmssRxQ        =   hQmssRxQ;
        pRxObjInfo->globalObjId		=	result;
        pRxObjInfo->bPSInfoPresent  =   bPSInfoPresent;
        pRxObjInfo->psLocation      =   psLocation;
        pRxObjInfo->hCppiRxChan     =   hCppiRxChan;
    }

    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* Done here. Release the CS lock */
    Bcp_osalMultiCoreCsExit ();

    /* Return success. Return global Rx object handle. */
    return ((Bcp_RxHandle) pRxObjInfo);

return_error:
    /* Close the Rx queue just opened. */
    if (hQmssRxQ)
        Qmss_queueClose (hQmssRxQ);

    if (hRxEndpointInfo)
        pBcpDrvInfo->pFxnBcpTunnel_rxClose (hRxEndpointInfo);                    

    if (hCppiRxChan)
        Cppi_channelClose (hCppiRxChan);

    /* Free up the Rx object */
    if (pRxObjInfo)
        Bcp_osalFree ((void *)pRxObjInfo, sizeof (Bcp_RxInfo), FALSE);

    /* Free up the Rx object's global counterpart too. */
    if (pRxGlobalObjInfo)
    	pRxGlobalObjInfo->bIsValid  =   0;

    Bcp_osalEndMemAccess (&gBcp_instanceInfo[instNum], sizeof (Bcp_InstanceInfo));

    /* exit critical section */
    Bcp_osalMultiCoreCsExit ();

    return NULL;
}

/**
* ============================================================================
*  @n@b Bcp_rxClose
*
*  @b   brief
*  @n   This API closes the Rx object, any Rx tunnel endpoints. 
*
*       On local device, the API closes the Rx flow and cleans up all 
*       associated queue and channel handles. 
*
*       On remote device, the API invokes the corresponding Rx tunnel endpoint 
*       close API.
*
*  @param[in]
*       hBcpRxInfo          BCP Rx object handle obtained using @a Bcp_rxOpen() 
*                           API.
*
*  @return     Bcp_RetVal
*  @li         BCP_RETVAL_EBAD_HANDLE  -   BCP Rx object close failed.
*  @li         BCP_RETVAL_SUCCESS      -   BCP Rx object successfully closed.
*
*  @pre
*  @n   Valid Rx object handle must be obtained using Bcp_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   BCP Rx object, Rx flow successfully closed on local device. Rx endpoint
*       close callback API invoked.
* ============================================================================
*/
Bcp_RetVal Bcp_rxClose 
(
    Bcp_RxHandle                hBcpRxInfo
)
{
    Bcp_RxInfo*                 pRxObjInfo;
    Bcp_RxGlobalInfo*           pRxGlobalObjInfo;
    uint16_t                    flowId;
    Bcp_DrvInfo*                pBcpDrvInfo;
    Bcp_FlowInfo*               pFlowInfo;

    /* Validate input handles and driver state  */
    if (!hBcpRxInfo)
    {
        Bcp_osalLog ("Invalid Receive Object Handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;	
    }

    pRxObjInfo      =   (Bcp_RxInfo *) hBcpRxInfo;
    pBcpDrvInfo     =   (Bcp_DrvInfo *) pRxObjInfo->hBcp;
    flowId          =   pRxObjInfo->flowId;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        /* Enter Critical Section */
        Bcp_osalMultiCoreCsEnter ();
            
        Bcp_osalBeginMemAccess (&gBcp_instanceInfo[pBcpDrvInfo->instNum], sizeof (Bcp_InstanceInfo));

        /* Clean up all BCP CPPI resources if BCP is local to the application */            
        pFlowInfo       =   &gBcp_instanceInfo[pBcpDrvInfo->instNum].Bcp_flowInfo[flowId];
        pRxGlobalObjInfo=   &gBcp_instanceInfo[pBcpDrvInfo->instNum].Bcp_rxObjGlobalInfo[pRxObjInfo->globalObjId];

        /* Close the Rx queue */
        Qmss_queueClose (pRxObjInfo->hQmssRxQ);

        /* Decrement reference count to release the flow */
        pFlowInfo->refCnt --;

        /* If flow not in use by any Rx object clean it up */
        if (pFlowInfo->refCnt == 0)
            Cppi_closeRxFlow (pFlowInfo->hCppiRxFlow);

        /* Disable the Rx channel */            
        Cppi_channelDisable (pRxObjInfo->hCppiRxChan);
        Cppi_channelClose (pRxObjInfo->hCppiRxChan);

        /* Free up the Rx object's global counterpart too. */
        pRxGlobalObjInfo->bIsValid  =   0;

        if (pRxObjInfo->hRxEndpointInfo)
            pBcpDrvInfo->pFxnBcpTunnel_rxClose (pRxObjInfo->hRxEndpointInfo);            

        Bcp_osalEndMemAccess (&gBcp_instanceInfo[pBcpDrvInfo->instNum], sizeof (Bcp_InstanceInfo));

        /* Release CS lock */
        Bcp_osalMultiCoreCsExit ();
    }
    else
    {
        /* Clean up the remote Rx endpoint we had created earlier */            
        pBcpDrvInfo->pFxnBcpTunnel_rxClose (pRxObjInfo->hRxEndpointInfo);            
    }

    /* Free up the Rx object space */
    Bcp_osalFree (pRxObjInfo, sizeof (Bcp_RxInfo), FALSE);
   
    /* Return Success */
    return BCP_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Bcp_findFlowIdByQueueNumber
*
*  @b   brief
*  @n   Given a Rx queue number, this API returns the corresponding flow Id 
*       associated from the driver's Rx object database.
*
*  @param[in]
*       hBcp                BCP driver handle obtained using @a Bcp_open () 
*                           API. Specifies which BCP instance to search for 
*                           a match.
*  @param[in]
*       rxQueueNumber       Rx queue number for which the flow Id lookup must
*                           be performed.
*
*  @return     Bcp_RetVal
*  @li         <0                       -   Flow Id retrieval failed. Unable 
*                                           to find a flow for the Rx queue 
*                                           number and BCP instance
*                                           provided in the driver database.
*  @li         >=0                      -   Success. Valid Flow Id returned.
*
*  @pre
*  @n   A valid driver handle must be obtained using @a Bcp_open () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Bcp_RetVal Bcp_findFlowIdByQueueNumber
(
    Bcp_DrvHandle               hBcp,
    uint32_t                    rxQueueNumber
)
{
    uint32_t                    i;
    Bcp_DrvInfo*                pBcpDrvInfo;
    Bcp_InstanceInfo*           pBcpInstInfo;

    /* Invalid driver handle specified. */
    if (!hBcp)
    {
        Bcp_osalLog ("Invalid driver handle specified \n");
        return BCP_RETVAL_EBAD_HANDLE;            
    }

    pBcpDrvInfo           =   (Bcp_DrvInfo *) hBcp;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE)
        return BCP_RETVAL_ENOT_SUPPORTED;            

    /* Enter critical section */
    Bcp_osalMultiCoreCsEnter ();

    Bcp_osalBeginMemAccess (&gBcp_instanceInfo[pBcpDrvInfo->instNum], sizeof (Bcp_InstanceInfo));

    pBcpInstInfo           =   (Bcp_InstanceInfo *)&gBcp_instanceInfo [pBcpDrvInfo->instNum];

    /* Search through the driver's database of all valid
     * Rx objects for the BCP instance corresponding to the 
     * driver handle passed to find a matching entry
     * for Rx queue number provided.
     */
    for (i = 0; i < BCP_MAX_NUM_RXOBJECTS; i ++)
    {
        if ((pBcpInstInfo->Bcp_rxObjGlobalInfo[i].bIsValid) && 
            (pBcpInstInfo->Bcp_rxObjGlobalInfo[i].rxQNum == rxQueueNumber))
        {
            /* Found a match. Return the flow Id corresponding to this object. */                
            break;                
        }
    }

    /* Check if we found a match or if we are here
     * because we are done searching through all entries
     * and found no matches.
     */
    if (i == BCP_MAX_NUM_RXOBJECTS)
    {
        /* Found no match. Return error */            
    
        /* Exit critical section */
        Bcp_osalMultiCoreCsExit ();

        return BCP_RETVAL_EINVALID_PARAMS;
    }

    /* Exit critical section */
    Bcp_osalMultiCoreCsExit ();

    /* Success. Return flow id found. */
    return (pBcpInstInfo->Bcp_rxObjGlobalInfo[i].flowId);
}

/**
* ============================================================================
*  @n@b Bcp_rxGetRxQueueNumber
*
*  @b   brief
*  @n   This API retrieves the given Rx object's Receive queue number.
*       This API is especially useful if the application didnt specify a 
*       Rx Queue number during @a Bcp_rxOpen () API and would like to
*       retrieve the queue number at a later point.
*
*       This API is supported only on BCP local device, i.e., if BCP driver
*       was initialized in "local" mode.
*
*  @param[in]
*       hBcpRxInfo          BCP Rx object handle obtained using 
*                           @a Bcp_rxOpen() API.

*
*  @return  Bcp_RetVal
*  @li      BCP_RETVAL_EBAD_HANDLE      -   Rx Queue Number retrieval failed.
*  @li      BCP_RETVAL_ENOT_SUPPORTED   -   This API not supported on BCP remote
*                                           device.
*  @li      >=0                         -   The destination queue number associated 
*                                           with the Rx object's flow.
*
*  @pre
*  @n   Valid Rx object handle must be obtained using @a Bcp_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Bcp_RetVal Bcp_rxGetRxQueueNumber
(
    Bcp_RxHandle                hBcpRxInfo
)
{
    Bcp_RxInfo*                 pRxObjInfo;
    Bcp_DrvInfo*                pBcpDrvInfo;

    /* Invalid Rx object handle specified. */
    if (!hBcpRxInfo)
    {
        Bcp_osalLog ("Invalid Rx object handle specified \n");
        return BCP_RETVAL_EBAD_HANDLE;            
    }

    pRxObjInfo  =   (Bcp_RxInfo *) hBcpRxInfo;        
    pBcpDrvInfo =   (Bcp_DrvInfo *) pRxObjInfo->hBcp;

    /* Return the destination queue number saved in the Rx object info */
    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
        return (pRxObjInfo->rxQNum);
    else
        return BCP_RETVAL_ENOT_SUPPORTED;            
}


/**
* ============================================================================
*  @n@b Bcp_rxGetFlowId
*
*  @b   brief
*  @n   This API retrieves a given Rx object's Rx flow id number.
*
*       This API is supported only on BCP local device, i.e., if BCP driver
*       was initialized in "local" mode.
*
*  @param[in]
*       hBcpRxInfo              BCP Rx object handle obtained using 
*                               @a Bcp_rxOpen() API.
*
*  @return  Bcp_RetVal
*  @li      BCP_RETVAL_EBAD_HANDLE      -   Flow Id value retrieval failed.
*                                           Invalid Rx object handle passed.
*  @li      BCP_RETVAL_ENOT_SUPPORTED   -   This API not supported on BCP remote
*                                           device.
*  @li         >=0                      -   The Flow Id associated with the Rx object
*                                           ranging between 0 and (BCP_MAX_NUM_FLOWS-1) 
*                                           (both inclusive).
*
*  @pre
*  @n   Valid Rx object handle must be obtained using @a Bcp_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Bcp_RetVal Bcp_rxGetFlowId
(
    Bcp_RxHandle                hBcpRxInfo
)
{
    Bcp_RxInfo*                 pRxObjInfo;
    Bcp_DrvInfo*                pBcpDrvInfo;

    /* Invalid Rx object handle specified. */
    if (!hBcpRxInfo)
    {
        Bcp_osalLog ("Invalid Rx object handle specified \n");
        return BCP_RETVAL_EBAD_HANDLE;            
    }

    pRxObjInfo  =   (Bcp_RxInfo *) hBcpRxInfo;        
    pBcpDrvInfo =   (Bcp_DrvInfo *) pRxObjInfo->hBcp;

    /* Return flow id */
    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
        return (pRxObjInfo->flowId);
    else
        return BCP_RETVAL_ENOT_SUPPORTED;
}

/**
* ============================================================================
*  @n@b Bcp_send
*
*  @b   brief
*  @n   Given a Tx object handle, descriptor containing data, and the descriptor 
*       size, this API pushes the data onto BCP Tx queue for processing. 
*
*       On remote device, this API instead invokes the BCP transport 
*       layer's send API to send out the packet to BCP using appropriate
*       transport mechanism.
*
*  @param[in]
*       hBcpTxInfo          Tx handle obtained using @a Bcp_txOpen () API.
*
*  @param[in]
*       hDrvBuffer          CPPI descriptor built by the application containing 
*                           data to be processed by BCP.
*
*  @param[in]
*       drvBufferLen        Size of the CPPI descriptor being sent.
*
*  @param[in]
*       pDestnAddress       Destination device info. On remote device, will contain
*                           information regarding BCP device and on local device
*                           must be set to NULL.
*
*  @return      Bcp_RetVal
*  @li          BCP_RETVAL_EBAD_HANDLE     -    Invalid input handles provided. 
*                                               BCP send failed.
*  @li          BCP_RETVAL_SUCCESS         -    Successfully sent data for BCP
*                                               processing.
*
*  @pre
*  @n   A valid Tx object handle must be obtained using @a Bcp_txOpen (), and a 
*       valid descriptor handle, length must be passed to this API.
*
*  @post
*  @n   Data pushed onto BCP Tx queue corresponding to the Tx object handle 
*       passed.
*
* ============================================================================
*/
Bcp_RetVal Bcp_send
(
    Bcp_TxHandle                hBcpTxInfo,
    Bcp_DrvBufferHandle         hDrvBuffer,
    uint32_t                    drvBufferLen,
    void*                       pDestnAddress
)
{
    Bcp_DrvInfo*                pBcpDrvInfo;        
    Bcp_TxInfo*                 pTxObjInfo;     
    Cppi_Desc*                  pCppiDesc;

#ifdef BCP_DRV_DEBUG
    /* Validate input handles */
    if (!hDrvBuffer)            
    {
        Bcp_osalLog("Invalid descriptor handle provided. \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }

    if (!hBcpTxInfo)
    {
        Bcp_osalLog ("Invalid Tx object handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }
#endif

    pTxObjInfo          =   (Bcp_TxInfo *) hBcpTxInfo;
    pBcpDrvInfo         =   (Bcp_DrvInfo *) pTxObjInfo->hBcp;
    pCppiDesc           =   (Cppi_Desc *) hDrvBuffer;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_LOCAL)
    {
        /* Push descriptor onto BCP Tx queue */
        Qmss_queuePushDescSize (pTxObjInfo->pTxQInfo->hQmssTxQ, pCppiDesc, drvBufferLen);    
    }
    else
    {
        /* Send data to BCP using BCP transport layer */
        return pBcpDrvInfo->pFxnBcpTunnel_send (pTxObjInfo->hTxEndpointInfo, pCppiDesc, drvBufferLen, pDestnAddress);
    }

    /* Return success. */
    return BCP_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Bcp_rxGetNumOutputEntries
*
*  @b   brief
*  @n   Given a Rx object handle, this API retrieves the number of BCP output
*       entries pending to be serviced by the application for it.
*
*  @param[in]
*       hBcpRxInfo          Rx handle obtained using @a Bcp_rxOpen () API.
*
*  @return  int32_t
*  @li      BCP_RETVAL_EBAD_HANDLE      -   Invalid input Rx handle provided.
*  @li      BCP_RETVAL_ENOT_SUPPORTED   -   API not supported on remote device.
*  @li          >=0                     -   Number of pending output entries 
*                                           for this Rx object.  
*
*  @pre
*  @n   A valid Rx object handle must be obtained using @a Bcp_rxOpen () 
*       API before calling this API.
*
*  @post
*  @n   Returns the number of BCP output entries available for this Rx object.
* ============================================================================
*/
Bcp_RetVal Bcp_rxGetNumOutputEntries 
(
    Bcp_RxHandle                hBcpRxInfo
)
{    
    Bcp_RxInfo*                 pRxObjInfo;
    Bcp_DrvInfo*                pBcpDrvInfo;

#ifdef BCP_DRV_DEBUG
    /* Validate input */
    if (!hBcpRxInfo)
    {
        Bcp_osalLog ("Invalid Rx object Handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo      =   (Bcp_RxInfo *)hBcpRxInfo;        
    pBcpDrvInfo     =   (Bcp_DrvInfo *)pRxObjInfo->hBcp;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE)
        return BCP_RETVAL_ENOT_SUPPORTED;

    if (pRxObjInfo->cfg.bUseInterrupts)
    {
        /* ISR not managed by driver. So no idea! */            
        return BCP_RETVAL_EBAD_HANDLE;            
    }
    else
    {
        /* Check the destination queue for the number of result packets to be serviced. */
        return (Qmss_getQueueEntryCount (pRxObjInfo->hQmssRxQ));
    }
}

/**
* ============================================================================
*  @n@b Bcp_recv
*
*  @b   brief
*  @n   Given an Rx object handle, this API checks if there is any output
*       available on the Rx queue from the BCP for its processing. If so, 
*       it does some basic validation on it and returns the descriptor handle, 
*       corresponding data buffer, its length, and any PS info found to the calling 
*       application. If no output found, this API either returns status to indicate 
*       the same.
*
*  @param[in]
*       hBcpRxInfo          Rx handle obtained using @a Bcp_rxOpen () API.
*
*  @param[out]
*       phDrvBuffer         Pointer to hold the output (descriptor) from BCP.
*
*  @param[out]
*       ppDataBuffer        Pointer to hold data buffer pointer obtained from BCP.
*
*  @param[out]
*       pDataBufferLen      Pointer to hold data buffer length, i.e.,holds the number of
*                           bytes available to application in the data buffer for use.
*
*  @param[out]
*       ppPSInfo            Pointer to PS Info pointer obtained from BCP.
*
*  @param[out]
*       pPSInfoLen          Pointer to PS Info length, i.e.,holds the number of bytes 
*                           of PS info available for application to parse.
*
*  @param[out]
*       pFlowId             Flow Id read from the descriptor's Source Tag lower
*                           order 8 bits.
*
*  @param[out]
*       pSrcId              Source Id read from the descriptor's Source Tag higher
*                           order 8 bits.
*
*  @param[out]
*       pDestnTagInfo       Destination tag info read from the descriptor.
*
*  @return      Bcp_RetVal
*  @li          BCP_RETVAL_EBAD_HANDLE         -   Invalid input Rx object handle provided.
*  @li          BCP_RETVAL_ENO_RESULT          -   No output available yet on the Rx queue.
*  @li          BCP_RETVAL_SUCCESS             -   Successfully retrieved output and stored in 
*                                                  output parameter handles.
*
*  @pre
*  @n   A valid Rx object handle must be obtained using @a Bcp_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   If BCP output available, it is validated and handed over to application
*       for further processing, otherwise appropriate status is returned.
* ============================================================================
*/
Bcp_RetVal Bcp_recv
(
    Bcp_RxHandle                hBcpRxInfo,
    Bcp_DrvBufferHandle*        phDrvBuffer,
    uint8_t**                   ppDataBuffer,
    uint32_t*                   pDataBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
)
{
    Bcp_RxInfo*                 pRxObjInfo;
    Cppi_Desc*                  pCppiDesc;
    Bcp_DrvInfo*                pBcpDrvInfo;

#ifdef BCP_DRV_DEBUG
    /* Validate input */
    if (!hBcpRxInfo || !phDrvBuffer || !ppDataBuffer || !pDataBufferLen 
        || !ppPSInfo || !pPSInfoLen || !pFlowId || !pDestnTagInfo
       )
    {
        Bcp_osalLog ("Invalid input handles provided \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo      =   (Bcp_RxInfo *) hBcpRxInfo;
    pBcpDrvInfo     =   (Bcp_DrvInfo *) pRxObjInfo->hBcp;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE)
    {
        pBcpDrvInfo->pFxnBcpTunnel_recv (pRxObjInfo->hRxEndpointInfo, (void **) &pCppiDesc);            

        if (*phDrvBuffer == NULL)
        {
            /* No BCP output ready yet. */            
            return BCP_RETVAL_ENO_RESULT;                
        }

        /* Get the result info from the descriptor */
        Bcp_rxProcessDesc (hBcpRxInfo,
                           pCppiDesc, 
                           phDrvBuffer,
                           ppDataBuffer,
                           pDataBufferLen,
                           ppPSInfo,
                           pPSInfoLen,
                           pFlowId,
                           pSrcId,
                           pDestnTagInfo);
    }
    else
    {
        /* Check the Rx queue for any pending output from BCP. */
        if (!pRxObjInfo->cfg.bUseInterrupts)
        {
            /* No interrupts enabled. 
             *
             * Check the Rx object's destination queue to see if a
             * result has arrived from the BCP engine or not.
             */
            if ((pCppiDesc = Qmss_queuePop (pRxObjInfo->hQmssRxQ)) == NULL)
            {
                /* No BCP output ready yet. */            
                return BCP_RETVAL_ENO_RESULT;                
            }

            /* Get the result info from the descriptor */
            Bcp_rxProcessDesc (hBcpRxInfo,
                               pCppiDesc, 
                               phDrvBuffer,
                               ppDataBuffer,
                               pDataBufferLen,
                               ppPSInfo,
                               pPSInfoLen,
                               pFlowId,
                               pSrcId,
                               pDestnTagInfo);
        }
        else
        {
            /* Interrupts enabled for this Rx object.
             *
             * Driver is not setup to manage the ISR for this Rx object. 
             * Cannot use this API.
            */
            return BCP_RETVAL_EBAD_HANDLE;
        }
    }

    /* Return success. */
    return BCP_RETVAL_SUCCESS;                
}

/**
* ============================================================================
*  @n@b Bcp_rxProcessDesc
*
*  @b   brief
*  @n   This API can be used to retrieve the various fields from
*       the output descriptor received from the engine.
*
*  @param[in]
*       hBcpRxInfo          Rx handle obtained using @a Bcp_rxOpen () API.
*
*  @param[in]
*       pCppiDesc           Rx descriptor containing the output that needs to be
*                           processed by this API.
*
*  @param[out]
*       phDrvBuffer         Pointer to hold BCP driver output handle.
*
*  @param[out]
*       ppDataBuffer        Pointer to hold data buffer pointer obtained from BCP.
*
*  @param[out]
*       pDataBufferLen      Pointer to data buffer length, i.e.,holds the number of
*                           bytes available to application in the output data buffer for
*                           parsing.
*  @param[out]
*       ppPSInfo            Pointer to PS Info pointer obtained from BCP.
*
*  @param[out]
*       pPSInfoLen          Pointer to PS Info length, i.e.,holds the number of bytes 
*                           of PS info available for application to parse.
*  @param[out]
*       pFlowId             Flow Id read from the descriptor's Source Tag lower
*                           order 8 bits.
*
*  @param[out]
*       pSrcId              Source Id read from the descriptor's Source Tag higher
*                           order 8 bits.
*  @param[out]
*       pDestnTagInfo       Destination tag info read from the descriptor.
*
*  @return     Bcp_RetVal
*  @li         BCP_RETVAL_EBAD_HANDLE      -   Bad input Handle provided.
*  @li         BCP_RETVAL_SUCCESS          -   Success.
*
*  @pre
*  @n   A valid CPPI descriptor handle must be sent to this API.
* ============================================================================
*/
 Bcp_RetVal Bcp_rxProcessDesc 
(
    Bcp_RxHandle                hBcpRxInfo,
    Cppi_Desc*                  pCppiDesc,
    Bcp_DrvBufferHandle*        phDrvBuffer,
    uint8_t**                   ppDataBuffer,
    uint32_t*                   pDataBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
)
{
    Cppi_DescTag                cppiTagInfo;
    Cppi_DescType               descType;
    Bcp_RxInfo*                 pRxObjInfo;

#ifdef BCP_DRV_DEBUG
    /* Validate input */
    if (!hBcpRxInfo || !phDrvBuffer || !ppDataBuffer || !pDataBufferLen 
        || !ppPSInfo || !pPSInfoLen || !pFlowId || !pDestnTagInfo
       )
    {
        Bcp_osalLog ("Invalid input Handle provided \n");
        return BCP_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo  =   hBcpRxInfo;

    /* Invalidate descriptor to ensure our reads/writes are consistent with 
     * the memory in case descriptor is in cacheable memory.
     * Since we dont manage the Tx/Rx descriptors, we dont know their sizes. 
     * We pass the Rx handle to the application to help it figure out the
     * descriptor size.
     */
    Bcp_osalBeginDescMemAccess (pRxObjInfo, pCppiDesc);
    
    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last 
     * 4 bits of the address.
     */
    pCppiDesc = (void*) (QMSS_DESC_PTR (pCppiDesc));    
    descType = Cppi_getDescType (pCppiDesc);

    /* Get the source and destination tag information */
    cppiTagInfo         =   Cppi_getTag (descType, pCppiDesc);
    *pSrcId             =   cppiTagInfo.srcTagHi;
    *pFlowId            =   cppiTagInfo.srcTagLo;
    *pDestnTagInfo      =   ((cppiTagInfo.destTagHi << 8) | (cppiTagInfo.destTagLo));

    /* Store the descriptor in the output driver buffer handle.
     *
     * Required when freeing the descriptor to restore it back to Rx FDQ
     */
    *phDrvBuffer        =   (Bcp_DrvBufferHandle) pCppiDesc;

    /* Get Data buffer containing the output and its length */
    Cppi_getData (descType, pCppiDesc, ppDataBuffer, pDataBufferLen);

    /* Get PS data buffer and its length from the descriptor */
    Cppi_getPSData (descType, pRxObjInfo->psLocation, pCppiDesc, ppPSInfo, pPSInfoLen);

    /* Check if any PS info is in the result SOP Buffer. If so,
     * move the data buffer pointer to point to the actual data and not the PS info.
     *
     * Due to a CDMA H/W Bug, the PS Location Info read from the descriptor
     * is no longer reliable. Hence, use the PS Info stored from the flow
     * configuration the Rx object uses.
     */ 
    if ((descType == Cppi_DescType_HOST) && 
        (pRxObjInfo->bPSInfoPresent) && 
        (pRxObjInfo->psLocation == Cppi_PSLoc_PS_IN_SOP))
    {
        *ppDataBuffer +=  *pPSInfoLen;                      
    }

    return BCP_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Bcp_rxFreeResult
*
*  @b   brief
*  @n   Given a Rx object handle and driver buffer handle obtained from 
*       @a Bcp_recv () API, this API frees the driver buffer handle and 
*       restores the associated Rx descriptor back to its Rx Free descriptor 
*       queue. This API must be called by the application once its done processing 
*       the result, otherwise the driver could run out of output buffers eventually 
*       if none restored in time. 
*
*       On remote device, this API instead invokes the free receive buffer API
*       of the BCP transport layer.
*
*  @param[in]
*       hBcpRxInfo          Rx handle obtained using @a Bcp_rxOpen () API.
*
*  @param[in]
*       hDrvBuffer          BCP driver output buffer handle obtained using 
*                           @a Bcp_recv () API.
*
*  @param[in]
*       drvBufferLen        Size of the CPPI descriptor (BCP output buffer) 
*                           being freed.
*
*  @return     Bcp_RetVal
*  @li         BCP_RETVAL_EBAD_HANDLE  -   Invalid input provided. Free failed.
*  @li         BCP_RETVAL_SUCCESS      -   Free succeeded.
*
*  @pre
*  @n   A valid driver buffer handle should have been obtained using 
*       @a Bcp_recv () before calling this API to free the descriptor.
*
*  @post
*  @n   The result buffers passed are freed up and restored to the Rx object's free
*       descriptors.
* ============================================================================
*/
Bcp_RetVal Bcp_rxFreeRecvBuffer 
(
    Bcp_RxHandle                hBcpRxInfo,
    Bcp_DrvBufferHandle         hDrvBuffer,
    uint32_t                    drvBufferLen
)
{
    Qmss_Queue                  rxReturnQInfo;
    Cppi_Desc*                  pCppiDesc;
    Bcp_DrvInfo*                pBcpDrvInfo;
    Bcp_RxInfo*                 pRxObjInfo;
    Qmss_QueueHnd               qHandle;

#ifdef BCP_DRV_DEBUG
    /* Validate input */
    if (!hBcpRxInfo || !hDrvBuffer)
        return BCP_RETVAL_EBAD_HANDLE;
#endif

    pRxObjInfo      =   (Bcp_RxInfo *) hBcpRxInfo;
    pCppiDesc       =   (Cppi_Desc*) hDrvBuffer;
    pBcpDrvInfo     =   (Bcp_DrvInfo*) pRxObjInfo->hBcp;

    if (pBcpDrvInfo->mode == Bcp_DrvMode_REMOTE)
    {
        pBcpDrvInfo->pFxnBcpTunnel_freeRecvBuffer (pRxObjInfo->hRxEndpointInfo, pCppiDesc, drvBufferLen); 
    }
    else
    {
        rxReturnQInfo = Cppi_getReturnQueue (Cppi_getDescType (pCppiDesc), pCppiDesc);
        qHandle = Qmss_getQueueHandle(rxReturnQInfo);

        /* Push descriptor back to free queue */
	    Qmss_queuePushDescSize (qHandle, pCppiDesc, drvBufferLen);
    }

    /* Return success. */
    return BCP_RETVAL_SUCCESS;
}

/**
@}
*/
