/** 
 *   @file  fftc.c
 *
 *   @brief  
 *      This file contains the FFTC driver's higher layer API implementations.
 *
 *      The FFTC Higher Layer presents a useful set of APIs to submit FFT requests
 *      and retrieve its results from the engine using the CPPI, QMSS 
 *      library APIs and the FFTC LLD APIs.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
/* FFTC types include */
#include <fftc_types.h>

/* FFTC OSAL include */
#include <fftc_osal.h>

/* FFTC include */
#include <ti/drv/fftc/fftc.h>

/* FFTC private definitions include file */
#include <ti/drv/fftc/include/fftc_pvt.h>

/* Standard C definitions include */
#include <string.h>

/***************************************************************
 ******************** GLOBAL VARIABLES *************************
 **************************************************************/
/** Global variable to hold the FFTC driver' state info */
extern Fftc_InstanceInfoObj        Fftc_instanceInfo [FFTC_MAX_NUM_INSTANCES];

/** @addtogroup FFTC_FUNCTION
 @{ */

/***************************************************************
 ******************* FFTC Internal APIs ************************
 **************************************************************/
 
/**
 * ============================================================================
 *  @n@b Fftc_setupCppiDesc
 *
 *  @b  brief
 *  @n  This API sets up a free queue for the application and populates it with some
 *      pre-initialized CPPI descriptors from a given memory region. Once a 
 *      memory region is formatted into descriptors as per input specified here,
 *      it cannot be reformatted to a different param set. Hence, this API 
 *      must be called to setup only global free queues that can be shared by
 *      various application threads using this driver.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]    
 *      pCppiDescCfg        Handle to initial CPPI descriptor configuration structure.
 *
 *  @return     Qmss_QueueHnd
 *  @li         NULL            -   Error. Invalid descriptor configuration provided.
 *  @li         Valid Handle    -   Success.Valid CPPI free queue handle returned
 *
 *  @pre
 *  @n  The input configuration structure handle passed 'pCppiDescCfg' must hold
 *      valid configuration for this API to succeed. 
 *
 *  @post
 *  @n  Specified CPPI memory region is formatted as per input into various
 *      descriptors and populated on next available QMSS free queue.
 * ============================================================================
 */
static Qmss_QueueHnd Fftc_setupCppiDesc
(
    Fftc_CppiDescCfg*       pCppiDescCfg
)
{
    Cppi_DescCfg            cppiDescCfg;
    Qmss_QueueHnd           hQmssFreeQ;
    uint32_t                numAllocated;

    if (!pCppiDescCfg)
    {
        /* Invalid descriptor configuration handle. Return error. */            
        return ((Qmss_QueueHnd) NULL);
    }

    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));

    cppiDescCfg.memRegion                       =   (Qmss_MemRegion) pCppiDescCfg->descMemRegion;
    cppiDescCfg.descNum                         =   pCppiDescCfg->numDesc;
    cppiDescCfg.destQueueNum                    =   QMSS_PARAM_NOT_SPECIFIED;     
    cppiDescCfg.queueType                       =   Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    cppiDescCfg.initDesc                        =   Cppi_InitDesc_INIT_DESCRIPTOR;
    cppiDescCfg.descType                        =   pCppiDescCfg->descType;

    /* By default return descriptors to tail of queue */
    cppiDescCfg.returnPushPolicy                =   Qmss_Location_TAIL;    

    if (pCppiDescCfg->descType == Cppi_DescType_HOST)
    {
        /* By default always return entire packet to this free queue */
        cppiDescCfg.cfg.host.returnPolicy       =   Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;    

        /* By default, assume that PS Data is always present in start of SOP buffer */
        cppiDescCfg.cfg.host.psLocation         =   Cppi_PSLoc_PS_IN_SOP;         
    }
    else if (pCppiDescCfg->descType == Cppi_DescType_MONOLITHIC)
    {
        /* By default, set the data offset to right after the 
         * mandatory monolithic header words
         */
        cppiDescCfg.cfg.mono.dataOffset         =   FFTC_CPPI_MONOLITHIC_DESC_SIZE;    
    }

    /* By default, we assume free q num < 4K, hence qMgr = 0 */
    cppiDescCfg.returnQueue.qMgr                =   QMSS_PARAM_NOT_SPECIFIED;    
    /* Recycle back to the same Free queue by default. */
    cppiDescCfg.returnQueue.qNum                =   QMSS_PARAM_NOT_SPECIFIED; 
    cppiDescCfg.epibPresent                     =   Cppi_EPIB_NO_EPIB_PRESENT;

    /* Initialize the descriptors, create a free queue and push descriptors to the free Queue */
    if ((hQmssFreeQ = Cppi_initDescriptor (&cppiDescCfg, &numAllocated)) <= 0)
    {
        Fftc_osalLog ("Error Initializing Free Descriptors of type: %d in region: %d, Error: %d \n", 
                        pCppiDescCfg->descType, pCppiDescCfg->descMemRegion, hQmssFreeQ);
        return ((Qmss_QueueHnd) NULL);
    }        
    else
    {
        Fftc_osalLog ("Free descriptor queue %d successfully setup. Descriptors allocated of type (%d): %d \n", 
                      hQmssFreeQ, pCppiDescCfg->descType, numAllocated);
        return (hQmssFreeQ);
    }
}

/**
 * ============================================================================
 *  @n@b Fftc_getFreeQ
 *
 *  @b  brief
 *  @n  Given a descriptor type and size, this API searches the application's 
 *      set of free queues (setup during driver open time) to find one matching 
 *      fitting the parameters. This API assumes that all the free queues setup 
 *      during driver open time were setup in the ascending order of the descriptor
 *      sizes.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]
 *      pFFTCUserInfo       FFTC driver handle obtained using @a Fftc_open () 
 *                          API.
 *  @param[in]    
 *      descSize            Size of the descriptor to find.
 *
 *  @param[in]    
 *      descType            CPPI descriptor type. Valid values are: Cppi_DescType_HOST,
 *                          Cppi_DescType_MONOLITHIC.
 *
 *  @return     Qmss_QueueHnd
 *  @li         NULL            -   Error. No matching free queue found in driver's setup.
 *  @li         Valid Handle    -   Success. Valid CPPI free queue handle returned.
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to setup the driver and initialize
 *      application's free queues before calling this API.
 *
 *  @post
 *  @n  Matching free queue handle is returned to the caller.
 * ============================================================================
 */
static Qmss_QueueHnd Fftc_getFreeQ
(
    Fftc_UserInfo*      pFFTCUserInfo,
    uint32_t            descSize,
    Cppi_DescType       descType
)
{
    uint32_t            i;
    Qmss_QueueHnd       hQmssFreeQ;

    /* validate input */
    if (!pFFTCUserInfo)
    {
        return ((Qmss_QueueHnd) NULL);            
    }

    /* This is assuming that all free queues are ordered in the ascending order of 
     * descriptor sizes.
     */

    /* Not a valid descriptor type for data buffer */
    if (descType != Cppi_DescType_HOST && descType != Cppi_DescType_MONOLITHIC)
        return ((Qmss_QueueHnd) NULL);    

    for (i = 0; i < pFFTCUserInfo->drvCfg.cppiNumFreeDescCfg; i ++)
    {
        if ((descSize   <=  pFFTCUserInfo->drvCfg.cppiFreeDescCfg [i].descSize) &&
            (descType   ==  pFFTCUserInfo->drvCfg.cppiFreeDescCfg [i].descType))
        {
            /* Found the free queue matching the descriptor size */                
            break;     
        }
    }  

    if (i != pFFTCUserInfo->drvCfg.cppiNumFreeDescCfg)
    {
        /* Return the free queue handle */            
        hQmssFreeQ = pFFTCUserInfo->hQmssFreeDescQ [i];
    }
    else if (i == pFFTCUserInfo->drvCfg.cppiNumFreeDescCfg)
    {
        /* No matching free queue found for the buffer size provided. */            
       hQmssFreeQ = (Qmss_QueueHnd)NULL;            
    }

    return (hQmssFreeQ);
}

/**
 * ============================================================================
 *  @n@b Fftc_cleanFreeQBuffers
 *
 *  @b  brief
 *  @n  This API cleans up a Tx/Rx object free descriptor queue created earlier using
 *      @a Fftc_allocFreeQBuffers () API. It detaches and frees any
 *      buffers if applicable, (buffers are freed only if descriptor is of host
 *      type) and finally restores the descriptor back to the application's global
 *      free queue provided.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]    
 *      hQmssFreeQ          Tx/Rx object free descriptor queue that needs to be 
 *                          de-allocated.
 *
 *  @param[in]    
 *      hQmssGblFreeQ       Application's global free queue to which the Rx/Tx object's
 *                          clean descriptors must be returned to.
 *
 *  @param[in]    
 *      numBuffers          Number of descriptors that need to be restored.
 *
 *  @param[in]    
 *      bufferSize          Size of the buffer enqueued on the descriptors.
 *
 *  @param[in]    
 *      descSize            CPPI descriptor size.
 *
 *  @return     None
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to setup the driver and initialize
 *      application's free queues. Also a Tx/Rx local free queue must have been
 *      successfully setup using  @a Fftc_allocFreeQBuffers () API before this
 *      API is called.
 *
 *  @post
 *  @n  The Tx/Rx object local free queue 'hQmssFreeQ''s buffers are cleaned up, and all
 *      its descriptors are returned to the application's free queue corresponding to
 *      'hQmssGblFreeQ'. 
 * ============================================================================
 */
static void Fftc_cleanFreeQBuffers
(
    Qmss_QueueHnd           hQmssFreeQ,
    Qmss_QueueHnd           hQmssGblFreeQ,
    uint32_t                numBuffers,
    uint32_t                bufferSize,
    uint32_t                descSize
)
{
    uint32_t                i;
    Cppi_Desc*              pCppiDesc = NULL;
    uint8_t*                pDataBuffer = NULL;
    uint32_t                bufferLen;
    Qmss_Queue              qInfo;
    Cppi_DescType			descType;

    /* Validate Input */
    if (!hQmssFreeQ || !hQmssGblFreeQ || !numBuffers)
    {
        /* erroneous input. */            
        return;            
    }

    qInfo = Qmss_getQueueNumber (hQmssGblFreeQ);

    /* Clean up the buffers allocated on the free descriptors */
    for (i = 0; i < numBuffers; i ++)
    {
        /* Free up all the descriptors en-queued to this queue */                    
        if ((pCppiDesc = Qmss_queuePop (hQmssFreeQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (void*) (QMSS_DESC_PTR (pCppiDesc));    
        descType = Cppi_getDescType (pCppiDesc);

        /* Get Data buffer and its length from the descriptor */
        Cppi_getData (descType, pCppiDesc, &pDataBuffer, &bufferLen);

        if (descType == Cppi_DescType_HOST)
        {
            Fftc_osalFree ((void*) (pDataBuffer), bufferSize, TRUE);
        }

        /* Reset the return policy to the global free queue. Is it better to use Q Divert here?? */
        Cppi_setReturnQueue (descType, pCppiDesc, qInfo);
        
        /* Commit descriptor contents to physical memory in case its
         * allocated from cacheable memory.
         */
        Fftc_osalEndDescMemAccess (pCppiDesc, descSize);

        /* Return descriptor back to global free queue */
        if(descSize > 256)
            descSize = 256;
        Qmss_queuePushDescSize (hQmssGblFreeQ, pCppiDesc, descSize);    
    }        

    /* Finally close the free queue */
    Qmss_queueClose (hQmssFreeQ);

    /* Done cleaning up. Return. */
    return;
}

/**
 * ============================================================================
 *  @n@b Fftc_allocFreeQBuffers
 *
 *  @b  brief
 *  @n  This API creates a Tx/Rx object local free descriptor queue and attaches 
 *      descriptors as per the number, size, type provided in the input parameters 
 *      'numBuffers', 'bufferSize', and 'descType' respectively. It also pre-allocates 
 *      memory for the buffers on host descriptors and populates all applicable 
 *      known fields. On successful completion, this API returns the handle to the 
 *      Tx/Rx object's local free queue, number of descriptors successfully allocated 
 *      and the application's free queue handle from which the descriptors were 
 *      retrieved.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]
 *      pFFTCUserInfo       FFTC driver handle obtained using @a Fftc_open () 
 *                          API.
 *
 *  @param[in]    
 *      numBuffers          Number of descriptors that need to be allocated.
 *
 *  @param[in]    
 *      bufferSize          Size of the buffer to be enqueued on the descriptors.
 *
 *  @param[in]    
 *      descType            CPPI descriptor type. Valid values are: Cppi_DescType_HOST,
 *                          Cppi_DescType_MONOLITHIC.
 *
 *  @param[in]    
 *      descSize            CPPI descriptor size.
 *
 *  @param[out]    
 *      pNumAllocated       Number of descriptors succesfully queued on this object's free queue.
 *
 *  @param[out]    
 *      phGlblFreeQ         The FFTC application's free queue handle from which the 
 *                          descriptors for this object have been allocated from.
 *
 *  @return     Qmss_QueueHnd
 *  @li         NULL            -   Error. Invalid input. No free descriptor 
 *                                  queue allocated with buffers.
 *  @li         Valid Handle    -   Success. Valid CPPI free descriptor queue 
 *                                  handle returned. 
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to setup the driver and initialize
 *      applicaiton's set of free queues before calling this API.
 *
 *  @post
 *  @n  A Tx/Rx object's local free queue is created with descriptors and 
 *      pre-allocated buffers. 
 * ============================================================================
 */
static Qmss_QueueHnd Fftc_allocFreeQBuffers
(
    Fftc_UserInfo*          pFFTCUserInfo,
    uint32_t                numBuffers,
    uint32_t                bufferSize,
    Cppi_DescType           descType,
    uint32_t                descSize,
    int32_t*                pNumAllocated,
    Qmss_QueueHnd*          phGlblFreeQ
)
{
    uint8_t                 isAllocated;        
    int32_t                 i;
    Cppi_Desc*              pCppiDesc;
    uint8_t*                pDataBuffer;
    Qmss_QueueHnd           hQmssGblFreeQ;
    Qmss_QueueHnd           hQmssFreeQ;
    Qmss_Queue              qInfo;

    /* Validate input */
    if (!pFFTCUserInfo || !numBuffers || !bufferSize || !pNumAllocated || !phGlblFreeQ)
    {
        /* Invalid input parameters. Return error. */            
        return ((Qmss_QueueHnd) NULL);            
    }

    *pNumAllocated  = 0;
    *phGlblFreeQ    = NULL;

    /* Get the Global free queue matching the buffer size specified */
    if ((hQmssGblFreeQ = Fftc_getFreeQ (pFFTCUserInfo, descSize, descType)) == (Qmss_QueueHnd) NULL)
    {
        Fftc_osalLog ("No Global Free Descriptor queue found matching the buffer size \n");
        return ((Qmss_QueueHnd) NULL);
    }    

    /* Open a free queue to hold the pre-allocated buffers. */
    if ((hQmssFreeQ = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	{
        Fftc_osalLog ("Error opening Free Queue \n");
		return ((Qmss_QueueHnd) NULL);
	}    

    qInfo = Qmss_getQueueNumber (hQmssFreeQ);

    /* Allocate and attach the buffers to the free descriptor queue if the requested
     * descriptor type is "Host" descriptors.
     */
    for (i = 0; i < numBuffers; i ++)
    {
        /* Get a free descriptor */
        if ((pCppiDesc = Qmss_queuePop (hQmssGblFreeQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor address, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (void*) (QMSS_DESC_PTR (pCppiDesc));    

        /* Add data buffer is its a host descriptor. */
        if (descType == Cppi_DescType_HOST)
        {
            pDataBuffer             =   (uint8_t *) Fftc_osalMalloc (bufferSize, TRUE); 
            if (!pDataBuffer)
            {
                break;                
            }

            Cppi_setData (descType, pCppiDesc, pDataBuffer, bufferSize);

            /* Save original buffer information */
            Cppi_setOriginalBufInfo (descType, pCppiDesc, pDataBuffer, bufferSize);
        }
        else 
        {
            /* Setup the data buffer offset for monolithic descriptors. */
            Cppi_setDataOffset (descType, pCppiDesc, FFTC_CPPI_MONOLITHIC_DESC_SIZE);
        }                

        /* Setup the Completion queue:
         *
         * Setup the return policy for this desc to return to the free q we just
         * setup instead of the global free queue.
         */
        Cppi_setReturnQueue (descType, pCppiDesc, qInfo);

        /* Set packet length */
        Cppi_setPacketLen (descType, pCppiDesc, bufferSize);

        /* Commit descriptor contents to physical memory in case its
         * allocated from cacheable memory.
         */
        Fftc_osalEndDescMemAccess (pCppiDesc, descSize);

        /* Push descriptor back to free queue */
        if(descSize > 256)
            descSize = 256;
        Qmss_queuePushDescSize (hQmssFreeQ, pCppiDesc, descSize);           
    }

    /* Done populating the descriptors with buffers. 
     *
     * Return the free queue handle and the number of buffers allocated.
     */
    *pNumAllocated  = i;
    *phGlblFreeQ    = hQmssGblFreeQ; 

    return (hQmssFreeQ);
}

/**
* ============================================================================
*  @n@b Fftc_txQueueOpen
*
*  @b   brief
*  @n   If not already open, this API sets up the FFTC Transmit queue corresponding 
*       to the queue number specified in 'fftcTxQNum' (valid values 0-3) and the
*       FFTC instance specified by 'pFFTCInstInfo' handle. If open already, the 
*       queue configuration is skipped.
*
*       As part of the queue configuration this API opens the CPPI queue, a Tx 
*       channel for it and also configures its FFTC Queue local registers.
*
*       The driver allows an FFTC queue to be opened in 2 modes, "shared" / "dedicated"
*       modes; When opened in 'shared' mode by setting 'bSharedMode' to 1,
*       multiple applications could share the same Tx queue to submit FFT requests
*       to the engine. However, if the queue is opened in 'dedicated' mode by setting
*       'bSharedMode' to 0, then the driver ensures that no other application
*       except the one that has opened it can use the queue for FFT processing.
*
*       If the queue had already been opened earlier and in 'shared' mode, then 
*       a call to this API simply increments it's reference count and 
*       returns the handle to the opened queue without modifying its configuration.
*       A call to this API on an already open queue in 'dedicated' mode returns 
*       an error.
*
*  @param[in]
*       pFFTCInstInfo       FFTC instance information handle.
*
*  @param[in]
*       fftcTxQNum          FFTC Transmit queue number for which the setup needs 
*                           to be performed. Valid values are between 0-3.
*
*  @param[in]
*       pFFTQCfg            Configuration structure that holds the settings for
*                           various FFTC Queue local configuration registers.
*
*  @param[in]
*       bSharedMode         When this flag is set to 0, the driver ensures that
*                           no other thread/application uses the same FFTC transmit
*                           queue and also puts additional checks to ensure that
*                           the configuration for this queue cannot be overridden
*                           using CPPI packets on the data path. When set to 1, the
*                           queue is opened in shared mode and its configuration can
*                           be changed using CPPI packets by multiple applications.
*
*  @return     Fftc_TxQInfo*
*  @li         NULL         -   Tx queue setup failed as per input provided.
*  @li         Valid Handle -   Success. Valid Tx queue handle returned.
*
*  @pre
*  @n   @a Fftc_open () API must have been called to initialize the driver's
*       global state before calling this API.
*
*  @post
*  @n   FFTC Queue Local Configuration registers for the queue 'fftcTxQNum' are
*       configured according to values specified in 'pFFTQCfg' structure and 
*       the corresponding CPPI channels and queues are opened too if called for the
*       first time. Reference counter for the queue is incremented to keep track
*       of all the application threads using this queue.
* ============================================================================
*/
static Fftc_TxQInfo* Fftc_txQueueOpen
(
    Fftc_InstanceInfoObj*       pFFTCInstInfo,
    Fftc_QueueId                fftcTxQNum,
    Fftc_QLocalCfg*             pFFTQCfg, 
    uint8_t                     bSharedMode
)
{
    uint8_t                     isAllocated;        
    Cppi_TxChInitCfg            txChCfg;
    Qmss_QueueHnd               hQmssTxQ = (Qmss_QueueHnd)NULL;
    Fftc_GlobalCfg              fftGlobalCfg_r;
    Cppi_ChHnd                  hCppiTxChan = NULL, hCppiRxChan = NULL;  
    void*                      	hFFTCTxQueueInfo;
    Cppi_RxChInitCfg            rxChInitCfg;

    /* If this is the first time we are initializing this the FFTC transmit queue,
     * setup the configuration, otherwise just increment the reference count
     * and return the queue handle if its being opened in "shared" mode.
     */
    if (!pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].refCnt)
    {
        txChCfg.channelNum      =   fftcTxQNum;       /* CPPI channels are mapped one-one to the FFTC Tx queues */
        txChCfg.txEnable        =   Cppi_ChState_CHANNEL_DISABLE;  /* Disable the channel for now. */
        txChCfg.filterEPIB      =   0;
        txChCfg.filterPS        =   0;
        txChCfg.aifMonoMode     =   0;

        /* Get the FFTC Control register contents */
        Fftc_readGlobalConfigReg (&pFFTCInstInfo->fftcLldObj, &fftGlobalCfg_r);
        txChCfg.priority        =   fftGlobalCfg_r.queuePriority[fftcTxQNum];

        /* Open a CPPI Tx Channel corresponding to the FFTC queue specified. 
         * In the FFTC H/w:     CPPI Channel 0 mapped to FFTC Tx Queue 0
         *                      CPPI Channel 1 mapped to FFTC Tx Queue 1
         *                      CPPI Channel 2 mapped to FFTC Tx Queue 2
         *                      CPPI Channel 3 mapped to FFTC Tx Queue 3
         */             
        if ((hCppiTxChan = Cppi_txChannelOpen (pFFTCInstInfo->hCppi, &txChCfg, &isAllocated)) == NULL)
        {
            Fftc_osalLog ("Error opening Tx channel corresponding to FFTC queue : %d\n", fftcTxQNum);
            goto return_error;
        }

        /* If the application is requesting a FFTC channel that would be 
         * exclusively used by it, ensure that the channel is not already
         * in use by someone else.
         */
        if ((isAllocated > 1) && !bSharedMode)
        {
            /* The channel we requested is already in use by some other application. 
             * Return error.
             */
            goto return_error;
        }

        /* Open the FFTC Tx queue */
        if ((hQmssTxQ = Qmss_queueOpen ((Qmss_QueueType) QMSS_PARAM_NOT_SPECIFIED, pFFTCInstInfo->baseQueueNum + fftcTxQNum, &isAllocated)) < 0)
        {
            Fftc_osalLog ("Error opening FFTC Tx Queue Number %d \n", fftcTxQNum);
            goto return_error;
        }

        /* If the application is requesting a FFTC channel that would be 
         * exclusively used by it, ensure that the queue corresponding to that
         * channel is not already in use by someone else.
         */
        if ((isAllocated > 1) && !bSharedMode)
        {
            /* The queue we requested is already in use by some other application.
             * Return error.
             */
            goto return_error;
        }    

        /* Setup the FFTC Queue configuration registers for this queue/channel */
        if (Fftc_writeQueueConfigRegs (&pFFTCInstInfo->fftcLldObj, (Fftc_QueueId) fftcTxQNum, pFFTQCfg) != 0)
        {
            Fftc_osalLog ("FFTC Queue configuration register setup failed \n");
            goto return_error;
        }

        /* The Rx channel is mapped one to one with Tx channel. 
         * We need to open and enable this one too.
         */
        rxChInitCfg.channelNum          = fftcTxQNum; 
        rxChInitCfg.rxEnable            = Cppi_ChState_CHANNEL_DISABLE; 
        if ((hCppiRxChan = Cppi_rxChannelOpen (pFFTCInstInfo->hCppi, &rxChInitCfg, &isAllocated)) == NULL)
        {
            Fftc_osalLog ("Error opening Rx channel: %d \n", fftcTxQNum);
            goto return_error;
        }

        /* Finally, enable the Tx channel so that we can start sending 
         * data blocks to FFTC engine.
         */
        Cppi_channelEnable (hCppiTxChan);

        /* Also enable Rx Channel */
        Cppi_channelEnable (hCppiRxChan);

        /* Save the channel setup info */
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].cppiTxQNum          =   fftcTxQNum;
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].bSharedMode         =   bSharedMode;
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].hCppiTxChan         =   hCppiTxChan;
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].hQmssTxQ            =   hQmssTxQ;
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].bSupressSideInfo    =   pFFTQCfg->controlRegConfig.bSupressSideInfo;
        pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].hCppiRxChan         =   hCppiRxChan;
        if (pFFTQCfg->cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
        {
            pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].cyclicPrefixAddNum  =   pFFTQCfg->cyclicPrefixRegConfig.cyclicPrefixAddNum;
        }
        else
        {
            pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].cyclicPrefixAddNum  =   0;
        }
    }
    else if (!pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].bSharedMode)
    {
        /* This queue is already open and is reserved for use
         * by another application to be exclusively used by it.
         * Hence, deny the request for this application to
         * obtain a handle for this queue.
         */
        Fftc_osalLog ("FFTC Queue %d is not a shared mode queue and is already in use. Try another Tx Queue \n", 
                      fftcTxQNum);
        return NULL;
    }

    /* Increment the reference count on this queue */
    pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum].refCnt ++;                    
    
    hFFTCTxQueueInfo = &pFFTCInstInfo->Fftc_txQInfo[fftcTxQNum];

    /* Return success. Return the handle to the Tx queue */
    return hFFTCTxQueueInfo;

return_error:
    if (hCppiTxChan)
    {
        Cppi_channelClose (hCppiTxChan);
    }
    if (hQmssTxQ)
    {
        Qmss_queueClose (hQmssTxQ);       
    }
    if (hCppiRxChan)
    {
        Cppi_channelClose (hCppiRxChan);
    }

    /* Return NULL handle to indicate that the setup failed */
    return NULL;
}

/**
* ============================================================================
*  @n@b Fftc_txQueueClose
*
*  @b   brief
*  @n   This API decrements the reference count on the transmit queue handle
*       provided and when the reference count reaches zero, i.e., when no
*       application thread/flow is no longer using this transmit queue handle it
*       frees up the FFTC transmit queue handle and all associated configuration.
*       If an application wishes to use the transmit queue, it would have to do
*       so by opening the FFTC transmit queue with appropriate configuration
*       using the API @a Fftc_txQueueOpen ().
*
*  @param[in]
*       pFFTCTxQInfo        Handle to the transmit queue obtained using
*                           @a  Fftc_txQueueOpen () API.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Invalid transmit queue handle provided.
*  @li         FFTC_RETVAL_SUCCESS      -   Succesfully closed the queue.
*
*  @pre
*  @n   @a Fftc_txQueueOpen () must have been called succesfully to setup the 
*       FFTC transmit queue before calling this API.
*
*  @post
*  @n   Reference counter on the queue handle decremented. If reference counter
*       reaches zero, queue configuration cleaned up and all associated CPPI
*       handles released.
* ============================================================================
*/
static Fftc_RetVal Fftc_txQueueClose 
(
    Fftc_TxQInfo*               pFFTCTxQInfo
)
{
    /* Validate input */
    if (!pFFTCTxQInfo)
    {
        Fftc_osalLog ("Invalid Transmit queue handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;	
    }

    if (!pFFTCTxQInfo->refCnt)
    {
        return FFTC_RETVAL_SUCCESS;	
    }

    /* We can close the queue and clean up all its associated CPPI
     * handles only if there is no other application/driver using
     * this queue other than us, i.e. refCnt is 1.
     */
    if (pFFTCTxQInfo->refCnt == 1)
    {
        /* Disable the Tx channel */            
        Cppi_channelDisable (pFFTCTxQInfo->hCppiTxChan);

        /* Disable the Rx channel */            
        Cppi_channelDisable (pFFTCTxQInfo->hCppiRxChan);

        /* Close the CPPI/QMSS channels and queues associated with 
         * the transmit queue specified.
         */
        Cppi_channelClose (pFFTCTxQInfo->hCppiTxChan);
        Cppi_channelClose (pFFTCTxQInfo->hCppiRxChan);
        Qmss_queueClose (pFFTCTxQInfo->hQmssTxQ);
    
        /* Update our database of tx queue state info */
        pFFTCTxQInfo->cppiTxQNum            =   0;
        pFFTCTxQInfo->bSharedMode           =   0;
        pFFTCTxQInfo->hCppiTxChan           =   NULL;
        pFFTCTxQInfo->hCppiRxChan           =   NULL;
        pFFTCTxQInfo->hQmssTxQ              =   (Qmss_QueueHnd) NULL;
        pFFTCTxQInfo->bSupressSideInfo      =   0;
        pFFTCTxQInfo->cyclicPrefixAddNum    =   0;
    }

    /* Decrement the reference count to reflect the successful
     * close.
     */
    pFFTCTxQInfo->refCnt --;

    /* Return Success */
    return FFTC_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Fftc_isValidFlow
 *
 *  @b  brief
 *  @n  Given a FFTC driver handle and a flow id, this API searches through
 *      the driver's flow database to check if the flow Id specified is 
 *      valid and active. 
 *
 *      This API is used internally by the driver at Rx object open time in 
 *      @a Fftc_rxOpen () API to ensure that flow requested is valid.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]    
 *      hFFTC               FFTC driver handle.
 *
 *  @param[in]    
 *      flowId              Flow Id that needs to be validated

 *  @return     uint8_t
 *  @li         0   -   No flow exists with the flow Id specified for the FFTC 
 *                      instance the driver handle uses.
 *                                  
 *  @li         1   -   Valid flow configuration. Found an active flow matching
 *                      the flow Id specified.
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to initialize the driver before 
 *      calling this API.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        ...

        // Validate the flow
        if (!Fftc_isValidFlow (hFFTC, 0))
        {
            // Error. Invalid flow specified.
        }        
    @endcode
 * ============================================================================
 */
static uint8_t Fftc_isValidFlow 
(
    Fftc_DrvHandle          hFFTC,
    int8_t                  flowId
)
{
    int32_t                 i, instNum, bMatchFound = 0;

    instNum =	((Fftc_UserInfo *)hFFTC)->instNum;

    /* Check if the flow Id specified is in valid range  */
    if (flowId < 0 || flowId >= FFTC_MAX_NUM_FLOWS)
    {
        return 0;
    }

    /* Search through the flow database of the FFTC instance
     * corresponding to the driver handle.
     */
    for (i = 0; i < FFTC_MAX_NUM_FLOWS; i++)
    {
        if (Fftc_instanceInfo[instNum].Fftc_flowInfo[i].refCnt)
        {
            /* Flow specified is active. */
            bMatchFound = 1;
            break;
        }
    }

    if (bMatchFound)
    {
        /* Flow specified is valid */            
        return 1;
    }
    else
    {
        /* Invalid flow Id. No flow active with flow id specified. */            
        return 0;                
    }
}

/**
 * ============================================================================
 *  @n@b Fftc_getNextAvailRxObjId
 *
 *  @b  brief
 *  @n  Given an FFTC peripheral instance number, this API searches through
 *      the instance's global Rx object info database to find the next 
 *      available empty slot to store a new Rx object's info. Each slot in this 
 *      global Rx object info database stores Rx object information that needs
 *      to be globally accessible, i.e., accessible from all cores of the 
 *      device.
 *
 *      * API for internal use by the driver *
 *
 *  @param[in]    
        instNum             FFTC instance number from which an empty Rx object
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
 *  @n  @a Fftc_open () must have been called to initialize the driver before 
 *      calling this API. Also a valid instance number must be specified to 
 *      this API in 'instNum' parameter.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        ...

        // Get the next available Rx object slot from the database.
        if (Fftc_getNextAvailRxObjId (CSL_FFTC_A) < 0)
        {
            // Number of Rx objects created exceeds maximum allowed.
        }        
    @endcode
 * ============================================================================
 */
static int32_t Fftc_getNextAvailRxObjId 
(
    uint8_t                   instNum
)
{
    int32_t                   i;

    for (i = 0; i < FFTC_MAX_NUM_RXOBJECTS; i++)
    {
        if (!Fftc_instanceInfo[instNum].Fftc_rxObjGlobalInfo[i].bIsValid)
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
    return FFTC_RETVAL_EFAILURE;
}

/***************************************************************
 ******************* FFTC Exported APIs ************************
 **************************************************************/

/**
 * ============================================================================
 *  @n@b Fftc_getVersionID
 *
 *  @b  brief
 *  @n  This API returns the FFTC driver's version Id.
 *      
 *  @return     
 *  @n  uint32_t    -   Returns the FFTC driver version Id
 *
 *  @pre
 *  @n  None. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
uint32_t Fftc_getVersionID 
( 
    void
)
{
    return FFTC_VERSION_ID;
}


const char  Fftc_versionStr[] = FFTC_VERSION_STR ":" __DATE__  ":" __TIME__;

/**
 * ============================================================================
 *  @n@b Fftc_getVersionStr
 *
 *  @b  brief
 *  @n  This API returns the FFTC driver's version information in string format.
 *      
 *  @return     
 *  @n  const char*     -   Returns the FFTC driver build and version info in 
 *                          string format.
 *
 *  @pre
 *  @n  None. 
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
const char* Fftc_getVersionStr 
(
    void
)
{
    return (const char *) (Fftc_versionStr);
}

/**
 * ============================================================================
 *  @n@b Fftc_init
 *
 *  @b  brief
 *  @n  This API initializes the FFTC driver state for the instance number 
 *      provided in 'instNum' input parameter. As part of initialization, this
 *      API opens the FFTC CPDMA, opens the FFTC LLD, initializes the FFTC
 *      MMR with parameters provided in 'pFFTCGlobalCfg' and resets the driver's 
 *      internal global state and all associated handles.
 *
 *      This API MUST be called ONLY once in the system and at system startup
 *      for any given FFTC peripheral instance before using any of the FFTC 
 *      driver APIs.
 *
 *  @param[in]    
 *      instNum             FFTC peripheral instance number.
 *
 *  @param[in]    
 *      pFFTCGlobalCfg      Handle to FFTC Global Configuration structure. 
 *
 *  @param[in]    
 *      pFFTCDevCfg         Handle to device specific configuration info.
 *
 *                           
 *  @return     Fftc_RetVal
 *  @li         FFTC_RETVAL_EINVALID_PARAMS -   Invalid instance number provided.
 *  @li         FFTC_RETVAL_EBAD_HANDLE     -   Invalid global configuration
 *                                              handle provided.
 *  @li         FFTC_RETVAL_EFAILURE        -   Error initializing driver for the
 *                                              FFTC instance requested.
 *  @li         FFTC_RETVAL_SUCCESS         -   FFTC instance initialized succesfully.                                              
 *
 *  @pre
 *  @n  This API MUST be called only once during system startup. 
 *
 *  @post
 *  @n  FFTC driver initialized for the instance specified.
 * ============================================================================
 */
Fftc_RetVal Fftc_init 
(
    uint8_t                     instNum, 
    Fftc_GlobalCfg*             pFFTCGlobalCfg,
    Fftc_DeviceCfg*             pFFTCDevCfg
)
{
    Cppi_Handle                 hCppi;
    Cppi_CpDmaInitCfg           cpdmaCfg;
	
    /* Validate user input */
    if (instNum >= FFTC_MAX_NUM_INSTANCES)
    {
        Fftc_osalLog ("Invalid instance number! Valid range 0 - %d \n", (FFTC_MAX_NUM_INSTANCES - 1));
        return FFTC_RETVAL_EINVALID_PARAMS;
    }

    if (!pFFTCGlobalCfg || !pFFTCDevCfg || !pFFTCDevCfg->cfgRegs)
    {
        Fftc_osalLog ("Invalid FFTC Global/Device configuration passed \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();

    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));    

    /* Reset the driver's global info for this instance */            
    memset (&Fftc_instanceInfo [instNum], 0, sizeof (Fftc_InstanceInfoObj));
    
    /* Setup the driver for this FFTC peripheral instance number. */

    /* Set up the FFTC CPDMA configuration */
    memset (&cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum = pFFTCDevCfg->cpdmaNum;

    /* Initialize FFTC CPDMA */
    if ((hCppi = Cppi_open (&cpdmaCfg)) == NULL)
    {
        Fftc_osalLog ("Error initializing CPPI for FFTC CPDMA %d\n", cpdmaCfg.dmaNum);

        /* Exit Critical Section */
        Fftc_osalMultiCoreCsExit ();
        return FFTC_RETVAL_EFAILURE;
    }

    /* Disable FFTC CDMA loopback */
    if (Cppi_setCpdmaLoopback (hCppi, 0) != CPPI_SOK)
    {
        Fftc_osalLog ("Error disabling loopback for FFTC CPDMA %d\n", cpdmaCfg.dmaNum);

        /* Exit Critical Section */
        Fftc_osalMultiCoreCsExit ();
        return FFTC_RETVAL_EFAILURE;
    }

    /* Open the FFTC LLD for this instance. 
     *
     * Required to setup/read the FFTC MMR for this instance.
     */
    if (Fftc_lldOpen (instNum, pFFTCDevCfg->cfgRegs, &Fftc_instanceInfo[instNum].fftcLldObj) != 0)
    {
        /* FFTC LLD Instance open failed */            
        Fftc_osalLog ("FFTC LLD Instance %d open failed \n", instNum);

        /* Exit Critical Section */
        Fftc_osalMultiCoreCsExit ();
        return FFTC_RETVAL_EFAILURE;
    }

    /* Configure the FFTC Global Configuration Register as per the inputs passed. */
    if (Fftc_writeGlobalConfigReg (&Fftc_instanceInfo[instNum].fftcLldObj, pFFTCGlobalCfg) != 0)
    {
        Fftc_osalLog ("Error configuring FFTC Configuration Register \n");

        /* Exit Critical Section */
        Fftc_osalMultiCoreCsExit ();
        return FFTC_RETVAL_EFAILURE;
    }

    /* Save the state info for this instance. */
    Fftc_instanceInfo[instNum].instNum              =   instNum;
    Fftc_instanceInfo[instNum].hCppi                =   hCppi;
    Fftc_instanceInfo[instNum].bIsDftSizeListInUse  =   0;
    Fftc_instanceInfo[instNum].cpdmaNum             =   pFFTCDevCfg->cpdmaNum;
    Fftc_instanceInfo[instNum].baseQueueNum         =   pFFTCDevCfg->baseQueueNum;

    /* Increment the reference count to indicate that this instance
     * has been initialized and is ready for use by an application.
     */
    Fftc_instanceInfo[instNum].refCnt ++;

    /* Writeback the shared Instance Info object */
    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Exit Critical Section */
    Fftc_osalMultiCoreCsExit ();

    /* Successfully done with initialization. Return. */
    return FFTC_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Fftc_deInit
 *
 *  @b  brief
 *  @n  This API closes the FFTC peripheral instance opened earlier using 
 *      @a Fftc_init () API. When all the applications using this FFTC
 *      instance have released it (reference count on the instance reaches zero),
 *      this API closes the FFTC CPDMA, closes the FFTC LLD and finally initiates 
 *      a software reset on the FFTC engine to reset all its registers, and 
 *      it's state machine. On success, it restores the FFTC peripheral state 
 *      to 'unopened' state
 *
 *      If an application or driver would like to use any of the driver APIs again,
 *      it can do so only after calling @a Fftc_init() again.
 *
 *  @param[in]
 *       instNum             FFTC peripheral instance number.
 *
 *  @return     Fftc_RetVal
 *  @li         FFTC_RETVAL_SUCCESS     -   Successfully closed the FFTC.
 *
 *  @pre
 *  @n  @a Fftc_init () must be called before calling this API. 
 *
 *  @post
 *  @n  Reference count decremented on the FFTC instance in the driver. When
 *      reference count reaches zero, CPDMA is closed, all FFTC Tx queues closed 
 *      and deallocated. FFTC engine software reset performed.
 * ============================================================================
 */
Fftc_RetVal Fftc_deInit 
(
    uint8_t                     instNum
)
{
    /* FFTC setup cannot be undone unless initialized */
    if (!Fftc_isInitialized (instNum))
    {
        Fftc_osalLog ("FFTC instance %d not open !! \n", instNum);
        return FFTC_RETVAL_SUCCESS;
    }

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();

    /* Ensure all applications using this FFTC instance are done
     * using before resetting the instance.
     */
    if (Fftc_instanceInfo[instNum].refCnt == 1)
    {
        /* Last application using the FFTC instance is 
         * closing. Clean up all associated handles.
         */
        /* Close the CPDMA */
        Cppi_close (Fftc_instanceInfo[instNum].hCppi);

        /* Issue a driver reset to FFTC engine to reset all its
         * register values and state machines to its initial state.
         */
        Fftc_doSoftwareReset (&Fftc_instanceInfo[instNum].fftcLldObj);

        Fftc_lldClose (&Fftc_instanceInfo[instNum].fftcLldObj);

        /* FFTC close successful. Update the status variables. */
        Fftc_instanceInfo[instNum].instNum              =   0;
        Fftc_instanceInfo[instNum].refCnt               =   0;
        Fftc_instanceInfo[instNum].hCppi                =   NULL;
        Fftc_instanceInfo[instNum].bIsDftSizeListInUse  =   0;
        Fftc_instanceInfo[instNum].baseQueueNum         =   0;
    }

    /* De-init complete. Write back the cached copy */
    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Exit Critical Section */
    Fftc_osalMultiCoreCsExit ();
    
    /* Return success. */
    return FFTC_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Fftc_isInitialized
 *
 *  @b  brief
 *  @n  Given an FFTC peripheral instance number, this API returns 1 to
 *      indicate that it is open and has been successfully setup, 0 otherwise.
 *
 *  @param[in]
 *      instNum             FFTC peripheral instance number to check the status 
 *                          on.
 *
 *  @return     uint8_t
 *  @li         1       -   Driver setup and opened for the instance number passed.
 *  @li         0       -   Instance number invalid/not initialized.
 *
 *  @pre
 *  @n  None.
 *
 *  @post
 *  @n  None.
 * ============================================================================
 */
uint8_t Fftc_isInitialized 
(
    uint8_t                     instNum
)
{
    uint8_t                     retVal;

    /* Critical section start */
    Fftc_osalMultiCoreCsEnter ();

    /* Invalidate cache before reading shared Instance Info to
     * ensure we are in sync.
     */
    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Validate input */
    if (instNum >= FFTC_MAX_NUM_INSTANCES || !Fftc_instanceInfo[instNum].refCnt)
    {
        /* critical section end */
        Fftc_osalMultiCoreCsExit ();
        return 0;
    }

    /* check if the FFTC instance is setup and valid */
    if (Fftc_instanceInfo[instNum].refCnt)
    {
        /* FFTC instance open */            
        retVal = 1;
    }
    else
    {
        /* FFTC instance not open */            
        retVal = 0;
    }    

    /* critical section end */
    Fftc_osalMultiCoreCsExit ();

    return retVal;
}

/**
 * ============================================================================
 *  @n@b Fftc_open
 *
 *  @b  brief
 *  @n  This API opens the FFTC driver and sets up a pool of free descriptor 
 *      queues for Rx, and Tx for this application as per the configuration 
 *      specified in 'pFFTCDrvCfg'. The descriptors setup by this API are
 *      later used by the driver in setting up free descriptor queues for 
 *      various Tx, Rx objects.
 *
 *      All the Tx, Rx descriptor configurations provided to this API in
 *      'pFFTCDrvCfg->cppiFreeDescCfg' MUST be ordered in the ascending order
 *      of the descriptor sizes. 
 *
 *      This API MUST be called by all FFTC driver users at least once to obtain
 *      an FFTC driver handle and setup all Tx, Rx descriptors required by it
 *      for FFT processing. 
 *
 *  @param[in]    
 *      instNum             FFTC Instance number that the application would like
 *                          to use.
 *
 *  @param[in]    
 *      pFFTCDrvCfg         Handle to FFTC driver configuration structure. Must
 *                          contain descriptor configuration relevant for this 
 *                          application.
 *
 *  @param[out]    
 *      pRetVal             Pointer to hold the return value returned by this API. 
 *                          Following are the possible return values for this API:
 *                            
 *  @li         FFTC_RETVAL_EINVALID_PARAMS -   Invalid instance number/descriptor 
 *                                              configuration provided.
 *  @li         FFTC_RETVAL_EBAD_HANDLE     -   Invalid driver configuration
 *                                              handle provided.
 *  @li         FFTC_RETVAL_ENO_MEM         -   Error allocating memory for driver's 
 *                                              internal book-keeping data.
 *  @li         FFTC_RETVAL_SUCCESS         -   Successfully setup the descriptors.
 *
 *  @return     Fftc_DrvHandle
 *  @li         NULL                        -   Error. Appropriate error code is returned
 *                                              in the output parameter 'pRetVal'.
 *  @li         >0                          -   Success. Valid FFTC driver handle is returned.
 *
 *  @pre
 *  @n  The 'pFFTCDrvCfg' driver configuration pointer MUST contain valid CPPI 
 *      descriptor configuration consistent with the system setup.
 *
 *  @post
 *  @n  Application requested descriptors setup. 
 * ============================================================================
 */
Fftc_DrvHandle Fftc_open 
(
    uint8_t                     instNum,
    Fftc_DrvCfg*                pFFTCDrvCfg,
    Fftc_RetVal*                pRetVal
)
{
    Fftc_UserInfo*              pFFTCDrvUserInfo;
    Qmss_QueueHnd               hQmssFreeQ;
    uint16_t                    i;

    /* Validate input */
    if (!pRetVal)
    {
        Fftc_osalLog ("Invalid Retrun value handle passed \n");
        return NULL;
    }

    /* Validate the instance number passed. Instance number passed is invalid if:
     *
     *  (1) Instance number exceeds the acceptable range.
     *  (2) Instance number is valid, but the driver's state info indicates that
     *      the instance has not been setup using Fftc_init () API.
     */
    if (!Fftc_isInitialized (instNum))
    {
        Fftc_osalLog ("Invalid instance number or instance not initialized yet. \n");
        *pRetVal    =   FFTC_RETVAL_EINVALID_PARAMS;
        return NULL;
    }
	    
    if (!pFFTCDrvCfg)
    {
        Fftc_osalLog ("Invalid FFTC Driver Configuration handle passed \n");
        *pRetVal    =   FFTC_RETVAL_EBAD_HANDLE;
        return NULL;
    }

    /* Allocate space to hold FFTC descriptor configuration for this user */
    if ((pFFTCDrvUserInfo = (Fftc_UserInfo *) Fftc_osalMalloc (sizeof (Fftc_UserInfo), FALSE)) == NULL)
    {
        Fftc_osalLog ("Error allocating memory for FFTC driver info! \n");
        *pRetVal    =   FFTC_RETVAL_ENO_MEM;
        return NULL;
    }
    memset ((void *)pFFTCDrvUserInfo, 0, sizeof (Fftc_UserInfo));

    /* Save driver settings */
    pFFTCDrvUserInfo->instNum                   =   instNum;   
    pFFTCDrvUserInfo->drvCfg                    =   *pFFTCDrvCfg;

    /* Setup the free queues and their descriptors as per the params specified. */
    for (i = 0; i < pFFTCDrvCfg->cppiNumFreeDescCfg; i++)
    {
        if ((hQmssFreeQ = Fftc_setupCppiDesc (&pFFTCDrvCfg->cppiFreeDescCfg[i])) == (Qmss_QueueHnd) NULL)
        {
            Fftc_osalLog ("Error setting up Free queue as per the Descriptor Config %d \n",  
                            pFFTCDrvCfg->cppiNumFreeDescCfg);
		    *pRetVal	=	FFTC_RETVAL_EINVALID_PARAMS;
            Fftc_osalFree ((void *)pFFTCDrvUserInfo, sizeof (Fftc_UserInfo), FALSE);
            return NULL;
	    }            

        /* Save the free queue configuration, handle and increment the allocated free queue count. */
        pFFTCDrvUserInfo->hQmssFreeDescQ[i]         =   hQmssFreeQ;
        pFFTCDrvUserInfo->drvCfg.cppiFreeDescCfg[i] =   pFFTCDrvCfg->cppiFreeDescCfg[i];
    }

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();

    /* Increment reference count to keep the instance info alive */
    Fftc_instanceInfo[instNum].refCnt ++;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Exit Critical Section */
    Fftc_osalMultiCoreCsExit ();

    /* Return success */
    *pRetVal    =   FFTC_RETVAL_SUCCESS;

    /* Return handle to the FFTC instance info */ 
    return ((Fftc_DrvHandle) pFFTCDrvUserInfo);
}

/**
 * ============================================================================
 *  @n@b Fftc_close
 *
 *  @b  brief
 *  @n  This API closes the FFTC driver instance opened earlier using a call
 *      to @a Fftc_open () API. 
 *
 *      Currently, there exists no way to cleanly free the descriptors obtained 
 *      during Fftc_open () API. 
 *
 *  @param[in]
 *      hFFTC               FFTC driver handle obtained using @a Fftc_open () 
 *                          API.
 *
 *  @return     Fftc_RetVal
 *  @li         FFTC_RETVAL_EBAD_HANDLE -   Invalid input handle passed. Error.
 *  @li         FFTC_RETVAL_SUCCESS     -   Successfully closed the FFTC.
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to open the driver instance prior to
 *      calling this API. Also, all open Tx, Rx objects opened by an application
 *      must be closed by calling @a Fftc_txClose () and Fftc_rxClose () APIs 
 *      before calling this API to ensure all associated memory is freed cleanly.
 *
 *  @post
 *  @n  Reference count decremented on the FFTC instance in the driver. 
 * ============================================================================
 */
Fftc_RetVal Fftc_close 
(
    Fftc_DrvHandle          hFFTC
)
{
    Fftc_UserInfo*          pFFTCUserInfo;

    /* Validate input */
    if (!hFFTC)
    {
        Fftc_osalLog ("Invalid FFTC instance handle passed \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }

    pFFTCUserInfo   =   (Fftc_UserInfo *) hFFTC;

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();

    /* Invalidate the FFTC instance in cache */
    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));
    Fftc_instanceInfo[pFFTCUserInfo->instNum].refCnt --;            
    Fftc_osalEndMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));

    /* Exit Critical Section */
    Fftc_osalMultiCoreCsExit ();
    
    /* Free the FFTC state info handle */
    Fftc_osalFree ((void*) pFFTCUserInfo, sizeof (Fftc_UserInfo), FALSE);

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;
}

/**
 * ============================================================================
 *  @n@b Fftc_getLLDObject
 *
 *  @b  brief
 *  @n  This API returns the FFTC LLD object handle corresponding to the driver
 *      handle provided to this API. 
 *
 *  @param[in]
 *      hFFTC               FFTC driver handle obtained using @a Fftc_open () 
 *                          API.
 *
 *  @return     Fftc_LldObj*
 *  @li         NULL        -   Invalid input handle passed. Error.
 *  @li         >0          -   FFTC LLD object handle returned.
 *
 *  @pre
 *  @n  @a Fftc_open () must have been called to open the driver instance prior to
 *      calling this API. 
 *
 *  @post
 *  @n  None
 * ============================================================================
 */
Fftc_LldObj* Fftc_getLLDObject 
(
    Fftc_DrvHandle          hFFTC
)
{
    Fftc_UserInfo*          pFFTCUserInfo;
    Fftc_LldObj*            pFftcLldObj = NULL;

    /* Validate input */
    if (!hFFTC)
    {
        Fftc_osalLog ("Invalid FFTC instance handle passed \n");
        return NULL;
    }

    pFFTCUserInfo   =   (Fftc_UserInfo *) hFFTC;

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();

    /* Invalidate the FFTC instance in cache */
    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));
    pFftcLldObj =   &Fftc_instanceInfo[pFFTCUserInfo->instNum].fftcLldObj;            

    /* Exit Critical Section */
    Fftc_osalMultiCoreCsExit ();
    
    /* Return LLD handle. */
    return pFftcLldObj;
}

/**
* ============================================================================
*  @n@b Fftc_txOpen
*
*  @b   brief
*  @n   This API sets up a transmit object for the calling application. A 
*       transmit object must be setup by any application that wishes to
*       submit FFT requests using the driver. 
*
*       Given an FFTC Transmit queue number using which the FFT requests must be
*       submitted, and the Tx descriptor configuration to use for submitting 
*       the requests, this API opens the FFTC transmit queue if not already open 
*       and initializes it with the queue configuration provided, sets up the Tx 
*       free descriptors and buffers if requested and returns a Tx object handle 
*       that can be used by the application to submit FFT requests.
*
*       The Tx object configuration MUST be provided in the input parameter 
*       'pFFTCTxCfg'. If the field 'bManageReqBuffers' of this structure is set
*       to 1, this API pre-allocates (Tx) request buffers and descriptors according 
*       to the parameters 'descType', 'cppiNumDesc', 'bufferSize' and holds enough 
*       space for specifying 128 DFT block sizes. It also reserves space for DFT 
*       size list configuration based on parameters 'bEnableDftSizeListCfg' and
*       'maxDftSizeListLen'.
*
*       The pre-allocated request buffers can then be obtained by application using
*       @a Fftc_txGetRequestBuffer () API, and can be used to hold the 
*       FFT/IFFT request data block. This eliminates the need for the application
*       to do separate buffer allocations on data path and the buffer management
*       is then entirely done by the FFTC driver. 
*
*       This API opens the FFTC Tx queue and initializes it with the configuration
*       provided in the input parameter when the API is called for the first time
*       for a given FFTC transmit queue number. On all future calls the queue 
*       configuration is omitted.
*
*       This API ensures that no two Tx objects for any given FFTC instance use
*       mixed size DFT configuration at the same time. 
*
*       On success, this API returns a valid, non-zero Tx handle and NULL 
*       otherwise to indicate an error.
*
*       Every application that wishes to submit FFT requests is expected to
*       call this API to obtain a Tx object handle.
*
*  @param[in]
*       hFFTC               FFTC driver handle obtained using @a Fftc_open () 
*                           API.
*
*  @param[in]  
*       pFFTCTxCfg          Input structure that holds configuration for the
*                           Tx object.
*
*  @return      Fftc_TxHandle
*  @li          NULL                -   Error. Invalid Tx object configuration. 
*  @li          Valid Tx handle     -   Success. 
*
*  @pre
*  @n   Valid FFTC driver handle must be obtained by calling @a Fftc_open () API
*       by the application before calling this API. Valid Tx configuration must be 
*       specified in 'pFFTCTxCfg' structure as input to this API. 
*
*  @post
*  @n   FFTC Tx queue open and setup if this API being called for first time for
*       a given Tx queue number. Tx descriptors, buffers allocated as per input
*       specified. Tx object is ready for use to submit FFT requests to the engine.
* ============================================================================
*/
Fftc_TxHandle Fftc_txOpen
(
    Fftc_DrvHandle              hFFTC,
    Fftc_TxCfg*                 pFFTCTxCfg
)
{
    int32_t                     i;                
    Fftc_UserInfo*              pFFTCUserInfo;
    Qmss_QueueHnd               hQmssTxFreeQ;
    Fftc_TxQInfo*               pFFTCTxQInfo;
    uint32_t                    txAddnlLen = 0;
    Qmss_QueueHnd               hTxGlobalFreeQ;
    Fftc_TxInfo*                pTxObjInfo;
    uint32_t                    descSize;
    uint8_t                     instNum;

    /* Validate application's FFTC driver handle. */
    if (!hFFTC)
    {
        Fftc_osalLog ("Invalid FFTC driver handle passed !! \n");
        return NULL;
    }

    /* Validate input configuration handle */
    if (!pFFTCTxCfg)
    {
        Fftc_osalLog ("Invalid Tx configuration handle passed \n");            
        return NULL;
    }

    pFFTCUserInfo   =   (Fftc_UserInfo *) hFFTC;
    instNum         =   pFFTCUserInfo->instNum;

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();            

    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Validate that no 2 Tx objects are using DFT size list configuration at same time */
    if (pFFTCTxCfg->bEnableDftSizeListCfg && Fftc_instanceInfo[instNum].bIsDftSizeListInUse)
    {
        Fftc_osalLog ("Error! DFT size list in use by another Tx object. \n");            
        Fftc_osalMultiCoreCsExit ();            
        return NULL;
    }
    
    /* Validate DFT size list configuration input */
    if (pFFTCTxCfg->maxDftSizeListLen > FFTC_MAX_NUM_BLOCKS)
    {
        Fftc_osalLog ("Invalid DFT size list length specified. Valid Values range between 0-128 \n");
        Fftc_osalMultiCoreCsExit ();            
        return NULL;
    }

    /* Open the Tx queue */
    if ((pFFTCTxQInfo    = (Fftc_TxQInfo *) Fftc_txQueueOpen (&Fftc_instanceInfo[instNum],
                                                              pFFTCTxCfg->txQNum,
                                                              &pFFTCTxCfg->fftQCfg,
                                                              pFFTCTxCfg->bSharedMode)) == NULL) 
    {
        Fftc_osalLog ("Invalid FFTC Transmit configuration specified \n");
        Fftc_osalMultiCoreCsExit ();
        return NULL;
    }

   /* Pre-allocate buffers for Tx free queue. 
    * 
    * This flag is to support some use cases where FFTC doesnt need to
    * allocate a Tx free queue for a flow. This is especially the case
    * when a DSP application is not the one that is going to be using
    * this Tx object to submit FFT requests.
    *
    * For example, when data is being sent from AIF --> FFTC, the AIF uses 
    * its Rx Free queue to get a Tx Free descriptor to transmit data blocks 
    * to FFTC instead of setting up a new one.
    */
    if (pFFTCTxCfg->bManageReqBuffers)
    {
       /* Allocate buffers and initialize the Tx Buffer Descriptors on the free queue 
        * local to this flow.
        * 
        * This way, the application during transmit can just get the buffer directly from the 
        * descriptor and hence can avoid cycle intensive memory allocations on the data path.
        */            
       /* Check if the Tx Queue specified is "shared" , i.e. the queue configuration can
        * be changed on the fly using CPPI packets or if its a "reserved" resource, i.e., 
        * once the transmit queue is opened and configured, it will not be re-configured
        * using CPPI packets on data path.
        *
        * If the Tx queue is shared, we need to allocate space for:
        *      FFTC Queue Specific Register Configurations     =   5 * 4 bytes
        *
        * The data being sent using this flow can have multiple data blocks of varying sizes 
        * (mixed transform sizes) and the maximum number of such blocks that will be sent out 
        * using any flow in each of the CPPI packet can be 128. Hence reserve space enough
        * to hold 128 DFT block sizes.
        *      size to reserve for DFT size list configuration =   (DFT size list len / 5 + 1) * 4
        *      (the maximum DFT block size that can be specified is 8192bytes, that can be
        *      accomodated in 2 bytes. The DFT sizes are specified to H/W in blocks of 5, hence
        *      to specify 128 block sizes it would require 128 / 5 + 1 = 25 + 1 DFT size list 
        *      groups each 4 bytes long).
        *
        * If either of the above 2 configurations are to be added, we need to also hold space     
        * for:
        *      FFTC Control Header                             =   4 bytes
        *
        * Finally, check if the flow is configured to carry "Protocol Specific Information".
        * If so, FFTC can at most carry 4 32-bit words of Pass through Protocol Specific
        * Info. Hence, if flow's protocol specific info is enabled, reserve the space for it.
        *      maximum size of Pass through PS Info using FFTC =   4 * 4 bytes
        */
        if (pFFTCTxCfg->bSharedMode)
        {
           /* Add space for the 5 FFTC queue specific register configuration 
            * addition in the descriptor.
            */
            txAddnlLen        +=   (5 * 4);                
        }

       /* Account for any DFT size list configuration */
       /* DFT size list configuration will be added on this Tx object. 
        * Hold space for the length specified here.
        *
        * Each DFT size group can hold up to 5 DFT sizes. Each group
        * requires 4 bytes to represent it.
        *
        * Space required to hold for DFT size list is number of DFT 
        * groups * 4 bytes.
        */
        if (pFFTCTxCfg->bEnableDftSizeListCfg)
        {
            if (pFFTCTxCfg->maxDftSizeListLen % 5) 
            {
                txAddnlLen        +=  ((pFFTCTxCfg->maxDftSizeListLen / 5 + 1) * 4);                  
            }
            else
            {
                txAddnlLen        +=  ((pFFTCTxCfg->maxDftSizeListLen / 5) * 4);                  
            }
        }

        if (pFFTCTxCfg->bPSInfoPresent)
        {
           /* If there is going to be Pass through Protocol Specific
            * data expected on Rx side, we will have to allocate 
            * space for it too in the Tx Desc so that FFTC engine
            * can copy it over from the Tx buffer to Rx buffer once
            * the DFT transformation is complete.
            *
            * The maximum amount of PS info that can be forwarded using
            * FFTC is 4 32-bit words = 4 * 4 bytes
            */
            txAddnlLen        +=  (FFTC_MAX_NUM_PS_WORDS * 4);                  
        }

        if (txAddnlLen > 0)
        {
           /* If either Tx Queue configuration is to be added / DFT size
            * list configuration is to be added/PS info to be sent, 
            * allocate space for FFTC Control header (4 bytes)
            */
            txAddnlLen        +=  4;
        }

        pFFTCTxCfg->bufferSize  +=  txAddnlLen;

        /* Calculate the descriptor size */
        if (pFFTCTxCfg->descType == Cppi_DescType_HOST)
        {
            /* On Tx side, the PS Info is always added to SOP buffer.
             * No EPIB/PS Info in descriptor, hence descriptor size is just the 
             * basic size. 
             */
            descSize    =   FFTC_CPPI_HOST_DESC_SIZE;
        }
        else
        {
            /* For monolithic descriptors, the descriptor size will always include
             * the buffer size irrespective of where PS info is present.
             */
            descSize    =   pFFTCTxCfg->bufferSize + FFTC_CPPI_MONOLITHIC_DESC_SIZE;

            /* Descriptor sizes must be 16 byte aligned */
            descSize    +=  (16 - (descSize % 16));
        }            

        /* Create a Tx FDQ */
        if ((hQmssTxFreeQ = Fftc_allocFreeQBuffers (pFFTCUserInfo, 
                                                    pFFTCTxCfg->cppiNumDesc, 
                                                    pFFTCTxCfg->bufferSize, 
                                                    pFFTCTxCfg->descType, 
                                                    descSize,
                                                    &i, 
                                                    &hTxGlobalFreeQ)
                                                    ) == (Qmss_QueueHnd) NULL)
        {
            Fftc_osalLog ("No Tx Free Descriptors allocated for the Tx object! \n");

            Fftc_txQueueClose (pFFTCTxQInfo);
            Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));
            Fftc_osalMultiCoreCsExit ();
            return NULL;
        }
    
        /* Check if expected number of descriptors were populated correctly with buffers. */
        if (i != pFFTCTxCfg->cppiNumDesc)
        {
            Fftc_osalLog ("Error attaching %d buffers of type %d descriptors on Tx Free Queue, numAllocated: %d \n", 
                            pFFTCTxCfg->cppiNumDesc, pFFTCTxCfg->descType, i);

            /* Lets clean up the buffers we allocated and return error */
            Fftc_cleanFreeQBuffers (hQmssTxFreeQ, 
                                    hTxGlobalFreeQ, 
                                    i, 
                                    pFFTCTxCfg->bufferSize, 
                                    descSize);

            Fftc_txQueueClose (pFFTCTxQInfo);
            Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));
            Fftc_osalMultiCoreCsExit ();
            return NULL;
        }
    }
    else
    {
        /* Tx descriptors not managed by driver. */            
        descSize    =   0;
    }

    /* Allocate memory for holding the Tx object info */
    if ((pTxObjInfo = Fftc_osalMalloc (sizeof (Fftc_TxInfo), FALSE)) == NULL)
    {
        Fftc_osalLog ("Error allocating memory for Tx object \n");

        /* Free any Tx Free Q buffers allocated */
        if (pFFTCTxCfg->bManageReqBuffers)
        {            
            Fftc_cleanFreeQBuffers (hQmssTxFreeQ, 
                                    hTxGlobalFreeQ, 
                                    pFFTCTxCfg->cppiNumDesc, 
                                    pFFTCTxCfg->bufferSize, 
                                    descSize);
        }

        Fftc_txQueueClose (pFFTCTxQInfo);
        Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));
        Fftc_osalMultiCoreCsExit ();
        return NULL;
    }

    /* Initialize the Tx handle */
    memset ((void*) pTxObjInfo, 0, sizeof (Fftc_TxInfo));

    /* Save the Tx settings and state information */
    pTxObjInfo->hQmssTxFreeQ            =   hQmssTxFreeQ;
    pTxObjInfo->hQmssTxGlblFreeQ        =   hTxGlobalFreeQ;
    pTxObjInfo->pFFTCTxQInfo            =   pFFTCTxQInfo;
    pTxObjInfo->hFFTC                   =   hFFTC;
    pTxObjInfo->bManageReqBuffers       =   pFFTCTxCfg->bManageReqBuffers;
    pTxObjInfo->cppiNumDesc             =   pFFTCTxCfg->cppiNumDesc;
    pTxObjInfo->descSize                =   descSize;
    pTxObjInfo->bufferSize              =   pFFTCTxCfg->bufferSize;
    pTxObjInfo->bEnableDftSizeListCfg   =   pFFTCTxCfg->bEnableDftSizeListCfg;
    pTxObjInfo->maxDftSizeListLen       =   pFFTCTxCfg->maxDftSizeListLen;
    pTxObjInfo->hQmssTxQ                =   pFFTCTxQInfo->hQmssTxQ;
    pTxObjInfo->bSharedMode             =   pFFTCTxQInfo->bSharedMode;

    /* Mark the DFT size list to be "in use" by this Tx object */
    if (pFFTCTxCfg->bEnableDftSizeListCfg)
        Fftc_instanceInfo[pFFTCUserInfo->instNum].bIsDftSizeListInUse = 1;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    Fftc_osalMultiCoreCsExit ();

    /* Return success. Return Tx object handle. */
    return ((Fftc_TxHandle) pTxObjInfo);
}


/**
* ============================================================================
*  @n@b Fftc_txClose
*
*  @b   brief
*  @n   This API closes the Tx object and frees any descriptors and buffers
*       allocated. It also decrements the reference count on the Tx queue this 
*       object uses. When the reference count on the queue reaches zero, the 
*       Tx queue is closed.
*
*  @param[in]
*       hFFTCTxInfo         FFTC Tx handle obtained using @a Fftc_txOpen() API.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Error. Invalid input passed.
*  @li         FFTC_RETVAL_SUCCESS      -   Success. FFTC Tx object closed.
*
*  @pre
*  @n   A valid Tx handle also must be obtained using @a Fftc_txOpen () 
*       before calling this API.
*
*  @post
*  @n   FFTC Tx object successfully closed and any Tx free descriptors, buffers 
*       allocated for the object are cleaned up. The Tx queue is closed if
*       reference count on it reaches zero.
* ============================================================================
*/
Fftc_RetVal Fftc_txClose 
(
    Fftc_TxHandle               hFFTCTxInfo
)
{
    Fftc_TxInfo*                pTxObjInfo;
    Fftc_UserInfo*              pFFTCUserInfo;
    uint8_t                     instNum;

    /* Validate input handles */
    if (!hFFTCTxInfo)
    {
        Fftc_osalLog ("Invalid Transmit Object Handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;	
    }

    pTxObjInfo      =   (Fftc_TxInfo *) hFFTCTxInfo;
    pFFTCUserInfo   =   (Fftc_UserInfo *) pTxObjInfo->hFFTC;
    instNum         =   pFFTCUserInfo->instNum;

    /* Free the pre-allocated request buffers and close the Tx Free Q */            
    if (pTxObjInfo->bManageReqBuffers)
    {
        Fftc_cleanFreeQBuffers (pTxObjInfo->hQmssTxFreeQ, 
                                pTxObjInfo->hQmssTxGlblFreeQ,
                                pTxObjInfo->cppiNumDesc, 
                                pTxObjInfo->bufferSize,
                                pTxObjInfo->descSize);
    }

    /* Grab the driver's lock to access shared info */
    Fftc_osalMultiCoreCsEnter ();

    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Close the Tx queue used by the Tx object */
    Fftc_txQueueClose (pTxObjInfo->pFFTCTxQInfo);

    /* If this flow was using DFT size list, release it */
    if (pTxObjInfo->bEnableDftSizeListCfg)
        Fftc_instanceInfo[instNum].bIsDftSizeListInUse = 0;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Release the driver's lock. */
    Fftc_osalMultiCoreCsExit ();       

    /* Done cleaning up. Free the Tx object handle. */
    Fftc_osalFree (pTxObjInfo, sizeof (Fftc_TxInfo), FALSE);

    /* Return Success */
    return FFTC_RETVAL_SUCCESS;
}


/**
* ============================================================================
*  @n@b Fftc_rxOpen
*
*  @b   brief
*  @n   This API sets up a Rx object for the calling application. A Rx object
*       is required by driver users to retreive FFT results from the 
*       engine using the driver APIs.
*
*       The Rx object configuration MUST be provided in the input parameter 
*       'pFFTCRxCfg'. 
*
*       This API creates a new Rx flow and sets up a Rx Free descriptor queue (FDQ) 
*       if requested, i.e., if 'useFlowId' parameter is set to -1 and driver is
*       setup to manage Rx FDQ and create a flow by setting 'bManageRxFlowCfg' to 1
*       and specifying descriptor configuration in 'rxFlowCfg.drvCfg'. Alternatively,
*       the application can choose to create the Rx FDQs required and setup the complete
*       flow configuration in 'rxFlowCfg.fullCfg' and setting 'bManageRxFlowCfg' to
*       0. In this case, the driver doesnt do any Rx FDQ setup and uses the flow
*       configuration specified as is to create a new flow.
*
*       Alternatively, the application can choose to share a single flow/Rx FDQ
*       between 2 or more Rx objects. It can do so by specifying the flow
*       Id it'd like to share in 'useFlowId' parameter. This API then doesnt 
*       create a new flow/Rx FDQ for this Rx object and ensures that the Rx
*       properties for this object are inherited from the flow number specified.
*
*       An application can choose to either use High priority interrupts
*       or polling for a Rx object. If interrupts are required, it can
*       indicate the same by setting 'bUseInterrupts' to 1 and specifying the
*       accumulator configuration in 'bManageAccumList' and 'accumCfg' parameters.
*
*       The application can also configure whether it wants its receive to be a 
*       blocking/non-blocking operation by indicating the same in 'bBlockOnResult'
*       parameter.
*
*       On success, this API returns a valid, non-zero Rx oject handle and returns 
*       NULL otherwise to indicate an error.
*
*       Every application that wishes to receive FFT results using the driver
*       API is expected to call this API to obtain a Rx object handle.
*
*  @param[in]
*       hFFTC               FFTC driver handle obtained using @a Fftc_open () 
*                           API.
*
*  @param[in]  
*       pFFTCRxCfg          Input structure that holds configuration for the
*                           Rx object.
*
*  @return      Fftc_RxHandle
*  @li          NULL                -   Error. Invalid Rx object configuration. 
*  @li          Valid Rx handle     -   Success. FFTC Rx object, Rx queue successfully 
*                                       setup. 
*
*  @pre
*  @n   Valid FFTC driver handle must be obtained by calling @a Fftc_open () API
*       by the application before calling this API. Valid Rx configuration must be 
*       specified in 'pFFTCRxCfg' structure as input to this API. 
*
*  @post
*  @n   A Rx queue, flow successfully setup with input configuration specified. The
*       Rx object can now be used to retrieve results from the FFTC engine. 
* ============================================================================
*/
Fftc_RxHandle Fftc_rxOpen
(
    Fftc_DrvHandle              hFFTC,
    Fftc_RxCfg*                 pFFTCRxCfg
)
{
    int32_t                     result;                
    Fftc_UserInfo*              pFFTCUserInfo;
    Cppi_RxFlowCfg              rxFlowCfg;
    Qmss_QueueHnd               hQmssRxFreeQ = (Qmss_QueueHnd)NULL, hQmssRxQ;
    Cppi_FlowHnd                hCppiRxFlow;
    Qmss_Queue                  rxFreeQInfo, rxQInfo;
    Qmss_QueueType              queueType;
    Qmss_QueueHnd               hRxGlobalFreeQ;
    uint8_t                     isAllocated, accChannelNum, bUsesDrvRxFlowCfg = 0, bUsesDrvAccumList = 0, bPSInfoPresent;
    Fftc_RxInfo*                pRxObjInfo = NULL;
    Fftc_RxGlobalInfo*          pRxGlobalObjInfo = NULL;
    uint32_t                    descSize, instNum, flowId, i;
    Fftc_RequestInfo*           pRequestInfo;
    Fftc_FlowInfo*              pFlowInfo;
    void*                       hResultSem = NULL;
    uint32_t                    *pAccEntry = NULL, *pOrigAccEntryAddress = NULL;
    uint16_t                    numAccEntries, listSize, intThreshold;
    Qmss_AccCmdCfg              cfg;
    Cppi_PSLoc                  psLocation;

    /* Validate input. */
    if (!hFFTC)
    {
        Fftc_osalLog ("Error! Invalid FFTC driver handle passed !! \n");
        return NULL;
    }

    if (!pFFTCRxCfg)
    {
        Fftc_osalLog ("Error! Invalid Rx configuration handle provided! \n");
        return NULL;
    }

    if (pFFTCRxCfg->useFlowId == -1 && pFFTCRxCfg->bManageRxFlowCfg && !pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc)
    {
        Fftc_osalLog ("Error! Number of Rx Descriptors must be atleast one (cppiNumDesc >= 1)! \n");
        return NULL;
    }

    /* If the Rx object wants to re-use an existing flow configuration,
     * check if the flow requested is indeed valid.
     */
    /* Critical Section Start */
    Fftc_osalMultiCoreCsEnter ();            

    pFFTCUserInfo   =   (Fftc_UserInfo *) hFFTC;
    instNum         =   pFFTCUserInfo->instNum;

    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    if (pFFTCRxCfg->useFlowId != -1 && !Fftc_isValidFlow (hFFTC, pFFTCRxCfg->useFlowId))
    {
        Fftc_osalLog ("Error! Flow id specified invalid! \n");
        Fftc_osalMultiCoreCsExit ();
        return NULL;
    }

    /*************************************************************
     *************** Rx queue and Interrupt Setup. ***************                        
     *************************************************************/
    /* Configure interrupts for this Rx object */
    if (pFFTCRxCfg->bUseInterrupts)
    {
        /* Check if the application wants driver to manage the accumulator list
         * for this Rx object or not.
         */
        if (pFFTCRxCfg->bManageAccumList)
        {
            /* Use driver specified accumulator configuration to program 
             * accumulator.
             */
            if (!pFFTCRxCfg->accumCfg.drvCfg.intThreshold)
            {
                /* A non-zero accumulator list size, i.e. entry count threshold value
                 * must be specified since this threshold value controls the number 
                 * of result entries that can be buffered safely without being 
                 * overwritten while the interrupt is being paced/serviced. 
                 */
                Fftc_osalLog ("Error! A non-zero entry count must be specified in 'intThreshold' parameter.\n");
                /* exit critical section */
                Fftc_osalMultiCoreCsExit ();
                return NULL;
            }

            /* Set the interrupt configuration defaults for non-paced mode. 
             * In pacing mode, configuration is taken from the application.
             */
            if (!pFFTCRxCfg->accumCfg.drvCfg.bEnablePacing)
            {
                /*  Pacing disabled, discard the pacing timer frequency configuration 
                 *  and override with defaults.
                 *
                 *  The defaults are to disable pacing completely.
                 */            
                pFFTCRxCfg->accumCfg.drvCfg.pacingFrequency   =   0;
                pFFTCRxCfg->accumCfg.drvCfg.pacingMode        =   Qmss_AccPacingMode_NONE;
            }

            /* Calculate size of accumulator list to allocate. 
             *
             * The High priority accumulator list consists of 2 buffers Ping and
             * Pong each consisting of the following entries:
             *
             * (1)  Entry count -   specifies number of results accumulated in
             *                      the list.
             * (2)  Descriptors -   an array of result descriptors accumulated
             *                      in this list.
             *
             * Hence the size of high priority accumulator list is calculated as 
             * follows:
             *
             * (1)  Get the interrupt threshold requested on this core by the user,
             *      i.e., number of entries requested.
             * (2)  Add an extra entry to the threshold requested to accomodate
             *      entry count of the list.
             * (3)  Double this to accomodate space for Ping/Pong lists.
             * (4)  The address must be 16-byte aligned. So, in case we need to
             *      align it we need extra 16 bytes for the alignment.
             *  
             * size =   ((interrupt threshold requested on this core + 1) * 2 ) + 16
             */
            numAccEntries   =   (pFFTCRxCfg->accumCfg.drvCfg.intThreshold + 1) * 2;
            listSize        =   (numAccEntries * sizeof (uint32_t)) + 16;

            /* Create Accumulator lists for this Rx object */
            if (!(pAccEntry =   (uint32_t *) Fftc_osalMalloc (listSize, TRUE)))
            {
                Fftc_osalLog ("Error allocating memory for Rx object's accumulator list\n");            
                /* exit critical section */
                Fftc_osalMultiCoreCsExit ();
                return NULL;
            }
   
            /* Store the original accumulator entry address obtained
             * from the heap. We will need this when freeing the memory
             * for this list.
             */
            pOrigAccEntryAddress    =   pAccEntry;

            /* Initialize the accumulator list memory */
            memset ((void *) pAccEntry, 0, listSize);

            /* Accmulator list must be 16 byte aligned. If not aligned, align it. */
            pAccEntry   =   (uint32_t *)((uint32_t) pAccEntry +  (16 - (((uint32_t) pAccEntry) % 16)));

            /* We need to have either a valid Rx queue number or
             * a valid queue type to open a queue.
             */
            if (pFFTCRxCfg->cppiRxQNum == QMSS_PARAM_NOT_SPECIFIED)
            {
                /* The application requested a Rx Q with interrupts capability.
                 *        
                 * The driver supports only High priority accumulation interrupts.
                 * Use a high priority queue capable of High Priority accumulation 
                 * for this Rx object.
                 */
                queueType = Qmss_QueueType_HIGH_PRIORITY_QUEUE;            
            }
            else
            {
                /* We have a valid queue number, the queue tye can be set to 0 */
                queueType = (Qmss_QueueType) 0;            
            }
        
            /* Open the High priority accumulation receive queue. */
            if ((hQmssRxQ = Qmss_queueOpen (queueType, pFFTCRxCfg->cppiRxQNum, &isAllocated)) < 0)
            {
                Fftc_osalLog ("Error opening High Priority Rx Queue Number %d \n",pFFTCRxCfg->cppiRxQNum);

                /* Free the accumulator list before returning */
                if (pOrigAccEntryAddress)
                    Fftc_osalFree (pOrigAccEntryAddress, listSize, TRUE);

                /* exit critical section */
                Fftc_osalMultiCoreCsExit ();
                return NULL;
            }

            /* Get the receive queue number in case it wasnt specified */
            pFFTCRxCfg->cppiRxQNum  =   Qmss_getQIDFromHandle(hQmssRxQ);
                
            /* Setup the accumulator settings */
            cfg.channel             =   pFFTCRxCfg->accumCfg.drvCfg.accChannelNum;
            cfg.command             =   Qmss_AccCmd_ENABLE_CHANNEL;
            cfg.queueEnMask         =   0;
            cfg.listAddress         =   (uint32_t) pAccEntry;
            cfg.queMgrIndex         =   hQmssRxQ;
            cfg.maxPageEntries      =   (pFFTCRxCfg->accumCfg.drvCfg.intThreshold + 1); /* Add an extra entry for holding the entry count */
            cfg.timerLoadCount      =   pFFTCRxCfg->accumCfg.drvCfg.pacingFrequency;
            cfg.interruptPacingMode =   pFFTCRxCfg->accumCfg.drvCfg.pacingMode;
            cfg.listEntrySize       =   Qmss_AccEntrySize_REG_D;
            cfg.listCountMode       =   Qmss_AccCountMode_ENTRY_COUNT;
            cfg.multiQueueMode      =   Qmss_AccQueueMode_SINGLE_QUEUE;

            accChannelNum           =   pFFTCRxCfg->accumCfg.drvCfg.accChannelNum;
            intThreshold            =   pFFTCRxCfg->accumCfg.drvCfg.intThreshold;
            
            /* Accumulator list being setup using driver configuration */
            bUsesDrvAccumList       =   1;
        
            /* Check if the Rx object has requested a blocking/non-blocking receive */
            if (pFFTCRxCfg->bBlockOnResult)
            {
                /* Blocking requested by application for this Rx object on Receive operation.
                 *
                 * Create a semaphore to block for this application.
                 */
                hResultSem  =   (void*) Fftc_osalCreateSem ();

                if (!hResultSem)
                {
                    Fftc_osalLog ("Error creating semaphore for this Rx object\n");

                    /* Free the accumulator list before returning */
                    if (pOrigAccEntryAddress)
                        Fftc_osalFree (pOrigAccEntryAddress, listSize, TRUE);

                    /* Close the Rx queue just opened. */
                    Qmss_queueClose (hQmssRxQ);

                    /* exit critical section */
                    Fftc_osalMultiCoreCsExit ();
                    return NULL;
                }
            }
        }
        else
        {
            /* Use application specified accumulator configuration as is */                
            cfg                     =   pFFTCRxCfg->accumCfg.fullCfg;                

            accChannelNum           =   cfg.channel;
            intThreshold            =   cfg.maxPageEntries;
            pAccEntry               =   (uint32_t *) cfg.listAddress;
        }
  
        /* Ensure that the accumulator channel we are programming is not 
         * in use currently.
         */
        result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel);
        if (result != QMSS_ACC_SOK && result != QMSS_ACC_CHANNEL_NOT_ACTIVE)
        {
            Fftc_osalLog ("Error Disabling PDSP1 accumulator for channel : %d error code: %d\n",
                          cfg.channel, result);
            goto return_error;
        }

        /* Program the accumulator */
        if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
        {
            Fftc_osalLog ("Error Programming PDSP1 accumulator for channel : %d queue : %d error code : %d\n",
                          cfg.channel, cfg.queMgrIndex, result);

            /* Free the accumulator list before returning */
            if (pOrigAccEntryAddress)
                Fftc_osalFree (pOrigAccEntryAddress, listSize, TRUE);

            /* Close the Rx queue just opened. */
            Qmss_queueClose (hQmssRxQ);

            /* exit critical section */
            Fftc_osalMultiCoreCsExit ();
            return NULL;
        }
    }
    else 
    {
        /* If we are here, it could be because:
         *     
         * No interrupt support is required on the Rx object
         * being created.
         *
         * Open the Rx Queue for this Rx object based on input specified.
         *
         * We need to specify either a Rx queue number or a queue type.
         * If Rx queue not specified, we'll let the QMSS library pick the next
         * available Rx queue for us.
         */
        if (pFFTCRxCfg->cppiRxQNum != QMSS_PARAM_NOT_SPECIFIED)
        {
            /* Queue number specified. Queue type must be zero for QMSS library to
             * correctly configure the queue.
             */
            queueType = (Qmss_QueueType) 0;
        }
        else
        {
            /* No specific queue number specified. Lets pick next available 
             * Rx Queue from the General purpose queue pool since we do not
             * need any interrupt capability on it.
             */
            queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;            
        }

        /* Open the receive queue. */
        if ((hQmssRxQ = Qmss_queueOpen (queueType, pFFTCRxCfg->cppiRxQNum, &isAllocated)) < 0)
        {
            Fftc_osalLog ("Error opening Rx Queue Number %d \n",pFFTCRxCfg->cppiRxQNum);
            /* exit critical section */
            Fftc_osalMultiCoreCsExit ();
            return NULL;
        }

        /* Get the receive queue number in case it wasnt specified */
        pFFTCRxCfg->cppiRxQNum          =   Qmss_getQIDFromHandle(hQmssRxQ);
    }

    /*************************************************************
     ****************    Flow and Rx FDQ Setup.  *****************                        
     *************************************************************/

    /* Check if we need to create a new flow and FDQ or if
     * we should re-use an existing one for this Rx object.
     */
    if (pFFTCRxCfg->useFlowId == -1)
    {
        /* Check whether the driver should setup the Rx FDQ and 
         * manage flow configuration or if the application is
         * managing them.
         */
        if (pFFTCRxCfg->bManageRxFlowCfg)            
        {
            /* Driver is responsible for creating and managing the Rx FDQ/Flow */                

            /* Calculate Rx descriptor size */
            if (pFFTCRxCfg->rxFlowCfg.drvCfg.descType == Cppi_DescType_HOST)
            {
                /* If the receive packet could carry a PS info, we need to make sure that
                 * we allocate memory for that too.
                 */
                if (pFFTCRxCfg->rxFlowCfg.drvCfg.bPSInfoPresent)
                {
                    if ((Cppi_PSLoc) pFFTCRxCfg->rxFlowCfg.drvCfg.psLocation == Cppi_PSLoc_PS_IN_SOP)
                    {
                        /* If PS Info will be in SOP Buffer allocate space in the Rx Free
                         * Buffers to hold it.
                         *
                         * The maximum amount of PS info that can be forwarded using
                         * FFTC is 4 32-bit words = 4 * 4 bytes
                         */
                        pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize  +=  (4 * FFTC_MAX_NUM_PS_WORDS);                  

                        /* No EPIB/PS Info in descriptor, hence descriptor size is just the 
                         * basic size. 
                         */
                        descSize    =   FFTC_CPPI_HOST_DESC_SIZE;
                    }
                    else
                    {
                        /* Reserve space for the largest PS data that can be sent
                         * using FFTC = 4 * 4 bytes
                         */
                        descSize    =   FFTC_CPPI_HOST_DESC_SIZE + (4 * FFTC_MAX_NUM_PS_WORDS);

                        /* Descriptor sizes must be 16 byte aligned */
                        descSize    +=  (16 - (descSize % 16));
                    }
                }
                else
                {
                    /* No EPIB/PS Info, hence descriptor size is just the basic size. */
                    descSize    =   FFTC_CPPI_HOST_DESC_SIZE;
                }
            }
            else
            {
                /* For monolithic descriptors, the descriptor size will always include
                 * the buffer size irrespective of where PS info is present.
                 */
                descSize    =   pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize + FFTC_CPPI_MONOLITHIC_DESC_SIZE;

                if (pFFTCRxCfg->rxFlowCfg.drvCfg.bPSInfoPresent)
                {
                    /* Reserve space for the largest PS data that can be sent
                     * using FFTC = 4 * 4 bytes
                     */
                    descSize    +=  (4 * FFTC_MAX_NUM_PS_WORDS);
                }

                /* Descriptor sizes must be 16 byte aligned */
                descSize    +=  (16 - (descSize % 16));
            }    

            /* Get the Receive free queue matching the buffer size provided
             * with pre-allocated buffers attached.
             */    
            if ((hQmssRxFreeQ = Fftc_allocFreeQBuffers (pFFTCUserInfo,
    											        pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc, 
                                                        pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize, 
                                                        pFFTCRxCfg->rxFlowCfg.drvCfg.descType, 
                                                        descSize,
                                                        &result, 
                                                        &hRxGlobalFreeQ)) 
                                                        == (Qmss_QueueHnd) NULL)
            {
                Fftc_osalLog ("No Rx Free Descriptors allocated for this Rx object! \n");
                goto return_error;
            }    

            /* Check if expected number of descriptors were populated correctly with buffers. */
            if (result != pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc)
            {
                Fftc_osalLog ("Error attaching %d buffers of type %d descriptors on Rx Queue, numAllocated: %d \n", 
                            pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc, pFFTCRxCfg->rxFlowCfg.drvCfg.descType, result);

                /* Lets clean up the Rx buffers we allocated and return error */
                Fftc_cleanFreeQBuffers (hQmssRxFreeQ, 
                                        hRxGlobalFreeQ, 
                                        result, 
                                        pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize, 
                                        descSize);

                /* Delete any semaphore created */
                if (hResultSem)
                    Fftc_osalDeleteSem (hResultSem);

                /* Free the accumulator list before returning */
                if (pOrigAccEntryAddress)
                    Fftc_osalFree (pOrigAccEntryAddress, listSize, TRUE);

                /* Close the Rx queue just opened. */
                Qmss_queueClose (hQmssRxQ);

                /* exit critical section */
                Fftc_osalMultiCoreCsExit ();
                return NULL;
            }

            /* Get the Rx FDQ info */
            rxFreeQInfo                     =   Qmss_getQueueNumber (hQmssRxFreeQ);

            /* Initialize the flow configuration */
            memset (&rxFlowCfg, 0, sizeof(Cppi_RxFlowCfg));

            /* Setup a Rx Flow for this Rx object */
            rxFlowCfg.flowIdNum             =   CPPI_PARAM_NOT_SPECIFIED; 
            rxQInfo                         =   Qmss_getQueueNumber (hQmssRxQ);
            
            rxFlowCfg.rx_dest_qmgr          =   rxQInfo.qMgr;    
            rxFlowCfg.rx_dest_qnum          =   rxQInfo.qNum;  
            rxFlowCfg.rx_desc_type          =   (Cppi_DescType) pFFTCRxCfg->rxFlowCfg.drvCfg.descType; 

            if (pFFTCRxCfg->rxFlowCfg.drvCfg.descType == Cppi_DescType_HOST)   
            {
                /* PS location configuration is valid for Host descriptors only.
                 *
                 * In monolithic descriptors, PS info is always put in the PS words
                 * of the descriptor.
                 */
                rxFlowCfg.rx_ps_location    =   (Cppi_PSLoc) pFFTCRxCfg->rxFlowCfg.drvCfg.psLocation;  
            }
            rxFlowCfg.rx_psinfo_present     =   pFFTCRxCfg->rxFlowCfg.drvCfg.bPSInfoPresent;           

            /* For monolithic descriptors, the PS info is put in the
             * PS Data words section of the descriptor and not as part of the
             * data itself, hence the SOP offset would have to be configured
             * accordinlgy to skip over the PS info too.
             */
            if (pFFTCRxCfg->rxFlowCfg.drvCfg.descType == Cppi_DescType_MONOLITHIC)
            {
                rxFlowCfg.rx_sop_offset     =   FFTC_CPPI_MONOLITHIC_DESC_SIZE;

                if (pFFTCRxCfg->rxFlowCfg.drvCfg.bPSInfoPresent)
                {
                    /* FFTC can handle upto maximum of 4 32bit words of PS info */            
                    rxFlowCfg.rx_sop_offset +=  4 * FFTC_MAX_NUM_PS_WORDS;
                }
            }

            rxFlowCfg.rx_error_handling     =   0;    /* Drop the packet, do not retry on starvation by default */       
            rxFlowCfg.rx_einfo_present      =   0;    /* By default no EPIB info */       
    
            rxFlowCfg.rx_dest_tag_lo_sel    =   4;    /* Always pick the dest tag 7:0 bits from the PD dest_tag */       
            rxFlowCfg.rx_dest_tag_hi_sel    =   5;    /* Always pick the dest tag 15:8 bits from the PD dest_tag */       

            rxFlowCfg.rx_src_tag_lo_sel     =   2;    /* Always pick the src tag 7:0 bits from the PD flow_id 7:0 */       
            rxFlowCfg.rx_src_tag_hi_sel     =   4;    /* Always pick the src tag 15:8 bits from the PD src_tag 7:0 */

            /* By default, we disable Rx Thresholds */
            rxFlowCfg.rx_size_thresh0_en    =   0;    
            rxFlowCfg.rx_size_thresh1_en    =   0;    
            rxFlowCfg.rx_size_thresh2_en    =   0;    

            rxFlowCfg.rx_size_thresh0       =   0x0;
            rxFlowCfg.rx_size_thresh1       =   0x0;
            rxFlowCfg.rx_size_thresh2       =   0x0;

            rxFlowCfg.rx_fdq0_sz0_qmgr      =   rxFreeQInfo.qMgr; /* Setup the Receive free queue for the flow */
            rxFlowCfg.rx_fdq0_sz0_qnum      =   rxFreeQInfo.qNum;    
            rxFlowCfg.rx_fdq0_sz1_qnum      =   0x0; 
            rxFlowCfg.rx_fdq0_sz1_qmgr      =   0x0;
            rxFlowCfg.rx_fdq0_sz2_qnum      =   0x0;
            rxFlowCfg.rx_fdq0_sz2_qmgr      =   0x0;
            rxFlowCfg.rx_fdq0_sz3_qnum      =   0x0;
            rxFlowCfg.rx_fdq0_sz3_qmgr      =   0x0;

            rxFlowCfg.rx_fdq1_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
            rxFlowCfg.rx_fdq1_qmgr          =   rxFreeQInfo.qMgr;
            rxFlowCfg.rx_fdq2_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
            rxFlowCfg.rx_fdq2_qmgr          =   rxFreeQInfo.qMgr;
            rxFlowCfg.rx_fdq3_qnum          =   rxFreeQInfo.qNum;  /* Use the Rx Queue to pick descriptors */
            rxFlowCfg.rx_fdq3_qmgr          =   rxFreeQInfo.qMgr;

            /* Configure the Rx flow */
            if ((hCppiRxFlow = Cppi_configureRxFlow (Fftc_instanceInfo[instNum].hCppi, 
                                                    &rxFlowCfg, 
                                                    &isAllocated)) 
                                                    == NULL)
            {
                Fftc_osalLog ("Error configuring Rx flow \n");
                goto return_error;
            }

            /* Flow setup using driver Rx flow configuration. */
            bUsesDrvRxFlowCfg               =   1;
            bPSInfoPresent                  =   pFFTCRxCfg->rxFlowCfg.drvCfg.bPSInfoPresent;
            psLocation                      =   pFFTCRxCfg->rxFlowCfg.drvCfg.psLocation;
        }
        else
        {
            /* Configure the Rx flow with the parameters specified by application */
            if ((hCppiRxFlow = Cppi_configureRxFlow (Fftc_instanceInfo[instNum].hCppi, 
                                                    &pFFTCRxCfg->rxFlowCfg.fullCfg, 
                                                    &isAllocated)) 
                                                    == NULL)
            {
                Fftc_osalLog ("Error configuring Rx flow \n");
                goto return_error;
            }
            /* Flow setup using application managed flow configuration */
            bUsesDrvRxFlowCfg               =   0;  
            descSize                        =   0;
            bPSInfoPresent                  =   pFFTCRxCfg->rxFlowCfg.fullCfg.rx_psinfo_present;
            psLocation                      =   (Cppi_PSLoc) pFFTCRxCfg->rxFlowCfg.fullCfg.rx_ps_location;
        }

        /* Get the flow id corresponding to the flow we just created. */
        flowId      =   Cppi_getFlowId (hCppiRxFlow);
    }
    else
    {
        /* Re-use an existing flow/FDQ for this Rx object. */            
        flowId      =   pFFTCRxCfg->useFlowId;
    }

    /* Get the flow info handle for the flow Rx object is using
     * from the driver's global database.
     */
    pFlowInfo   =   &Fftc_instanceInfo[instNum].Fftc_flowInfo[flowId];

    /*************************************************************
     *****************    Save Info and Return.  *****************                        
     *************************************************************/
    /* Get the next available slot from the Rx object global database to 
     * store our info.
     */
    if ((result = Fftc_getNextAvailRxObjId (instNum)) == FFTC_RETVAL_EFAILURE)
    {
        Fftc_osalLog ("Error! Max number of Rx objects exceeded. \n");
        goto return_error;
    }
    pRxGlobalObjInfo   =   &Fftc_instanceInfo[pFFTCUserInfo->instNum].Fftc_rxObjGlobalInfo[result];
    /* Allocate memory for holding the Rx object info */
    if ((pRxObjInfo = Fftc_osalMalloc (sizeof (Fftc_RxInfo), FALSE)) == NULL)
    {
        Fftc_osalLog ("Error allocating memory for Rx object \n");
        goto return_error;
    }
    memset ((void*) pRxObjInfo, 0, sizeof (Fftc_RxInfo));
    memset ((void*) pRxGlobalObjInfo, 0, sizeof (Fftc_RxGlobalInfo));

    /* Allocate result buffers to hold the results received from
     * driver ISR.
     *
     * We dont need to allocate request buffers if using polling or
     * if the application owns the accumulator list and ISR handling.
     */
    if (pFFTCRxCfg->bUseInterrupts && pFFTCRxCfg->bManageAccumList)
    {
        /* Number of result buffers = Interrupt threshold * 2 
         *
         * Allocate memory for holding FFT results received by the ISR.  
         */
        for (i = 0; i < (pFFTCRxCfg->accumCfg.drvCfg.intThreshold * 2); i ++)
        {
            if (!(pRequestInfo = Fftc_osalMalloc (sizeof (Fftc_RequestInfo), FALSE)))
            {
                Fftc_osalLog ("Error allocating results for this Rx object.\n");
                goto return_error;
            }
            memset ((void *)pRequestInfo, 0, sizeof (Fftc_RequestInfo));

            /* Populate the free FFT requests list for this Rx object */
            Fftc_listCat (  (Fftc_ListNode **)&pRxObjInfo->freeResultQueue,
                            (Fftc_ListNode **)&pRequestInfo);                       
            pRxObjInfo->resultQueue     =   NULL;
            pRxObjInfo->resultQueueLen  =   0;
        }    
    }

    /* Save the flow settings in driver database if we created a new one. */
    if (pFFTCRxCfg->useFlowId == -1)
    {
        memset (pFlowInfo, 0, sizeof (Fftc_FlowInfo));  
        /* Save FDQ info */
        pFlowInfo->bUsesDrvRxFlowCfg    =   bUsesDrvRxFlowCfg;
        pFlowInfo->hQmssRxFreeQ         =   hQmssRxFreeQ;
        pFlowInfo->hQmssRxGlblFreeQ     =   hRxGlobalFreeQ;
        pFlowInfo->cppiNumDesc          =   pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc;
        pFlowInfo->descSize             =   descSize;
        pFlowInfo->bufferSize           =   pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize;
        pFlowInfo->hCppiRxFlow          =   hCppiRxFlow;    
        pFlowInfo->cppiFlowId           =   flowId;   
        pFlowInfo->bPSInfoPresent       =   bPSInfoPresent;
        pFlowInfo->psLocation           =   psLocation;
    }

    /* Increment the flow reference count to keep it around while in use by this Rx object. */
    pFlowInfo->refCnt ++;

    /* Initialize the Rx global object in the driver's global Rx object database and mark it "valid" */
    pRxGlobalObjInfo->Id                =   result;
    pRxGlobalObjInfo->bIsValid          =   1;
    pRxGlobalObjInfo->cppiFlowId        =   flowId;   
    pRxGlobalObjInfo->cppiRxQNum        =   pFFTCRxCfg->cppiRxQNum;

    /* We are all done here. Save the Rx info and return handle */
    pRxObjInfo->hFFTC                   =   hFFTC;
    pRxObjInfo->bUsesDrvAccumList       =   bUsesDrvAccumList;    
    pRxObjInfo->cppiFlowId              =   flowId;   
    pRxObjInfo->cppiRxQNum              =   pFFTCRxCfg->cppiRxQNum;
    pRxObjInfo->cfg                     =   *pFFTCRxCfg;
    pRxObjInfo->pOrigAccListAddress     =   pOrigAccEntryAddress;
    pRxObjInfo->pHiPrioAccList          =   pAccEntry;    
    pRxObjInfo->hQmssRxQ                =   hQmssRxQ;
    pRxObjInfo->hResultSem              =   hResultSem;
    pRxObjInfo->accChannelNum           =   accChannelNum;
    pRxObjInfo->intThreshold            =   intThreshold;
    pRxObjInfo->rxGlobalObjId			=	result;
    pRxObjInfo->descSize                =   pFlowInfo->descSize;

    /* PS Info read from Rx descriptors is no longer reliable because
     * of a CDMA H/W Bug. Hence save the settings so as to use on Rx
     * results.
     */    
    pRxObjInfo->bPSInfoPresent          =   pFlowInfo->bPSInfoPresent;
    pRxObjInfo->psLocation              =   pFlowInfo->psLocation;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* Done here. Release the CS lock */
    Fftc_osalMultiCoreCsExit ();

    /* Return success. Return global Rx object handle. */
    return ((Fftc_RxHandle) pRxObjInfo);

return_error:
    /* Lets clean up the Rx buffers and their queues and return error */
    if (hQmssRxFreeQ)
    {
        Fftc_cleanFreeQBuffers (hQmssRxFreeQ, 
                                hRxGlobalFreeQ, 
                                pFFTCRxCfg->rxFlowCfg.drvCfg.cppiNumDesc, 
                                pFFTCRxCfg->rxFlowCfg.drvCfg.bufferSize, 
                                descSize);
    }
        
    /* Free the accumulator list before returning */
    if (pOrigAccEntryAddress)
        Fftc_osalFree ((void *)pOrigAccEntryAddress, listSize, TRUE);

    /* Close the Rx queue just opened. */
    Qmss_queueClose (hQmssRxQ);

    /* Delete if we create a Rx semaphore */
    if (hResultSem)
        Fftc_osalDeleteSem (hResultSem);

    /* Free all the FFT results allocated so far */
    while ((pRequestInfo = (Fftc_RequestInfo *) Fftc_listRemove ((Fftc_ListNode **)&pRxObjInfo->freeResultQueue)))
        Fftc_osalFree ((void *)pRequestInfo, sizeof (Fftc_RequestInfo), FALSE);            

    /* Free up the Rx object */
    Fftc_osalFree ((void *)pRxObjInfo, sizeof (Fftc_RxInfo), FALSE);

    /* Free up the Rx object's global counterpart too. */
    if (pRxGlobalObjInfo)
    	pRxGlobalObjInfo->bIsValid  =   0;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[instNum], sizeof (Fftc_InstanceInfoObj));

    /* exit critical section */
    Fftc_osalMultiCoreCsExit ();

    return NULL;
}

/**
* ============================================================================
*  @n@b Fftc_rxClose
*
*  @b   brief
*  @n   This API closes the Rx object and closes the Rx flow and cleans up all 
*       associated handles. All pending FFTC results are dicarded for this 
*       Rx object.
*
*  @param[in]
*       hFFTCRxInfo         FFTC Rx object handle obtained using @a Fftc_rxOpen() 
*                           API.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   FFTC Rx object close failed.
*  @li         FFTC_RETVAL_SUCCESS      -   FFTC Rx object successfully closed.
*
*  @pre
*  @n   Valid Rx object handle must be obtained using Fftc_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   FFTC Rx object, Rx flow successfully closed and all pending FFTC  
*       results on the Rx object are dicarded.
* ============================================================================
*/
Fftc_RetVal Fftc_rxClose 
(
    Fftc_RxHandle               hFFTCRxInfo
)
{
    Fftc_RxInfo*                pRxObjInfo;
    Fftc_RxGlobalInfo*          pRxGlobalObjInfo;
    uint16_t                    flowId, listSize;
    Fftc_UserInfo*              pFFTCUserInfo;
    Fftc_RequestInfo*           pRequestInfo;
    Fftc_FlowInfo*              pFlowInfo;

    /* Validate input handles and driver state  */
    if (!hFFTCRxInfo)
    {
        Fftc_osalLog ("Invalid Receive Object Handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;	
    }

    pRxObjInfo      =   (Fftc_RxInfo *) hFFTCRxInfo;
    pFFTCUserInfo   =   (Fftc_UserInfo *) pRxObjInfo->hFFTC;
    flowId          =   pRxObjInfo->cppiFlowId;

    /* Enter Critical Section */
    Fftc_osalMultiCoreCsEnter ();
    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));
    pFlowInfo       =   &Fftc_instanceInfo[pFFTCUserInfo->instNum].Fftc_flowInfo[flowId];
    pRxGlobalObjInfo=   &Fftc_instanceInfo[pFFTCUserInfo->instNum].Fftc_rxObjGlobalInfo[pRxObjInfo->rxGlobalObjId];

    /* Free all the Rx packets, Free results, pending and free requests. */
    while ((pRequestInfo = (Fftc_RequestInfo *) Fftc_listRemove ((Fftc_ListNode **) &pRxObjInfo->resultQueue)))
    {
        /* Push descriptor back to free queue */
        if (pRequestInfo->pCppiDesc)
            Qmss_queuePushDescSize (pFlowInfo->hQmssRxFreeQ, pRequestInfo->pCppiDesc, pFlowInfo->descSize);
        Fftc_osalFree (pRequestInfo, sizeof(Fftc_RequestInfo), FALSE);
    }
    while ((pRequestInfo = (Fftc_RequestInfo *) Fftc_listRemove ((Fftc_ListNode **) &pRxObjInfo->freeResultQueue)))
    {
        Fftc_osalFree ((void *)pRequestInfo, sizeof(Fftc_RequestInfo), FALSE);
    }    

    /* Close the Rx queue */
    Qmss_queueClose (pRxObjInfo->hQmssRxQ);

    /* Clean up any interrupt configuration if interrupts were setup for this Rx object. */
    if (pRxObjInfo->bUsesDrvAccumList)
    {
        /* Disable accumulator channel. */
        Qmss_disableAccumulator (Qmss_PdspId_PDSP1, pRxObjInfo->accChannelNum);

        /* Calculate the accumulator list size */
        listSize            =   (sizeof (uint32_t) *  (pRxObjInfo->intThreshold + 1) * 2) + 16;

        /* Free the accumulator list. 
         * Use the original address and not aligned accumulator address.
         */
        Fftc_osalFree (pRxObjInfo->pOrigAccListAddress, listSize, TRUE);
    }

    /* De-allocate the semaphore obtained */
    if (pRxObjInfo->hResultSem)
    {
        Fftc_osalDeleteSem (pRxObjInfo->hResultSem);
    }

    /* Decrement reference count to release the flow */
    pFlowInfo->refCnt --;

    /* If flow not in use by any Rx object clean it up */
    if (pFlowInfo->refCnt == 0)
    {
        /* Free the result buffers (Rx) and close the Rx Free Q if driver
         * created them
         */
        if (pFlowInfo->bUsesDrvRxFlowCfg)
        {
            Fftc_cleanFreeQBuffers (pFlowInfo->hQmssRxFreeQ, 
                                    pFlowInfo->hQmssRxGlblFreeQ,
                                    pFlowInfo->cppiNumDesc, 
                                    pFlowInfo->bufferSize,
                                    pFlowInfo->descSize);
        }

        Cppi_closeRxFlow (pFlowInfo->hCppiRxFlow);
    }

    /* Free up the Rx object's global counterpart too. */
    pRxGlobalObjInfo->bIsValid  =   0;

    Fftc_osalEndMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));

    /* Release CS lock */
    Fftc_osalMultiCoreCsExit ();

    /* Free up the Rx object space */
    Fftc_osalFree (pRxObjInfo, sizeof (Fftc_RxInfo), FALSE);
   
    /* Return Success */
    return FFTC_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Fftc_findFlowIdByQueueNumber
*
*  @b   brief
*  @n   Given a Rx queue number, this API returns the corresponding flow Id 
*       associated from the driver's Rx object database.
*
*  @param[in]
*       hFFTC               FFTC driver handle obtained using @a Fftc_open () 
*                           API. Specifies which FFTC instance to search for 
*                           a match.
*  @param[in]
*       rxQueueNumber       Rx queue number for which the flow Id lookup must
*                           be performed.
*
*  @return     Fftc_RetVal
*  @li         <0                       -   Flow Id retrieval failed. Unable 
*                                           to find a flow for the Rx queue 
*                                           number and FFTC instance
*                                           provided in the driver database.
*  @li         >=0                      -   Success. Valid Flow Id returned.
*
*  @pre
*  @n   A valid driver handle must be obtained using @a Fftc_open () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Fftc_RetVal Fftc_findFlowIdByQueueNumber
(
    Fftc_DrvHandle              hFFTC,
    uint32_t                    rxQueueNumber
)
{
    uint32_t                    i;
    Fftc_UserInfo*              pFFTCUserInfo;
    Fftc_InstanceInfoObj*       pFFTCInstInfo;

    /* Invalid driver handle specified. */
    if (!hFFTC)
    {
        Fftc_osalLog ("Invalid driver handle specified \n");
        return FFTC_RETVAL_EBAD_HANDLE;            
    }

    pFFTCUserInfo           =   (Fftc_UserInfo *) hFFTC;

    /* Enter critical section */
    Fftc_osalMultiCoreCsEnter ();

    Fftc_osalBeginMemAccess (&Fftc_instanceInfo[pFFTCUserInfo->instNum], sizeof (Fftc_InstanceInfoObj));
    pFFTCInstInfo           =   (Fftc_InstanceInfoObj *) &Fftc_instanceInfo [pFFTCUserInfo->instNum];

    /* Search through the driver's database of all valid
     * Rx objects for the FFTC instance corresponding to the 
     * driver handle passed to find a matching entry
     * for Rx queue number provided.
     */
    for (i = 0; i < FFTC_MAX_NUM_RXOBJECTS; i ++)
    {
        if ((pFFTCInstInfo->Fftc_rxObjGlobalInfo[i].bIsValid) && 
            (pFFTCInstInfo->Fftc_rxObjGlobalInfo[i].cppiRxQNum == rxQueueNumber))
        {
            /* Found a match. Return the flow Id corresponding to this object. */                
            break;                
        }
    }

    /* Check if we found a match or if we are here
     * because we are done searching through all entries
     * and found no matches.
     */
    if (i == FFTC_MAX_NUM_RXOBJECTS)
    {
        /* Found no match. Return error */            
    
        /* Exit critical section */
        Fftc_osalMultiCoreCsExit ();

        return FFTC_RETVAL_EINVALID_PARAMS;
    }

    /* Exit critical section */
    Fftc_osalMultiCoreCsExit ();

    /* Success. Return flow id found. */
    return (pFFTCInstInfo->Fftc_rxObjGlobalInfo[i].cppiFlowId);
}

/**
* ============================================================================
*  @n@b Fftc_rxGetRxQueueNumber
*
*  @b   brief
*  @n   This API retrieves the given Rx object's Receive/Destination queue number.
*       This API is especially useful if the application didnt specify a 
*       Rx Queue number during @a Fftc_rxOpen () API and would like to
*       retrieve the queue number.
*
*  @param[in]
*       hFFTCRxInfo         FFTC Rx object handle obtained using 
*                           @a Fftc_rxOpen() API.

*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Rx Queue Number retrieval failed.
*  @li         >=0                      -   The destination queue number associated 
*                                           with the Rx object's flow.
*
*  @pre
*  @n   Valid Rx object handle must be obtained using @a Fftc_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Fftc_RetVal Fftc_rxGetRxQueueNumber
(
    Fftc_RxHandle               hFFTCRxInfo
)
{
    Fftc_RxInfo*                pRxObjInfo;

    /* Invalid Rx object handle specified. */
    if (!hFFTCRxInfo)
    {
        Fftc_osalLog ("Invalid Rx object handle specified \n");
        return FFTC_RETVAL_EBAD_HANDLE;            
    }

    pRxObjInfo   =   (Fftc_RxInfo *) hFFTCRxInfo;        

    /* Return the destination queue number saved in the Rx object info */
    return (pRxObjInfo->cppiRxQNum);
}


/**
* ============================================================================
*  @n@b Fftc_rxGetFlowId
*
*  @b   brief
*  @n   This API retrieves a given Rx object's Rx flow id number.
*
*  @param[in]
*       hFFTCRxInfo             FFTC Rx object handle obtained using 
*                               @a Fftc_rxOpen() API.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Flow Id value retrieval failed.
*                                           Invalid Rx object handle passed.
*  @li         >=0                      -   The Flow Id associated with the Rx object
*                                           ranging between 0 and 7 (both inclusive).
*
*  @pre
*  @n   Valid Rx object handle must be obtained using @a Fftc_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   None.
* ============================================================================
*/
Fftc_RetVal Fftc_rxGetFlowId
(
    Fftc_RxHandle               hFFTCRxInfo
)
{
    Fftc_RxInfo*                pRxObjInfo;

    /* Invalid Rx object handle specified. */
    if (!hFFTCRxInfo)
    {
        Fftc_osalLog ("Invalid Rx object handle specified \n");
        return FFTC_RETVAL_EBAD_HANDLE;            
    }

    pRxObjInfo       = (Fftc_RxInfo *) hFFTCRxInfo;        

    /* Return flow id */
    return (pRxObjInfo->cppiFlowId);
}

/**
* ============================================================================
*  @n@b Fftc_txGetRequestBuffer
*
*  @b   brief
*  @n   This API provides the calling application with an empty pre-allocated data 
*       buffer that it can  use to fill in the FFTC request data and any Protocol
*       Specific (PS) Information that it would like to pass to the receiver. 
*
*       This API also returns an FFTC request handle (CPPI Descriptor) containing 
*       the request settings that the application would have to pass when 
*       submitting the FFT request using @a Fftc_txSubmitRequest () API. 
*
*       This API prepares a Tx descriptor for request submission. It formulates 
*       an FFTC control header based on the parameters provided and attaches it
*       to the data buffer.
*
*       This API MUST be called only if the Tx object was setup such that the driver
*       manages the request buffers.
*
*  @param[in]
*       hFFTCTxInfo         Tx object handle obtained using Fftc_txOpen () API.
*
*  @param[in]
*       pDFTBlockSizeInfo   DFT block size list configuration structure. Specifies 
*                           the number of DFT blocks in the FFTC request, size of
*                           each DFT block and whether all blocks are of the same
*                           size/not.
*
*  @param[in]
*       pFFTCQConfig        FFTC Queue local configuration input. This needs to be provided
*                           if the FFTC Tx queue needs to be re-configured with a new set
*                           of FFT parameters for request processing. This parameter MUST 
*                           always be specified if using the Tx queue in "shared" mode, 
*                           otherwise MUST be set to NULL. 
*
*  @param[in]
*       psInfoLen           The length of PS information being added to the request in bytes.
*                           FFTC engine hardware allows upto only 16 bytes of PS info. Care
*                           must be taken by the application that the PS info never exceeds
*                           this length. This can be set to 0 if no PS Pass through info 
*                           is to be passed to receiver.
*
*  @param[in]
*       destnFlowId         Flow Id to use for the result corresponding to this packet. 
*                           Specifies the Rx flow/FDQ to use and the Rx queue where the 
*                           result for this packet must be placed.
*
*  @param[in]
*       destnTagInfo        Destination Tag Info. Will be put in the Destination Tag of the Tx
*                           descriptor. If Rx flow configured appropriately, this field is 
*                           copied to Result (rx) descriptor.Can be used by the application to 
*                           its discretion to track the packet.
*
*  @param[out]
*       phFFTCRequest       FFTC request handle. Contains Tx request descriptor.
*
*  @param[out]
*       ppReqDataBuffer     Pointer to data buffer pointer that the application
*                           can use to fill in FFTC request data and PS info. The
*                           driver expects that the application fills the buffer
*                           in the following order: PS info if any followed by the 
*                           request data. 
*
*  @param[out]
*       pMaxDataBufferLen   Pointer that holds data buffer length, i.e.,holds the number of
*                           bytes available to application in the data buffer to fill
*                           in any Protocol specific info (PS info) and FFT request
*                           data.
*
*   Note: The application MUST always fill the data buffer in the following
*   order: Protocol specific pass through info (PS Info) if any available
*   then followed by the actual FFT request blocks.
*
*   If the default Rx queue number used in setting up 'destnFlowId' is 
*   different from the Rx queue to which the result must be sent. The 
*   Queue configuration must be modified accordingly to use the correct
*   queue number.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE      -   Get Request buffer failed. Bad Tx Handle provided.
*  @li         FFTC_RETVAL_EINVALID_PARAMS  -   Error. Invalid configuration parameters provided.
*  @li         FFTC_RETVAL_ENO_MEM          -   Error. No more free descriptors available for this 
*                                               Tx object.
*  @li         FFTC_RETVAL_EFAILURE         -   Error adding the FFTC control header to the descriptor.
*  @li         FFTC_RETVAL_SUCCESS          -   Valid request data buffer, length and request 
*                                               handle returned. API succeeded.
*
*  @pre
*  @n   @a Fftc_txOpen () must be called before calling this API. This API 
*       can be called to obtain a free request buffer only if free request descriptors
*       and buffers were allocated during @a Fftc_txOpen() API (by setting 'bManageReqBuffers' 
*       of 'pFFTCTxCfg' to 1)
* ============================================================================
*/
Fftc_RetVal Fftc_txGetRequestBuffer 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_BlockInfo*             pDFTBlockSizeInfo,
    Fftc_QLocalCfg*             pFFTCQConfig, 
    uint32_t                    psInfoLen,
    uint8_t                    	destnFlowId,
    uint16_t                    destnTagInfo,
    Fftc_RequestHandle*         phFFTCRequest,     
    uint8_t**                   ppReqDataBuffer, 
    uint32_t*                   pMaxDataBufferLen
)
{
    Fftc_TxInfo*                pTxObjInfo;
    void*                       pCppiDesc;
    uint8_t                     *pOrigDataBuffer;
    uint32_t                    origDataBufferLen, psFlags, tmpLen, dataBufferOffset = 0;
    Fftc_ControlHdr             fftcCtrlHdrObj;
    Cppi_DescTag                cppiTagCfg;
    Cppi_DescType               descType;

#ifdef FFTC_DRV_DEBUG
    /* Validate Tx handle specified. */
    if (!hFFTCTxInfo)
    {
        Fftc_osalLog ("Invalid Tx handle passed \n");
        return FFTC_RETVAL_EBAD_HANDLE;            
    }
#endif

    pTxObjInfo          =   (Fftc_TxInfo *) hFFTCTxInfo;

#ifdef FFTC_DRV_DEBUG
    /* Validate input parameters */
    if (!pTxObjInfo->hQmssTxFreeQ)
    {
        Fftc_osalLog ("No pre-allocated buffers on Tx Free Queue for this Tx object. Cannot use this API \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }

    /* Validate queue mode against queue configuration */
    if ((!pTxObjInfo->bSharedMode && pFFTCQConfig) ||
        (pTxObjInfo->bSharedMode && !pFFTCQConfig))
    {
        Fftc_osalLog ("Error! Queue configuration specified doesnt agree with the Tx queue mode \n");
        return FFTC_RETVAL_EINVALID_PARAMS;
    }

    /* DFT size list can be used by only one Tx queue at any point 
     * of time.
     *
     * Ensure that this flow is using a Tx queue that has 
     * DFT size list configuration enabled.
     */
    if (!pTxObjInfo->bEnableDftSizeListCfg && !pDFTBlockSizeInfo->bIsEqualSize)
    {
        Fftc_osalLog ("The Tx object is not configured for mixed DFT size list configuration \n");
        return FFTC_RETVAL_EINVALID_PARAMS;
    }

    /* Validate that the PS info length specified doesnt exceed the
     * maximum allowed by the FFTC engine.
     */
    if (psInfoLen > (FFTC_MAX_NUM_PS_WORDS * 4))
    {
        Fftc_osalLog ("PS Info cannot exceed %d words \n",  FFTC_MAX_NUM_PS_WORDS);           
        return FFTC_RETVAL_EINVALID_PARAMS;
    }
#endif

    /* Done with input validation. 
     *
     * Get a Tx Free descriptor with a pre-allocated buffer and
     * populate FFTC control header and configure the descriptor
     * as much as possible here.
     */
    /* Get a Tx free descriptor */
    if ((pCppiDesc = Qmss_queuePop (pTxObjInfo->hQmssTxFreeQ)) == NULL)
    {
        Fftc_osalLog ("Out of descriptors!! Cannot get a free Tx descriptor \n");
		return FFTC_RETVAL_ENO_MEM;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor size, always mask off the last 
     * 4 bits of the address.
     */
    pCppiDesc = (void*) (QMSS_DESC_PTR (pCppiDesc));    
    descType  = Cppi_getDescType (pCppiDesc);

    /* Initialize data offset for monolithic descriptors. */
    if (descType == Cppi_DescType_MONOLITHIC)
    {
        Cppi_setDataOffset (descType, pCppiDesc, FFTC_CPPI_MONOLITHIC_DESC_SIZE);
    }

    /* Return the original data buffer pointer, its length and the offset at
     * which the application can start filling in PS Info/payload for FFTC engine.
     */
    Cppi_getData (descType, pCppiDesc, &pOrigDataBuffer, &origDataBufferLen);

    /* Setup FFTC control header if:
     *      (1) a valid FFTC queue configuration specified 
     *      (2) mixed size DFT list configuration specified 
     *      (3) PS Info specified to send along with data in this packet.
     */
    if (pFFTCQConfig || !pDFTBlockSizeInfo->bIsEqualSize || psInfoLen)
    {
        /* Enable Bit 0 of the PS flags to indicate presence of control header. */
        psFlags                                 =   (1 << 0);         
        Cppi_setPSFlags (descType, pCppiDesc, psFlags);  

        /* Start calculating the data buffer offset, i.e., offset from start 
         * of the buffer at which FFT request data will be put.
         *
         * FFTC Control header (4 bytes) will be added to SOP of the buffer, hence
         * initialize the data buffer offset to include the header size.
         */
        dataBufferOffset    =  4;           

        /* Build the control header as per the configuration specified. */

        /* Convert the PS data length from bytes to number of words */
        fftcCtrlHdrObj.psFieldLen               =   (psInfoLen / 4);          
        if (psInfoLen)        
        {
            /* Enable PS Pass through data present bit */                           
            fftcCtrlHdrObj.bPSPassThruPresent   =   1;  
        }
        else
            fftcCtrlHdrObj.bPSPassThruPresent   =   0;  

        if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
        {
            /* Enable DFT size list if a valid DFT size list length passed and if the DFT list is a mixed block size */                
            fftcCtrlHdrObj.bDFTSizeListPresent  =   1;  

            /* Convert the DFT size list length from bytes to number of words */
            if (pDFTBlockSizeInfo->numBlocks % 5)
                fftcCtrlHdrObj.dftSizeListLen   =   ((pDFTBlockSizeInfo->numBlocks / 5) + 1);             
            else
                fftcCtrlHdrObj.dftSizeListLen   =   (pDFTBlockSizeInfo->numBlocks / 5);             
            dataBufferOffset                    +=  (fftcCtrlHdrObj.dftSizeListLen * 4);
        }
        else
        {
            fftcCtrlHdrObj.bDFTSizeListPresent  =   0;
            
            /* Convert the DFT size list length from bytes to number of words */
            fftcCtrlHdrObj.dftSizeListLen       =   0;
        }             
              
        if (pFFTCQConfig)
        {
            /* Enable the FFT queue configuration present bit */                
            fftcCtrlHdrObj.bLocalConfigPresent  =   1; 

            /* Reserve space for FFTC queue configuration.
             *
             * There are 5 FFTC Queue local configuration registers and
             * each is 4 bytes wide, hence hold space for 5 * 4 bytes.
             */            
            dataBufferOffset                    +=  (5 * 4);  
        }
        else
            fftcCtrlHdrObj.bLocalConfigPresent  =   0; 

        /* Invalidate data buffer memory (if cached) to ensure we are 
         * not reading stale data.
         */
        Fftc_osalBeginDataBufMemAccess (pOrigDataBuffer, origDataBufferLen);

        /* Add the FFTC control header to the buffer */
        tmpLen = 0;
        if (Fftc_createControlHeader (&fftcCtrlHdrObj, pOrigDataBuffer, &tmpLen) != 0)
        {
            Fftc_osalLog("FFTC Control Header creation failed \n");
            goto return_error;
        }

        /* Increment pointer to move over the control header  */
        pOrigDataBuffer += tmpLen;  

        /* Add in the FFT Queue configuration parameters */
        tmpLen = 0;
        if (pFFTCQConfig)
        {
            /* If a DFT size list is being specified, overwrite the DFT Size field
             * of the Control register to '0x3F' (63) to indicate to the hardware
             * to use DFT size list specified in the packet.
             */
            if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
            {
                /* Use DFT size list to process blocks. */                    
                pFFTCQConfig->controlRegConfig.dftSize      =   FFTC_DEF_DFT_SIZE;            
            }
                
            if (Fftc_compileQueueLocalConfigParams (pFFTCQConfig, (uint8_t *) pOrigDataBuffer, &tmpLen) != 0)
            {
                Fftc_osalLog ("FFTC Queue local configuration creation failed \n");
                goto return_error;
            }

            /* Increment pointer to move over the FFTC configuration  */
            pOrigDataBuffer += tmpLen;      
        }

        /* Add in the DFT size list configuration */
        tmpLen = 0;
        if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
        {
            if ((Fftc_createDftSizeList (pDFTBlockSizeInfo->blockSizes, pDFTBlockSizeInfo->numBlocks, 
                                        pOrigDataBuffer, &tmpLen)) != 0)                
            {
                Fftc_osalLog ("FFTC DFT Size list creation failed \n");
                goto return_error;
            }

            /* Increment pointer to move over the DFT Size List  */            
            pOrigDataBuffer += tmpLen;      
        }

        /* Write back the contents of updated data buffer (if cached) */
        Fftc_osalEndDataBufMemAccess (pOrigDataBuffer, origDataBufferLen);

        /* PS is always added to SOP of Buffer on Tx Side. */
        if (descType == Cppi_DescType_HOST)
        {
            /* Configure PS location in BD to indicate that PS is in the data itself. */
            Cppi_setPSLocation (descType, pCppiDesc, Cppi_PSLoc_PS_IN_SOP);
        }

    	/* Set PS words length in the descriptor */
    	Cppi_setPSLen (descType, pCppiDesc, psInfoLen + dataBufferOffset);
    	        
    	if (descType == Cppi_DescType_MONOLITHIC)
    	{
        	/* Configure data offset for monolithic descriptors. */
        	Cppi_setDataOffset (descType, pCppiDesc, FFTC_CPPI_MONOLITHIC_DESC_SIZE + psInfoLen + dataBufferOffset);
    	}    	        
    }

    /* Populate the tag info as follows:
     *
     * Source Tag (15:8)    -   Not used by CDMA. 
     *
     * Source Tag (7:0)     -   Set to Rx flow Id that should receive the result corresponding
     *                          to this request.
     *
     * Destn Tag (15:8)     -   Set this to the request Id higher order 8 bits passed by the
     *                          application.
     *
     * Destn Tag (7:0)      -   Set this to the request Id lower order 8 bits passed by the
     *                          application.
     */
    cppiTagCfg.srcTagHi     =   0;
    cppiTagCfg.srcTagLo     =   (destnFlowId & 0x00FF);
    cppiTagCfg.destTagHi    =   ((destnTagInfo & 0xFF00) >> 8) ;
    cppiTagCfg.destTagLo    =   (destnTagInfo & 0x00FF);
    Cppi_setTag (descType, pCppiDesc, &cppiTagCfg);       

    /* Done adding the headers and doing the pre-work.
     *
     * Return the request data buffer handle and available data buffer 
     * length to the user.
     *
     * Factor in the header length so that applcn doesnt exceed its space.
     */
    *pMaxDataBufferLen      =   (origDataBufferLen - dataBufferOffset);     
    *ppReqDataBuffer        =   pOrigDataBuffer;

    /* No Request settings saved. Just return the descriptor handle */
    *phFFTCRequest          =   (Fftc_RequestHandle) pCppiDesc;

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;

return_error:
    /* Error processing the configuration provided.
     *
     * Restore the descriptor back to Tx free queue
     * and return error.
     */
    if (pTxObjInfo->descSize > 256)
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxFreeQ, pCppiDesc, 256);    
    else
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxFreeQ, pCppiDesc, pTxObjInfo->descSize);    

    return FFTC_RETVAL_EFAILURE;
}


/**
* ============================================================================
*  @n@b Fftc_txSubmitRequest
*
*  @b   brief
*  @n   Given a Tx object handle, the request handle obtained from 
*       @a Fftc_txGetRequestBuffer() API, and the FFT request data buffer length, 
*       this API submits the FFT request to the hardware to do its processing. 
*       This API assumes that the data buffer length provided to this API doesnt 
*       exceed the maximum length returned by the 
*       @a Fftc_txGetRequestBuffer() API. 
*
*       Alternatively, this API can be used also to submit Tx descriptors
*       built by the application. 
*
*  @param[in]
*       hFFTCTxInfo         Tx handle obtained using @a Fftc_txOpen () API.
*
*  @param[in]
*       hRequestInfo        FFTC request handle obtained from @a Fftc_txGetRequestBuffer () 
*                           API or Tx CPPI descriptor built by the application containing 
*                           request data.
*
*  @param[in]
*       reqBufferLen        FFT Request data length. 
*                           If data buffer was obtained using @a  Fftc_txGetRequestBuffer () API, 
*                           this must contain the actual length of buffer used not including 
*                           the PS info length if PS in SOP. This and PS info length put together 
*                           must not exceed the maximum buffer length obtained from 
*                           @a  Fftc_txGetRequestBuffer () API. In case of application
*                           managed descriptor, i.e., if descriptor was not built using
*                           @a  Fftc_txGetRequestBuffer () API and built by application, 
*                           then this must contain the descriptor length.
*
*  @return      Fftc_RetVal
*  @li          FFTC_RETVAL_EBAD_HANDLE     -   Invalid input handles provided. FFTC Submit 
*                                               Request failed.
*  @li          FFTC_RETVAL_SUCCESS         -   Successfully submitted the request.
*
*  @pre
*  @n   A valid Tx object handle must be obtained using @a Fftc_txOpen (), and a 
*       valid request buffer handle obtained using 
*       @a Fftc_txGetRequestBuffer () API or any valid Tx CPPI descriptor handle
*       must be sent to this API.
*
*  @post
*  @n   FFTC Request submitted.
*
* ============================================================================
*/
Fftc_RetVal Fftc_txSubmitRequest 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_RequestHandle          hRequestInfo,
    uint32_t                    reqBufferLen
)
{
    Fftc_TxInfo*                pTxObjInfo;     
    Cppi_Desc*                  pCppiDesc;
    uint32_t                    descSize;

#ifdef FFTC_DRV_DEBUG
    /* Validate input handles */
    if (!hRequestInfo)            
    {
        Fftc_osalLog("Invalid FFT request handle provided. \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }

    if (!hFFTCTxInfo)
    {
        Fftc_osalLog ("Invalid Tx object handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    pTxObjInfo          =   (Fftc_TxInfo *) hFFTCTxInfo;
    pCppiDesc           =   (Cppi_Desc *) hRequestInfo;

    if (pTxObjInfo->bManageReqBuffers)
    {
        /* Packet Len       =   Actual Data Length 
         *
         * (should not include PS length even if the PS Info is in SOP Buffer)
         */
        Cppi_setPacketLen (Cppi_getDescType (pCppiDesc), pCppiDesc, reqBufferLen);

        /* Writeback descriptor to ensure our updates to the descriptor 
         * reflect in the actual physical memory.
         */
        Fftc_osalEndDescMemAccess (pCppiDesc, pTxObjInfo->descSize);

        descSize    =   pTxObjInfo->descSize; 
    }
    else
    {
        descSize    =   reqBufferLen; 
    }
    
    /* Push descriptor onto FFTC Tx queue */
    if (descSize > 256)
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxQ, pCppiDesc, 256);    
    else
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxQ, pCppiDesc, descSize);    

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Fftc_txFreeRequestBuffer
*
*  @b   brief
*  @n   This API frees a request buffer obtained earlier by the application using 
*       @a Fftc_txGetRequestBuffer () API. This API must be called if a request
*       submission failed, i.e., @a Fftc_txSubmitRequest () API returned an 
*       error. This API discards the request settings and restores the descriptor,
*       and buffer back to the Tx objects's Free descriptor queue for a future use.
*
*  @param[in]
*       hFFTCTxInfo         Tx object handle obtained using Fftc_txOpen () API.
*
*  @param[in]
*       hRequestInfo        Request handle obtained using @a Fftc_txSubmitRequest ()
*                           API. Contains all the request settings and descriptor
*                           info.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE      -   Bad input Handle provided.
*  @li         FFTC_RETVAL_SUCCESS          -   Request handle and descriptor freed
*                                               successfully.
*
*  @pre
*  @n   A valid Tx object handle must be obtained using @a Fftc_txOpen () 
*       before calling this API. This API can be called to free a request buffer 
*       obtained earlier using @a Fftc_txGetRequestBuffer () API.
* ============================================================================
*/
Fftc_RetVal Fftc_txFreeRequestBuffer 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_RequestHandle          hRequestInfo
)
{
    Fftc_TxInfo*                pTxObjInfo;
    Cppi_Desc*					pCppiDesc;

#ifdef FFTC_DRV_DEBUG
    /* Validate Tx handle specified. */
    if (!hFFTCTxInfo)
    {
        Fftc_osalLog ("Invalid Tx handle passed \n");
        return FFTC_RETVAL_EBAD_HANDLE;            
    }

    if (!hRequestInfo)            
    {
        Fftc_osalLog("Invalid FFT request handle provided. \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    pTxObjInfo          =   (Fftc_TxInfo *) hFFTCTxInfo;

#ifdef FFTC_DRV_DEBUG
    /* Validate input parameters */
    if (!pTxObjInfo->hQmssTxFreeQ)
    {
        Fftc_osalLog ("Tx Free Queue Not managed by driver for this Tx object. Cannot use this API \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    /* Get the descriptor */
    pCppiDesc           =   (Cppi_Desc *) hRequestInfo;

    /* Return descriptor back to Tx free queue */
    if (pTxObjInfo->descSize > 256)
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxFreeQ, pCppiDesc, 256);    
    else
        Qmss_queuePushDescSize (pTxObjInfo->hQmssTxFreeQ, pCppiDesc, pTxObjInfo->descSize);    

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Fftc_rxProcessDesc
*
*  @b   brief
*  @n   This API can be used to retrieve the various result information from
*       the result descriptor received from the engine.
*
*  @param[in]
*       hFFTCRxInfo         Rx handle obtained using @a Fftc_rxOpen () API.
*
*  @param[in]
*       pCppiDesc           Rx descriptor containing the result that needs to be
*                           processed by this API.
*
*  @param[out]
*       phResultInfo        Pointer to FFTC result handle containing Rx descriptor
*                           handle.
*
*  @param[out]
*       ppResultBuffer      Pointer to hold result data buffer pointer obtained from 
*                           FFTC engine.
*
*  @param[out]
*       pResultBufferLen    Pointer to result buffer length, i.e.,holds the number of
*                           bytes available to application in the result data buffer for
*                           parsing.
*
*  @param[out]
*       ppPSInfo            Pointer to PS Info pointer obtained from FFTC engine result.
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
*
*  @param[out]
*       pDestnTagInfo       Destination tag info read from the descriptor.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE      -   Bad input Handle provided.
*  @li         FFTC_RETVAL_SUCCESS          -   Success.
*
*  @pre
*  @n   A valid CPPI descriptor handle must be sent to this API.
* ============================================================================
*/
 Fftc_RetVal Fftc_rxProcessDesc 
(
    Fftc_RxHandle               hFFTCRxInfo,
    Cppi_Desc*                  pCppiDesc,
    Fftc_ResultHandle*          phResultInfo,
    uint8_t**                   ppResultBuffer,
    uint32_t*                   pResultBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
)
{
    Cppi_DescTag                cppiTagInfo;
    Cppi_DescType               descType;
    Fftc_RxInfo*                pRxObjInfo;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!hFFTCRxInfo || !phResultInfo || !ppResultBuffer || !pResultBufferLen 
        || !ppPSInfo || !pPSInfoLen || !pFlowId || !pDestnTagInfo
       )
    {
        Fftc_osalLog ("Invalid input Handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo          =   hFFTCRxInfo;

    /* Invalidate descriptor to ensure our reads/writes are consistent with 
     * the memory in case descriptor is in cacheable memory.
     */
    Fftc_osalBeginDescMemAccess (pRxObjInfo, pCppiDesc, pRxObjInfo->descSize);
    
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

    /* Store the result descriptor in the result handle.
     *
     * Required when freeing the result to restore it back to Rx FDQ
     */
    *phResultInfo       =   (Fftc_ResultHandle) pCppiDesc;

    /* Get Data buffer containing the result and its length */
    Cppi_getData (descType, pCppiDesc, ppResultBuffer, pResultBufferLen);

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
        *ppResultBuffer +=  *pPSInfoLen;                      
    }

    return FFTC_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Fftc_rxGetResult
*
*  @b   brief
*  @n   Given an Rx object handle, this API checks if there are any FFTC results
*       available from the engine for its processing. If so, it does some basic 
*       validation on the result and returns a result handle, the FFT/IFFT raw 
*       result data buffer, its length, and any PS info found to the calling 
*       application. If no result found, this API either returns status to indicate 
*       the same or blocks on the result waiting for it to arrive depending on
*       the Rx object's interrupt and Rx block mode configuration.
*
*  @param[in]
*       hFFTCRxInfo         Rx handle obtained using @a Fftc_rxOpen () API.
*
*  @param[out]
*       phResultInfo        Pointer to FFTC result handle containing info that driver
*                           uses internally to process the result.
*
*  @param[out]
*       ppResultBuffer      Pointer to hold result data buffer pointer obtained from 
*                           FFTC engine.
*
*  @param[out]
*       pResultBufferLen    Pointer to result buffer length, i.e.,holds the number of
*                           bytes available to application in the result data buffer for
*                           parsing.
*
*  @param[out]
*       ppPSInfo            Pointer to PS Info pointer obtained from FFTC engine result.
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
*  @return      Fftc_RetVal
*  @li          FFTC_RETVAL_EBAD_HANDLE         -   Invalid input Rx object handle provided.
*  @li          FFTC_RETVAL_ENO_RESULT          -   No FFT results available yet on the Rx queue.
*  @li          FFTC_RETVAL_EFAILURE            -   Error. Cant process/save the result received.
*  @li          FFTC_RETVAL_SUCCESS             -   Successfully retrieved result and stored in 
*                                                   output parameter handles.
*
*  @pre
*  @n   A valid Rx object handle must be obtained using @a Fftc_rxOpen () API 
*       before calling this API.
*
*  @post
*  @n   If FFT/IFFT Result available, it is validated and handed over to application
*       for further processing, otherwise appropriate status is returned.
* ============================================================================
*/
Fftc_RetVal Fftc_rxGetResult
(
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle*          phResultInfo,
    uint8_t**                   ppResultBuffer,
    uint32_t*                   pResultBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
)
{
    Fftc_RxInfo*                pRxObjInfo;
    Cppi_Desc*                  pCppiDesc;
    Fftc_RequestInfo            *pResultInfo;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!hFFTCRxInfo || !phResultInfo || !ppResultBuffer || !pResultBufferLen 
        || !ppPSInfo || !pPSInfoLen || !pFlowId || !pDestnTagInfo
       )
    {
        Fftc_osalLog ("Invalid input Handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo      =   (Fftc_RxInfo *) hFFTCRxInfo;

    /* Check if interrupts were enabled for this Rx object. Depending
     * on interrupt configuration, we'd have to either get the results
     * from the Rx queue itself or the interrupt result queue (software list) of this
     * object.
     *
     * If Interrupts were enabled, all received packets would have been 
     * already dequeued by the accumulator, serviced by ISR and enqueued
     * to this Rx object's result queue.
     *
     * If interrupts were not setup for this Rx object, then we'd
     * have to go ahead and dequeue the packets now explicitly from
     * the Rx queue.
     */
    if (!pRxObjInfo->cfg.bUseInterrupts)
    {
        /* No interrupts enabled. 
         *
         * Check the Rx object's destination queue to see if a
         * result has arrived from the FFTC engine or not.
         */
        if ((pCppiDesc = Qmss_queuePop (pRxObjInfo->hQmssRxQ)) == NULL)
        {
            /* No FFT results ready yet. */            
            return FFTC_RETVAL_ENO_RESULT;                
        }

        /* Get the result info from the descriptor */
        Fftc_rxProcessDesc (hFFTCRxInfo,
                            pCppiDesc, 
                            phResultInfo,
                            ppResultBuffer,
                            pResultBufferLen,
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
         * Get results from the resultQueue in Rx object.
         */            
        if (pRxObjInfo->bUsesDrvAccumList)
        {
            /* Interrupt configuration setup on a per Rx object basis.
             *
             * Get the results from the 'resultQueue' a FIFO software list
             * maintained by the driver to hold all the results received for
             * this Rx object from the accumulator.
             */
            Fftc_osalInterruptCsEnter ();
            while (!(pResultInfo = (Fftc_RequestInfo *) Fftc_listRemove ((Fftc_ListNode **)&pRxObjInfo->resultQueue)))
            {
                /* No FFT results ready yet. */            
                Fftc_osalInterruptCsExit ();
           
                /* Check if this Rx object was configured to wait/block until
                 * a FFT result arrives for it to service. If not, just return 
                 * an error indicating that there are no results yet. Otherwise,
                 * grab the semaphore on this Rx object and block until result
                 * arrives through accumulator.
                 */
                if (!pRxObjInfo->cfg.bBlockOnResult)
                {
                    /* Non-blocking Rx object. Return status */                    
                    return FFTC_RETVAL_ENO_RESULT;                
                }
                else
                {
                    /* Blocking Rx object. Wait on this object until a result arrives. */                    
                    Fftc_osalPendSem (pRxObjInfo->hResultSem);                  
                }
            }
			/* Decrement the number of pending results for this flow. */
            pRxObjInfo->resultQueueLen--;

            /* Get the descriptor out and restore the result buffer back for use by ISR */
            pCppiDesc = pResultInfo->pCppiDesc;
            Fftc_listCat ((Fftc_ListNode **)&pRxObjInfo->freeResultQueue, (Fftc_ListNode **)&pResultInfo);

            Fftc_osalInterruptCsExit ();
            
            /* Get the result info from the descriptor */
            Fftc_rxProcessDesc (hFFTCRxInfo,
                                pCppiDesc, 
                                phResultInfo,
                                ppResultBuffer,
                                pResultBufferLen,
                                ppPSInfo,
                                pPSInfoLen,
                                pFlowId,
                                pSrcId,
                                pDestnTagInfo);
        }
        else
        {
            /* Driver is not setup to manage the ISR for this Rx object. 
             * Cannot use this API.
             */
            return FFTC_RETVAL_EBAD_HANDLE;
        }
    }

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;                
}

/**
* ============================================================================
*  @n@b Fftc_rxGetNumPendingResults
*
*  @b   brief
*  @n   Given a Rx object handle, this API retrieves the number of FFT results 
*       pending to be serviced by the application for it.
*
*  @param[in]
*       hFFTCRxInfo         Rx handle obtained using @a Fftc_rxOpen () API.
*
*  @return     int32_t
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Invalid input Rx handle provided.
*  @li          >=0                     -   Number of pending results for this 
*                                           Rx object.  
*
*  @pre
*  @n   A valid Rx object handle must be obtained using @a Fftc_rxOpen () 
*       API before calling this API.
*
*  @post
*  @n   Returns the number of FFT results available for this Rx object.
* ============================================================================
*/
Fftc_RetVal Fftc_rxGetNumPendingResults 
(
    Fftc_RxHandle               hFFTCRxInfo
)
{    
    Fftc_RxInfo*                pRxObjInfo;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!hFFTCRxInfo)
    {
        Fftc_osalLog ("Invalid Rx object Handle provided \n");
        return FFTC_RETVAL_EBAD_HANDLE;
    }
#endif

    pRxObjInfo      =   (Fftc_RxInfo *)hFFTCRxInfo;        

    if (pRxObjInfo->cfg.bUseInterrupts)
    {
        if (pRxObjInfo->bUsesDrvAccumList)
        {
            /* Interrupts managed by driver. Check the result queue. */            
            return pRxObjInfo->resultQueueLen;            
        }
        else
        {
            /* ISR not managed by driver. So no idea!
             */            
            return FFTC_RETVAL_EBAD_HANDLE;            
        }
    }
    else
    {
        /* Check the destination queue for the number of result packets to be serviced. */
        return (Qmss_getQueueEntryCount (pRxObjInfo->hQmssRxQ));
    }
}

/**
* ============================================================================
*  @n@b Fftc_rxFreeResult
*
*  @b   brief
*  @n   Given a Rx object handle and result handle obtained from @a Fftc_rxGetResult ()
*       API, this API frees the result handle and restores the associated Rx descriptor
*       back to its Rx Free descriptor queue. This API must be called by the 
*       application once its done processing the result, otherwise
*       the driver could run out of result buffers eventually if none restored
*       in time. 
*
*  @param[in]
*       hFFTCRxInfo         Rx handle obtained using @a Fftc_rxOpen () API.
*
*  @param[in]
*       hResultInfo         FFTC result handle obtained using @a Fftc_rxGetResult ()
*                           API.
*
*  @return     Fftc_RetVal
*  @li         FFTC_RETVAL_EBAD_HANDLE  -   Invalid input provided. Free failed.
*  @li         FFTC_RETVAL_SUCCESS      -   Result free succeeded.
*
*  @pre
*  @n   A valid result handle should have been obtained using @a Fftc_rxGetResult () 
*       before calling this API to free the result.
*
*  @post
*  @n   The result buffers passed are freed up and restored to the Rx object's free
*       descriptors.
* ============================================================================
*/
Fftc_RetVal Fftc_rxFreeResult 
(
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle           hResultInfo
)
{
    Qmss_Queue                  rxReturnQInfo;
    Cppi_Desc*                  pCppiDesc;
    Qmss_QueueHnd               qHandle;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!hFFTCRxInfo || !hResultInfo)
        return FFTC_RETVAL_EBAD_HANDLE;
#endif

    pCppiDesc       =   (Cppi_Desc*) hResultInfo;

    rxReturnQInfo = Cppi_getReturnQueue (Cppi_getDescType (pCppiDesc), pCppiDesc);
    
    qHandle = Qmss_getQueueHandle(rxReturnQInfo);

    /* Push descriptor back to free queue */
	Qmss_queuePushDescSize (qHandle, pCppiDesc, QMSS_DESC_SIZE (pCppiDesc));

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;
}

/**
* ============================================================================
*  @n@b Fftc_rxParseResult
*
*  @b   brief
*  @n   Given a FFT result handle obtained from @a Fftc_rxGetResult () API or
*       a valid CPPI descriptor received from FFTC engine containing result,  
*       the Rx object handle, and associated request settings, this API parses 
*       the result buffer associated with result handle and compiles a per 
*       block result buffer, error status and 
*       other relevant information for easy interpretation by the application.
*
*  @param[in]
*       hFFTCRxInfo         Rx handle obtained using @a Fftc_rxOpen () API.
*
*  @param[in]
*       hResultInfo         FFTC result handle obtained using @a Fftc_rxGetResult ()
*                           API or valid CPPI descriptor handle containing FFT
*  @param[in]
*       pResultBuffer       Result data buffer pointer obtained from FFTC engine.
*
*  @param[in]
*       resultBufferLen     Result buffer length.
*
*  @param[in]
*       pDFTBlockSizeInfo   DFT block size list configuration structure. Specifies 
*                           the number of DFT blocks in the FFTC request, size of
*                           each DFT block and whether all blocks are of the same
*                           size/not.
*
*  @param[in]
*       bSupressSideInfo    Indicates if side band information should be read from
*                           the result.
*
*  @param[in]
*       cyclicPrefixAddNum  Cyclic Prefix addition length value if cyclic prefix
*                           addition was requested on Tx.
*
*   @param[out]                           
*       pFFTResult          Output result structure that will be filled in by this 
*                           API on successful parsing of the result.
*
*  @return      Fftc_RetVal
*  @li          FFTC_RETVAL_EBAD_HANDLE -   Bad Result/Rx object handles provided.
*  @li          FFTC_RETVAL_EFAILURE    -   Result parse failed. The result buffer length 
*                                           obtained is not same as what is expected.
*  @li          FFTC_RETVAL_EINV_DESTID_RESULT  -   Invalid result. Flow Id in result doesnt 
*                                                   match the Rx object's flow Id.
*  @li          FFTC_RETVAL_SUCCESS     -   Successfully parsed the result and filled it in 
*                                           output param structure 'pFFTResult'.
*
*  @pre
*  @n   A valid result obtained using @a Fftc_rxGetResult () API or a valid result 
*       descriptor obtained by the application from its ISR must be passed to this API.
*
*       Appropriate request settings used on Tx must be provided here to this API
*       for it to parse the result correctly.
*
*       The output param pointer 'pFFTResult' should be a valid pointer and must have 
*       been allocated memory by the application before calling this API.
*
*  @post
*  @n   The output param structure 'pFFTResult' is filled with the formatted result
*       and error status.
* ============================================================================
*/
Fftc_RetVal Fftc_rxParseResult
(    
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle           hResultInfo,
    uint8_t*                    pResultBuffer,
    uint32_t                    resultBufferLen,
    Fftc_BlockInfo*             pDFTBlockSizeInfo,
    uint8_t                     bSupressSideInfo,
    uint16_t                    cyclicPrefixAddNum, 
    Fftc_Result*                pFFTResult
)
{
    int32_t                     i, blockSize, tmpLen, errorFlags;
    Cppi_Desc*                  pCppiDesc;
    Cppi_DescTag                cppiTagInfo;
    Cppi_DescType               descType;
    Fftc_RxInfo*                pRxObjInfo;

#ifdef FFTC_DRV_DEBUG
    /* Invalid input handles provided */        
    if (!hResultInfo || !hFFTCRxInfo || !pFFTResult || !pResultBuffer || !pDFTBlockSizeInfo)        
        return FFTC_RETVAL_EBAD_HANDLE;            
#endif

    pRxObjInfo      =   (Fftc_RxInfo *) hFFTCRxInfo;
    pCppiDesc       =   (Cppi_Desc *) hResultInfo;
    pCppiDesc       =   (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
    descType        =   Cppi_getDescType (pCppiDesc);

#ifdef FFTC_DRV_DEBUG
    /* Check if valid DFT block size information is passed. */
    if (!pDFTBlockSizeInfo->numBlocks)
    {
        Fftc_osalLog ("Error! Cannot parse result. Invalid DFT block size info passed! \n");            
        return FFTC_RETVAL_EFAILURE;                    
    }
#endif

    /* Get the number of FFTC blocks */
    pFFTResult->numFFTCBlocks   =   pDFTBlockSizeInfo->numBlocks;    

    /* Initialize our running result buffer length */
    tmpLen = 0;

    /* Calculate the expected result buffer length and validate
     * against the length passed to ensure there is no error.
     */
    for (i = 0; i < pDFTBlockSizeInfo->numBlocks; i ++)
    {
    	if (!bSupressSideInfo)
        {
            /* Account for side band info (4 words) consisting of:
             *      -   Block exponent
             *      -   Destination Tag
             *      -   Flow Id
             *      -   Src Id
             */
        	tmpLen  +=  (4  * 4);            
        }

        if (pDFTBlockSizeInfo->bIsEqualSize)            
            tmpLen  +=  pDFTBlockSizeInfo->blockSizes[0] * 4;    /* each sample is 4 bytes wide */
        else 
            tmpLen  +=  pDFTBlockSizeInfo->blockSizes[i] * 4;                                           

        tmpLen      +=  cyclicPrefixAddNum * 4;
    }

    /* The size of result buffer doesnt match the expected length */
    if (tmpLen != resultBufferLen)
    {
        Fftc_osalLog ("Invalid result length, expected: %d received: %d \n", tmpLen, resultBufferLen);            
        pRxObjInfo->rxBadLength ++;
        return FFTC_RETVAL_EFAILURE;            
    }

    /* Invalidate data buffer (in case its cached) to ensure what we are
     * reading is in fact consistent with physical memory.
     */
    Fftc_osalBeginDataBufMemAccess (pResultBuffer, resultBufferLen);
                
    /* Break up the result into per-block result info */
    for (i = 0; i < pDFTBlockSizeInfo->numBlocks; i ++)
    {
        if (!bSupressSideInfo)
        {
            if (i == 0)                     
            {
                /* Block 0 has source and destination tag info additionally to retrieve */               
                pFFTResult->srcId               =   (*(uint32_t *)pResultBuffer & 0x000000FF);
                pResultBuffer                   +=  4; 
                pFFTResult->flowId              =   (*(uint32_t *)pResultBuffer & 0x000000FF);
                pResultBuffer                   +=  4; 
                pFFTResult->destnTagInfo        =   (*(uint32_t *)pResultBuffer & 0x0000FFFF);
                pResultBuffer                   +=  4; 

                /* Read error info from descriptor */
                errorFlags                      =   Cppi_getDescError (descType, pCppiDesc);
                pFFTResult->bErrorDetected      =   (errorFlags & 0x00000001);
                pFFTResult->bClippingDetected   =   ((errorFlags >> 2) & 0x00000001);

                /* Validate the received packet based on flow id */
                if (pRxObjInfo->cppiFlowId != pFFTResult->flowId)
                {
                    Fftc_osalLog ("Error! Received a packet not destined for this object \n");                
                    pRxObjInfo->rxBadDestn ++;
                    return FFTC_RETVAL_EFAILURE;
                }
            }
            else
            {
				/* All blocks other than Block 0 have no useful
				 * information in srcId, flowId and destination Tag fields
				 * of the buffer.
				 */
            	pResultBuffer                   +=  (3 * 4);             	
            }

            /* Get the "Block Exponent" fields for this block. */
            pFFTResult->fftBlockResult[i].blockExponentVal  =  (*(uint32_t *)pResultBuffer & 0x0000FFFF);

            /* skip over to the result now */
            pResultBuffer                       +=  4; 
        }
        else
        {
            if (i == 0)
            {
                /* When side info is supressed, the srcId, flowId and destTag info
                 * would have to be extracted from descriptor instead of data buffer
                 */            
                cppiTagInfo                     =   Cppi_getTag (descType, pCppiDesc);
                pFFTResult->srcId               =   cppiTagInfo.srcTagHi;
                pFFTResult->flowId              =   cppiTagInfo.srcTagLo;
                pFFTResult->destnTagInfo		=   ((cppiTagInfo.destTagHi << 8) | (cppiTagInfo.destTagLo));                

                pFFTResult->bErrorDetected      =   0;
                pFFTResult->bClippingDetected   =   0;

                /* Validate the received packet based on flow id */
                if (pRxObjInfo->cppiFlowId != pFFTResult->flowId)
                {
                    Fftc_osalLog ("Error! Received a packet not destined for this object, Received: %d Expected: %d \n",
                                  pFFTResult->flowId, pRxObjInfo->cppiFlowId);                
                    pRxObjInfo->rxBadDestn ++;
                    return FFTC_RETVAL_EFAILURE;
                }
            }
        }    

        /* Get the FFT result for this block */
        if (pDFTBlockSizeInfo->bIsEqualSize)
        {
            /* If all the blocks are of the same size, then only one
             * size will be provided.
             */
            blockSize                           =   pDFTBlockSizeInfo->blockSizes[0];                   
        }
        else
        {
            blockSize                           =   pDFTBlockSizeInfo->blockSizes[i];                   
        }

        /* If cyclic prefix addition was enabled for this FFT request,
         * the output buffer size will include the cyclic prefix sample
         * length too.
         */
        blockSize                               += cyclicPrefixAddNum;

        pFFTResult->fftBlockResult[i].pBuffer   =   pResultBuffer;
        pFFTResult->fftBlockResult[i].bufferLen =   blockSize * 4; 

        /* skip over this block's result now */       
        pResultBuffer       +=  (blockSize * 4); 
    }

    /* Return success. */
    return FFTC_RETVAL_SUCCESS;    
}

/**
* ============================================================================
*  @n@b Fftc_rxHiPriorityRxISR
*
*  @b   brief
*  @n   FFTC driver's high priority accumulation Interrupt Service Routine (ISR)
*       for any given Rx object.
*
*       This API must be registered as the ISR handler if the Rx object was setup
*       to use driver managed accumulator list configuration, i.e., In @a Fftc_rxOpen () 
*       API 'bUseInterrupts' to 1 and 'bManageAccumList' was set to 1.
*
*       This API when invoked on an interrupt, reads the High priority accumulator 
*       list registered by the driver and checks if any results are available 
*       for this Rx object. It enqueues the packets received to
*       the Rx object's result queue for processing later by the application.
*       If the Rx object was opened in "blocking" mode, then this API posts
*       on the Rx object's semaphore to notify it of the result received.
*
*  @param[in]
*       hFFTCRxInfo             FFTC Rx object handle
*
*  @return      void
*
*  @pre
*  @n   This function MUST be registered as the ISR handler for the system event id 
*       (lookup EVM specification for system event id) on the core on which the 
*       application wishes to receive results from FFTC driver using interrupts.
*
*  @post
*  @n   Interrupt serviced. Results retrieved from the Rx object's high priority
*       accumulator list. 
* ============================================================================
*/
void Fftc_rxHiPriorityRxISR 
(
    Fftc_RxHandle               hFFTCRxInfo
)
{
    Cppi_Desc*                  pCppiDesc;        
    Fftc_RxInfo*                pRxObjInfo;
    uint8_t                     count, i;
    Fftc_RequestInfo*           pResultInfo;
    Qmss_Queue                  rxReturnQInfo;
    Qmss_QueueHnd               qHandle;
    
#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!hFFTCRxInfo)
        return;
#endif

    pRxObjInfo          =   (Fftc_RxInfo *) hFFTCRxInfo;

#ifdef FFTC_DRV_DEBUG
    /* Check if the accum list is managed by driver or application.
     * This ISR is capable of handling only Accum List allocated 
     * by driver.
     */
    if (!pRxObjInfo->bUsesDrvAccumList)
    {
        /* Error. Driver not managing Accum list for this Rx object. */
        return;
    }
#endif

    /* Process ISR. 
     *
     * Get the number of entries in accumulator list. 
	 * The hardware enqueues data alternatively to Ping/Pong buffer lists in
     * the accumulator. Hence, we need to track which list (Ping/Pong)
     * we serviced the last time and accordingly process the other one
     * this time around.
     */ 
     if (!pRxObjInfo->bPingListUsed)
     {
		/* Serviced Pong list last time. So read off the Ping list now */     	
    	count   =   pRxObjInfo->pHiPrioAccList[0];     	
     }
     else
     {
		/* Serviced Pong list last time. So read off the Ping list now */     	
    	count   =   pRxObjInfo->pHiPrioAccList[pRxObjInfo->intThreshold + 1];     	
     }
    
    /* Process all the Results received 
     *
     * Skip the first entry in the list that contains the 
     * entry count and proceed processing results.
     */
    for (i = 1; i <= count; i ++)
    {
        /* Get the result descriptor.
         *
         * The hardware enqueues data alternatively to Ping/Pong buffer lists in
         * the accumulator. Hence, we need to track which list (Ping/Pong)
         * we serviced the last time and accordingly process the other one
         * this time around.
         */
        if (!pRxObjInfo->bPingListUsed)
        {
            /* Serviced Pong list last time. So read off the Ping list now */                
            pCppiDesc   =   (Cppi_Desc *) pRxObjInfo->pHiPrioAccList [i];
        
            pRxObjInfo->pHiPrioAccList [i] = NULL;
        }
        else
        {
            /* Serviced Ping list last time. So read off the Pong list now 
             *
             * Skip over Ping list length to arrive at Pong list start.
             */                
            pCppiDesc   =   (Cppi_Desc *) pRxObjInfo->pHiPrioAccList [i + pRxObjInfo->intThreshold + 1];

            pRxObjInfo->pHiPrioAccList [i + pRxObjInfo->intThreshold + 1] = NULL;
        }

        /* Store the result received.
         *
         * Get a free result list entry for this flow.
         *
         * We store the result details in the result list entries maintained
         * by the driver as a software queue on a per flow basis called
         * 'resultQueue'. 
         */
        if (!(pResultInfo = (Fftc_RequestInfo *) Fftc_listRemove ((Fftc_ListNode **)&pRxObjInfo->freeResultQueue)))
        {
            pRxObjInfo->rxFull ++;

            /* Invalidate descriptor to ensure our reads/writes are consistent with 
             * the memory in case descriptor is in cacheable memory.
             */
            Fftc_osalBeginDescMemAccess (pRxObjInfo, pCppiDesc, pRxObjInfo->descSize);
    
            /* Cant save result info. Recycle the descriptor and continue processing the next result. */
            rxReturnQInfo = Cppi_getReturnQueue (Cppi_getDescType (pCppiDesc), pCppiDesc);
            qHandle = Qmss_getQueueHandle(rxReturnQInfo);

            /* Push descriptor back to free queue */
	        Qmss_queuePushDescSize (qHandle, pCppiDesc, QMSS_DESC_SIZE (pCppiDesc));

            continue;            
        }

        /* Mark the result available flag for this pending request */
        pResultInfo->pCppiDesc          =   pCppiDesc;

        /* Enqueue the result to the flow's 'resultQueue' and increment its
         * length.
         *
         * 'resultQueue' is a software list maintained by driver to hold all
         * the results arrived for processing by this flow via an accumulator 
         * interrupt.
         */
        Fftc_listCat ((Fftc_ListNode **)&pRxObjInfo->resultQueue, (Fftc_ListNode **)&pResultInfo);
        pRxObjInfo->resultQueueLen  ++;
    }

    /* Wake up any application blocked on this result. */
    if (count && pRxObjInfo->cfg.bBlockOnResult)
    {
        Fftc_osalPostSem (pRxObjInfo->hResultSem);                
    }

    /* Clear the accumulator list and save whether we used Ping/Pong
     * list information for next time around.
     */
    if (!pRxObjInfo->bPingListUsed)
    {
        /* Just processed Ping list */            
        pRxObjInfo->bPingListUsed  =   1;
    }
    else
    {
        /* Just processed Pong list */            
        pRxObjInfo->bPingListUsed  =   0;
    }

    /* Clear INTD */
    Qmss_ackInterrupt (pRxObjInfo->accChannelNum, 1);
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, pRxObjInfo->accChannelNum);

    /* Done processing interrupt. Return */
    return;
}

/**
@}
*/
