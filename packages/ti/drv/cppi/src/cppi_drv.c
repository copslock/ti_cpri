/**
 *   @file  cppi_drv.c
 *
 *   @brief   
 *      This is the CPPI Low Level Driver.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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
 *  \par
*/

/* CPPI Types includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* CPPI LLD includes */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/include/cppi_listlib.h>
#include <ti/drv/cppi/include/cppi_pvt.h>

/* CPPI OSAL layer */
#include <cppi_osal.h>

/* RM LLD includes */
#include <ti/drv/rm/rm_services.h>

/**********************************************************************
 ************************** Globals ***********************************
 **********************************************************************/

/* CPPI object */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (cppiObject, CPPI_MAX_CACHE_ALIGN)
#pragma DATA_SECTION (cppiObject, ".cppi");
#endif
Cppi_Obj     cppiObject;

/* CPPI Local object */
Cppi_LocalObj    cppiLObj = 
{
    NULL,  /* Initialize to NULL */
};

/** @brief Global Variable which describes the CPPI LLD Version Information */
const char   cppiLldVersionStr[] = CPPI_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/**********************************************************************
 ************************** Local Functions ***************************
 **********************************************************************/

/*
 *  @b Description
 *  @n  
 *      This function sends a service (alloc/free) request to RM
 *
 *  @param[in]  rmService
 *      rm service handle
 *  @param[in]  type
 *      Service request type
 *  @param[in]  resName
 *      Pointer to the RM resource name
 *  @param[out]  resNum
 *      Pointer to the resource value to check.  If the input value is
 *      unspecified the resource value returned by RM will be returned 
 *      through this parameter.
 *
 *  @retval
 *      1 - RM approved request.
 *      0 - RM denied request or experienced an error
 */
static int cppiRmService (Rm_ServiceType type, const char *resName, int32_t *resNum,
                          int *isAllocated)
{
    Rm_ServiceHandle   *rmService = (Rm_ServiceHandle *)cppiLObj.cppiRmServiceHandle;
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;  
    int                 retVal = 0;

    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));
    
    rmServiceReq.type = type;
    rmServiceReq.resourceName = resName;
    if (*resNum >= 0)
    {
        rmServiceReq.resourceBase = *resNum;
    }
    else
    {
        rmServiceReq.resourceBase = RM_RESOURCE_BASE_UNSPECIFIED;
    }
    rmServiceReq.resourceLength = 1;
    /* RM will block until resource is returned since callback is NULL */
    rmServiceReq.callback.serviceCallback = NULL;
    rmService->Rm_serviceHandler(rmService->rmHandle, &rmServiceReq, &rmServiceResp);
    if ((rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
        (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC))
    {
        /* Only return value through resNum if the service type is
         * ALLOCATE... */
        if ((type == Rm_service_RESOURCE_ALLOCATE_INIT) ||
            (type == Rm_service_RESOURCE_ALLOCATE_USE))
        {
            *resNum = rmServiceResp.resourceBase;
        }
        retVal = 1;
    }
    
    if (isAllocated)
    {
        if (rmServiceResp.resourceNumOwners == RM_RESOURCE_NUM_OWNERS_INVALID)
        {
            *isAllocated = 0;
        }
        else
        {
            /* QM would like to have the global allocation count, but RM doesn't provide that.
             * Most use cases rely on 0, 1, or 2 (last instance closed, first instance opened,
             * or subsequent instance opened.
             * The following logic will always return 0,1, or 2 correctly, but values >=2
             * only mean that this is not first open request
             */
            if ((rmServiceResp.resourceNumOwners == 1) && 
                (rmServiceResp.instAllocCount != RM_INST_ALLOC_COUNT_INVALID) &&
                (rmServiceResp.instAllocCount >= 1))
            {
                /* If this is only owner, return how many times it was opened on this instance; */
                *isAllocated = rmServiceResp.instAllocCount;
            }
            else
            {
                /* If there are multiple owners or no owners, return the owner count */
                *isAllocated = rmServiceResp.resourceNumOwners;
            }
            /* Error or denial occurred in RM but not total failure since refCount was returned */
            retVal = 1;
        }
    }   
    return (retVal);
}

/**
 *  @b Description
 *  @n  
 *      This function closes the CPPI CPDMA instance. 
 *      The instance reference count is decremented based on whether or not
 *      the reference counts for the Rx, Tx channels and Rx flows are ignored
 *      CPPI CPDMA instance is closed only if the reference count is zero.
 *      CPPI CPDMA instance is closed only if all the Rx, Tx channels and Rx
 *      flows are closed.
 *      This function should be called to close all instances before re-opening
 *      the CPPI CPDMA instance. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *  @param[in]  ignoreRefCnt
 *      choose whether to ignore Rx, Tx channel and Rx flow reference counts
 *      when decrementing instance reference count
 *
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      CPPI CPDMA object is freed if the reference count is zero.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_INITIALIZED
 *  @retval
 *      Failure -   CPPI_TX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_FLOWS_NOT_CLOSED
 */
static Cppi_Result cppiCloseCommon (Cppi_Handle hnd, int ignoreRefCnt)
{
    Cppi_DMAObj     *dmaObjPtr;
    void            *key;
    Cppi_Result      retVal;

    if (hnd == NULL)
    {
        return CPPI_INVALID_PARAM;
    }

    dmaObjPtr = (Cppi_DMAObj *) hnd;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* Invalidate CPDMA Object */
    Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    if (dmaObjPtr->refCnt == 0)
    {
        retVal = CPPI_CPDMA_NOT_INITIALIZED;
        goto end;
    }

    if ((dmaObjPtr->refCnt == 1) || !ignoreRefCnt)
    {
        /* If channels or flows are still open then don't close CPDMA instance */
        if (dmaObjPtr->txChCnt)
        {
            retVal = CPPI_TX_CHANNELS_NOT_CLOSED;
            goto end;
        }
        if (dmaObjPtr->rxChCnt)
        {
            retVal = CPPI_RX_CHANNELS_NOT_CLOSED;
            goto end;
        }
        if (dmaObjPtr->rxFlowCnt)
        {
            retVal = CPPI_RX_FLOWS_NOT_CLOSED;
            goto end;
        }
    }
    dmaObjPtr->refCnt--;

    /* Writeback reference count */
    Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    retVal = dmaObjPtr->refCnt;
    /* Let go of HwOpen resource as long as the HwOpen was not denied */
    if (! cppiLObj.hwOpenDenied[dmaObjPtr->dmaNum])
    {
        if (cppiLObj.cppiRmServiceHandle)
        {
            /* CPDMA HW open resource always uses resource #0 */
            int32_t openRes = 0;

            if (! cppiRmService(Rm_service_RESOURCE_FREE,
                                dmaObjPtr->rmCpdmaHwOpen,
                                &openRes, NULL))
            {
                retVal = CPPI_RM_ERR_FREEING_RESOURCE;
            }
        }
    }

end:
    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return retVal;
}

/** @addtogroup CPPI_LLD_FUNCTION
@{ 
*/

/**********************************************************************
 ************************** APIs **************************************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      This function initializes the CPPI low level driver
 *      This function is called once in the system to setup the CPPI 
 *      low level driver with information pertaining to maximum supported Rx priority, 
 *      Tx priority, Rx channels, Tx channels, Rx flows and memory mapped address for each CPPI CPDMA. 
 *
 *  @param[in]  cppiGblCfgParams
 *      Initialization structure that contains the CPPI device specific information.
 *  @param[in]  initCfg
 *      Initialization structure that contains the heap config information which can be NULL.
 *
 *  @post  
 *      CPPI instance is created.
 *
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *      Success -   CPPI_SOK
 */
Cppi_Result Cppi_initCfg (Cppi_GlobalConfigParams *cppiGblCfgParams, Cppi_InitCfg *initCfg)
{
    Cppi_DMAObj     *dmaObjPtr;
    uint32_t          i;
    void             *key;
    Cppi_HeapParams  *heapParams;
    Cppi_GlobalCPDMAConfigParams *cfgParams;

    /* Zero overhead check that CPPI_BLOCK_CHUNK_SIZE is good */
    CPPI_BLOCK_CHUNK_SIZE_CHECK;
    /* Zero overhead check that CPPI_MAX_CPDMA is consistent */
    CPPI_COMPILE_TIME_SIZE_CHECK (CPPI_MAX_CPDMA == (Cppi_CpDma_LAST+1));

    /* Make sure the enum values match the indicies */
    for (i = 0; i < CPPI_MAX_CPDMA; i++)
    {
       if ((Cppi_CpDma)i != cppiGblCfgParams->cpDmaCfgs[i].dmaNum)
       {
           return CPPI_INVALID_PARAM;
       }
    }

    /* Begin Critical Section before accessing Shared resources */
    key = Cppi_osalCsEnter ();

    /* Invalidate Global Object */
    Cppi_osalBeginMemAccess ((void *) &cppiObject, sizeof (Cppi_Obj));

    memset ((void *) &cppiObject, 0, sizeof (Cppi_Obj));

    for (i = 0; i < CPPI_MAX_CPDMA; i++)
    {
        /* Get the pointer to dmaobject */
        cfgParams = &cppiGblCfgParams->cpDmaCfgs[i];
        dmaObjPtr = &(cppiObject.obj.dmaCfg[cfgParams->dmaNum]);

        /* Store CPDMA related configuration */
        dmaObjPtr->dmaNum = cfgParams->dmaNum;
        dmaObjPtr->maxRxCh = cfgParams->maxRxCh;
        dmaObjPtr->maxTxCh = cfgParams->maxTxCh;
        dmaObjPtr->maxRxFlow = cfgParams->maxRxFlow;
        dmaObjPtr->rxPriority = cfgParams->rxPriority;
        dmaObjPtr->txPriority = cfgParams->txPriority;

        /* Store the register base address */
        dmaObjPtr->gblCfgRegs = cfgParams->gblCfgRegs;
        dmaObjPtr->rxChRegs = cfgParams->rxChRegs;
        dmaObjPtr->txChRegs = cfgParams->txChRegs;
        dmaObjPtr->txSchedRegs = cfgParams->txSchedRegs;
        dmaObjPtr->rxFlowRegs = cfgParams->rxFlowRegs;

        /* Store RM resource names */
        cppi_strncpy(dmaObjPtr->rmCpdmaRxCh, cfgParams->rmCpdmaRxCh, CPPI_RM_RESOURCE_NAME_MAX_CHARS);
        cppi_strncpy(dmaObjPtr->rmCpdmaTxCh, cfgParams->rmCpdmaTxCh, CPPI_RM_RESOURCE_NAME_MAX_CHARS);
        cppi_strncpy(dmaObjPtr->rmCpdmaRxFlow, cfgParams->rmCpdmaRxFlow, CPPI_RM_RESOURCE_NAME_MAX_CHARS);     
        cppi_strncpy(dmaObjPtr->rmCpdmaHwOpen, cfgParams->rmCpdmaHwOpen, CPPI_RM_RESOURCE_NAME_MAX_CHARS);     
    }

    /* Store the qm base addresses  */
    cppiObject.obj.qm0BaseAddress = cppiGblCfgParams->qm0BaseAddress;
    cppiObject.obj.qm1BaseAddress = cppiGblCfgParams->qm1BaseAddress;
    cppiObject.obj.qm2BaseAddress = cppiGblCfgParams->qm2BaseAddress;
    cppiObject.obj.qm3BaseAddress = cppiGblCfgParams->qm3BaseAddress;

    /* Create static memory if provided */
    cppiObject.obj.heapDesc.alignPow2 = CPPI_BLOCK_ALIGN_POW2;
    cppiObject.obj.heapDesc.blockSize = CPPI_BLOCK_SIZE;
    if (initCfg)
    {
        heapParams = &initCfg->heapParams;
        if (heapParams->heapAlignPow2)
        {
            cppiObject.obj.heapDesc.alignPow2 = heapParams->heapAlignPow2;
        }
        if (heapParams->dynamicHeapBlockSize)
        {
            if (heapParams->dynamicHeapBlockSize > 0)
            {
                cppiObject.obj.heapDesc.blockSize = (uint32_t)heapParams->dynamicHeapBlockSize;
            }
            else
            {
                /* Disable dynamic allocation */
                cppiObject.obj.heapDesc.blockSize = 0;
            }
        }
        if (heapParams->staticHeapBase && heapParams->staticHeapSize)
        {
            cppi_internal_heap_add (&cppiObject.obj.heapDesc, heapParams->staticHeapBase, 
                                    heapParams->staticHeapSize);
        }
    }

    /* Writeback Global Object */
    Cppi_osalEndMemAccess ((void *) &cppiObject, sizeof (Cppi_Obj));

    /* Initialize the rmServiceHandle in the local object to NULL */
    cppiLObj.cppiRmServiceHandle = NULL;
    /* Initialize HW open to "not denied" (0) for each CPDMA */
    memset(cppiLObj.hwOpenDenied, 0, sizeof(cppiLObj.hwOpenDenied));

    /* End Critical Section */
    Cppi_osalCsExit (key);
    return CPPI_SOK;
}


/**
 *  @b Description
 *  @n  
 *      This function initializes the CPPI low level driver
 *      This function is called once in the system to setup the CPPI 
 *      low level driver with information pertaining to maximum supported Rx priority, 
 *      Tx priority, Rx channels, Tx channels, Rx flows and memory mapped address for each CPPI CPDMA. 
 *
 *      This works by calling @ref Cppi_initCfg with initCfg = NULL.
 *
 *  @param[in]  cppiGblCfgParams
 *      Initialization structure that contains the CPPI device specific information.
 *
 *  @post  
 *      CPPI instance is created.
 *
 *  @retval
 *      Success -   CPPI_SOK
 */
Cppi_Result Cppi_init (Cppi_GlobalConfigParams *cppiGblCfgParams)
{
    return Cppi_initCfg (cppiGblCfgParams, NULL);
}

/**
 *  @b Description
 *  @n  
 *      This function returns the worst-case heap requirements for the CPPI LLD given
 *      a cppiGblCfgParams set of managed resources.
 *
 *      This can be used to optionally statically allocate all of the heap memory, 
 *      which is then passed in trhough @ref Cppi_initCfg.  As long as the static heap
 *      is at least the size returned by this function then no dynamic allocations
 *      via Cppi_osalMalloc will ever be made.
 *
 *      Use of this API is completely optional.  A static heap can be used of any
 *      size.  If it is too small then dynamic allocation will occur.  The system
 *      can also be used with only dynamic allocation, where no static memory is
 *      provided through Cppi_initCfg.
 *
 *  @param[in]  cppiGblCfgParams
 *      Initialization structure that contains the CPPI device specific information.
 *
 *  @param[out] size
 *      Total bytes of memory that would be used if all flows and channels specified
 *      in cppiGblCfgParams are actually used.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *      Failure -   CPPI_INVALID_PARAM: NULL passsed for cppiGblCfgParmas or size
 */
Cppi_Result Cppi_getHeapReq (Cppi_GlobalConfigParams *cppiGblCfgParams, uint32_t *size)
{
    int i;

    if (cppiGblCfgParams && size)
    {
        uint32_t totalFlows = 0;
        uint32_t totalChannels = 0;

        for (i = 0; i < CPPI_MAX_CPDMA; i++)
        {
            totalFlows += cppiGblCfgParams->cpDmaCfgs[i].maxRxFlow;
            totalChannels += cppiGblCfgParams->cpDmaCfgs[i].maxRxCh +
                             cppiGblCfgParams->cpDmaCfgs[i].maxTxCh;
        }

        *size = CPPI_BLOCK_CHUNK_SIZE * (totalFlows + totalChannels);

        return CPPI_SOK;
    }

    return CPPI_INVALID_PARAM;
}

/**
 *  @b Description
 *  @n  
 *      This function initializes any CPPI start configurations such as the Resource Manager handle
 */
void Cppi_startCfg (Cppi_StartCfg *startCfg)
{
    /* Copy RM service handle into local object */
    cppiLObj.cppiRmServiceHandle = startCfg->rmServiceHandle;

    /* Initialize HW open to "not denied" (0) for each CPDMA */
    memset(cppiLObj.hwOpenDenied, 0, sizeof(cppiLObj.hwOpenDenied));
}

/**
 *  @b Description
 *  @n  
 *      This function deinitializes the CPPI low level driver.
 *      The LLD is deinitialized only if all the CPDMA instances are closed.
 *      This function should be called before re-initializing the CPPI low level driver.
 *
 *  @pre  
 *      Cppi_init function should be called before calling this function.
 *
 *  @post  
 *      CPPI instance is closed.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_CLOSED
 */
Cppi_Result Cppi_exit (void)
{
    Cppi_DMAObj     *dmaObjPtr;
    uint32_t        i;
    void            *key;
    Cppi_Result     retVal;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* Invalidate the global object */
    Cppi_osalBeginMemAccess ((void *) &cppiObject, sizeof (cppiObject));

    for (i = 0; i < CPPI_MAX_CPDMA; i++)
    {
        dmaObjPtr = &(cppiObject.obj.dmaCfg[i]);
        
        if (dmaObjPtr->refCnt > 0)
        {
            retVal = CPPI_CPDMA_NOT_CLOSED;                
            goto end;
        }
    }

    /* Return all allocated memory */
    cppi_internal_heap_release(&cppiObject.obj.heapDesc);

    /* Writeback the global object */
    Cppi_osalEndMemAccess ((void *) &cppiObject, sizeof (cppiObject));

    retVal = CPPI_SOK; 
end:
    /* End Critical Section. */
    Cppi_osalCsExit (key);
    return retVal;
}

/**
 *  @b Description
 *  @n  
 *      This function opens the CPPI CPDMA instance. 
 *      This function is called by the CPPI CPDMA driver, application to initialize the 
 *      global configuration pertaining to CPPI CPDMA that includes rx priority, 
 *      tx priority, write arbitration FIFO depth, receive starvation timeout and QM base addresses. 
 *      The registers are configured only once when the function 
 *      is called for the first time. Any future calls to this function returns the CPDMA 
 *      object handle created during the first call.
 *
 *  @param[in]  initCfg
 *      Initialization structure that contains the global configuration parameters 
 *      pertaining to CPPI CPDMA. 
 *
 *  @param[out] handle
 *      Resulting instance handle.  Will be NULL if hard error occurs (result < 0).  
 *      Will be present for soft error/warning (result > 0)
 *
 *  @pre  
 *      Cppi_init function should be called before calling this function.
 *
 *  @post  
 *      Global configuration registers are configured with input parameters
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   < 0 (see defines based off CPPI_LLD_EBASE).  Value in handle will be NULL.
 *  @retval
 *      Warning -   Register initialiation failed due to RM, but instance still opened 
 */
Cppi_Result Cppi_openStatus (Cppi_CpDmaInitCfg *initCfg, Cppi_Handle *handle)
{
    Cppi_DMAObj     *dmaObjPtr;
    void            *key;
    Cppi_Result      retVal = CPPI_SOK;
    int              writeOK;

    if (handle == NULL)
    {
        return CPPI_INVALID_PARAM;
    }

    *handle = NULL;
    if (initCfg == NULL)
    {
        return CPPI_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    dmaObjPtr = &(cppiObject.obj.dmaCfg[initCfg->dmaNum]);

    /* Invalidate CPDMA Object */
    Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    /* Check if this dmaNum exists on this device */
    if ((dmaObjPtr->maxRxCh == 0) && (dmaObjPtr->maxTxCh == 0))
    {
        dmaObjPtr = NULL;
        retVal = CPPI_ERR_NO_SUCH_DMA;
        goto cppi_exit;
    }

    /* Check with RM if we can poke the registers */
    writeOK = (initCfg->regWriteFlag == Cppi_RegWriteFlag_ON);
    if (cppiLObj.cppiRmServiceHandle)
    {
        /* CPDMA HW open resource always uses resource #0 */
        int32_t openRes = 0;

        if (! cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT,
                            dmaObjPtr->rmCpdmaHwOpen,
                            &openRes, NULL))
        {
            /* Prevent the resource from being freed in Cppi_close */
            cppiLObj.hwOpenDenied[initCfg->dmaNum] = 1;

            /* Disable register write and spit out warning.
             * To avoid warning, set
             * initCfg->regWriteFlag = Cppi_RegWriteFlag_OFF */
            if (writeOK)
            {
                writeOK = 0;
                retVal = CPPI_WARN_OPEN_RM_REGS;
            }
        }
    }

    if (dmaObjPtr->refCnt > 0)
    {
        dmaObjPtr->refCnt++;

        /* Writeback Global Object */
        Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj)); 

        goto cppi_exit;
    }

    if (initCfg->writeFifoDepth > 32)
    {
        dmaObjPtr = NULL;
        retVal = CPPI_INVALID_PARAM;
        goto cppi_exit;
    }
    dmaObjPtr->writeFifoDepth = initCfg->writeFifoDepth;
    dmaObjPtr->timeoutCount = initCfg->timeoutCount;

    /* initialize the DMA obj with the defaults from Cppi_init() */
    dmaObjPtr->qm0BaseAddress = cppiObject.obj.qm0BaseAddress;
    dmaObjPtr->qm1BaseAddress = cppiObject.obj.qm1BaseAddress;
    dmaObjPtr->qm2BaseAddress = cppiObject.obj.qm2BaseAddress;
    dmaObjPtr->qm3BaseAddress = cppiObject.obj.qm3BaseAddress;

    /* Replace the defaults with the initCfg values from Cppi_open() */
    if (initCfg->qm0BaseAddress)
        dmaObjPtr->qm0BaseAddress = initCfg->qm0BaseAddress;

    if (initCfg->qm1BaseAddress)
        dmaObjPtr->qm1BaseAddress = initCfg->qm1BaseAddress;

    if (initCfg->qm2BaseAddress)
        dmaObjPtr->qm2BaseAddress = initCfg->qm2BaseAddress;

    if (initCfg->qm3BaseAddress)
        dmaObjPtr->qm3BaseAddress = initCfg->qm3BaseAddress;

    if (writeOK)
    {
        if (initCfg->writeFifoDepth != 0)
            CSL_FINS (dmaObjPtr->gblCfgRegs->PERF_CONTROL_REG, 
                      CPPIDMA_GLOBAL_CONFIG_PERF_CONTROL_REG_WARB_FIFO_DEPTH, initCfg->writeFifoDepth);

        CSL_FINS (dmaObjPtr->gblCfgRegs->PERF_CONTROL_REG, 
                  CPPIDMA_GLOBAL_CONFIG_PERF_CONTROL_REG_TIMEOUT_CNT, initCfg->timeoutCount);

        CSL_FINS (dmaObjPtr->gblCfgRegs->PRIORITY_CONTROL_REG, 
                  CPPIDMA_GLOBAL_CONFIG_PRIORITY_CONTROL_REG_TX_PRIORITY, dmaObjPtr->txPriority);
        CSL_FINS (dmaObjPtr->gblCfgRegs->PRIORITY_CONTROL_REG, 
                  CPPIDMA_GLOBAL_CONFIG_PRIORITY_CONTROL_REG_RX_PRIORITY, dmaObjPtr->rxPriority);

        /* The precedence for the base addresses is follows:
         * 1) Value specified to Cppi_open initCfg (if nonzero)
         * 2) Value specified to Cppi_init (if nonzero)
         * 3) Value already in registers
         */

        /* if both the initCfg and the default is 0 or !writeOK, then the register is left alone */
        if (dmaObjPtr->qm0BaseAddress != 0)
            CSL_FINS (dmaObjPtr->gblCfgRegs->QM_BASE_ADDRESS_REG[0], 
                      CPPIDMA_GLOBAL_CONFIG_QM_BASE_ADDRESS_REG_QM_BASE, dmaObjPtr->qm0BaseAddress);
        if (dmaObjPtr->qm1BaseAddress != 0)
            CSL_FINS (dmaObjPtr->gblCfgRegs->QM_BASE_ADDRESS_REG[1], 
                      CPPIDMA_GLOBAL_CONFIG_QM_BASE_ADDRESS_REG_QM_BASE, dmaObjPtr->qm1BaseAddress);
        if (dmaObjPtr->qm2BaseAddress != 0)
            CSL_FINS (dmaObjPtr->gblCfgRegs->QM_BASE_ADDRESS_REG[2], 
                      CPPIDMA_GLOBAL_CONFIG_QM_BASE_ADDRESS_REG_QM_BASE, dmaObjPtr->qm2BaseAddress);
        if (dmaObjPtr->qm3BaseAddress != 0)
            CSL_FINS (dmaObjPtr->gblCfgRegs->QM_BASE_ADDRESS_REG[3], 
                      CPPIDMA_GLOBAL_CONFIG_QM_BASE_ADDRESS_REG_QM_BASE, dmaObjPtr->qm3BaseAddress);
    }

    dmaObjPtr->refCnt++;

    /* Writeback Global Object */
    Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

cppi_exit:
    /* End Critical Section. */
    Cppi_osalCsExit (key);
    *handle = (Cppi_Handle) dmaObjPtr;
    return retVal;
} /* Cppi_openStatus */

/**
 *  @b Description
 *  @n  
 *      This function opens the CPPI CPDMA instance using @ref Cppi_openStatus.  The return
 *      value from Cppi_openStatus is discarded.  
 *
 *  @param[in]  initCfg
 *      Initialization structure that contains the global configuration parameters 
 *      pertaining to CPPI CPDMA. 
 *
 *  @retval
 *      Success -   Handle to CPPI CPDMA object. Used as an input parameter to all other CPPI LLD APIs.
 *  @retval
 *      Failure -   NULL
 */
Cppi_Handle Cppi_open (Cppi_CpDmaInitCfg *initCfg)
{
    Cppi_Handle retVal;

    Cppi_openStatus (initCfg, &retVal);
    /* Return value intentionally discarded.  In order to check the return value, which
     * could warn that RM prevented register initialization, must use @ref Cppi_openStatus
     */

    return retVal;
} /* Cppi_open */

/**
 *  @b Description
 *  @n  
 *      This function closes the CPPI CPDMA instance. 
 *      The instance reference count is decremented. 
 *      CPPI CPDMA instance is closed only if the reference count is zero.
 *      CPPI CPDMA instance is closed only if all the Rx, Tx channels and Rx flows are closed.
 *      This function should be called to close all instances before re-opening the CPPI CPDMA instance. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *
 *  @pre  
 *      All Rx, Tx channel and Rx flow reference counts must be zero.
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      CPPI CPDMA object is freed if the reference count is zero.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_INITIALIZED
 *  @retval
 *      Failure -   CPPI_TX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_FLOWS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RM_ERR_FREEING_RESOURCE
 */
Cppi_Result Cppi_close (Cppi_Handle hnd)
{
    /* Do not ignore Rx, Tx channel and Rx flow reference counts */
    return (cppiCloseCommon(hnd, 0));
}

/**
 *  @b Description
 *  @n  
 *      This function closes the CPPI CPDMA instance. 
 *      The instance reference count is decremented regardless of the
 *      Rx, Tx channel and Rx flow object reference counts.
 *      The reference count will not be decremented to zero if any of the
 *      Rx, Tx channels or Rx flows are still open.
 *      This function should be called to close all instances before re-opening
 *      the CPPI CPDMA instance. 
 *
 *      This function is preferred for dyanamic systems where it is unknown
 *      what the reference counts will be zero.
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      CPPI CPDMA object is freed if the reference count is zero.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_INITIALIZED
 *  @retval
 *      Failure -   CPPI_TX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_CHANNELS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RX_FLOWS_NOT_CLOSED
 *  @retval
 *      Failure -   CPPI_RM_ERR_FREEING_RESOURCE
 */
Cppi_Result Cppi_closeDecRef (Cppi_Handle hnd)
{
    /* Do not ignore Rx, Tx channel and Rx flow reference counts */
    return (cppiCloseCommon(hnd, 1));
}

/**
 *  @b Description
 *  @n  
 *      This function opens a CPPI transmit channel. 
 *      The channel can be opened in two ways:
 *          1) If channel number in cfg structure is set to CPPI_PARAM_NOT_SPECIFIED, then a new 
 *              available channel is allocated.
 *          2) If channel number in cfg structure is a valid channel i.e., >= 0, 
 *               then the channel is allocated if free 
 *               else
 *               a handle to a previously opened channel is returned.
 *      The channel is configured only if it is a new channel allocation. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *
 *  @param[in]  cfg
 *      Tx channel configuration specifying scheduler priority for the channel, channel number, 
 *      channel enable, PS and EPIB filtering control, special AIF 
 *      Monolithic Mode.
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested channel number is a new channel allocation(1).
 *      or was already allocated. If the channel was previously allocated this parameter returns 
 *      the reference count.
 *
 *  @param[in]   cfgHw
 *      Configures the CPDMA hardware on channel creation if set.  If not set will need to close
 *      channel and reopen with this parameter unset. 
 *       
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      Transmit channel is allocated and configured.
 *
 *  @retval
 *      Success -   Channel Handle. Used as a input parameter in all other channel related APIs.
 *  @retval
 *      Failure -   NULL
 */
Cppi_ChHnd Cppi_txChannelOpenWithHwCfg (Cppi_Handle hnd, Cppi_TxChInitCfg *cfg, uint8_t *isAllocated, int32_t cfgHw)
{
    Cppi_DMAObj     *dmaObjPtr;
    Cppi_ChObj      *chObjPtr = NULL;
    uint32_t        index=0, bitPos;
    void            *key;
    int32_t         chNum = -1;
    uint32_t        mask;
    bool            rmReturnedChannel = false;

    if (cppiLObj.cppiRmServiceHandle)
    {
        chNum = cfg->channelNum;
    }

    *isAllocated = 0;

    if ((hnd == NULL) || (cfg == NULL))
    {
        return NULL;
    }

    dmaObjPtr = (Cppi_DMAObj *) hnd;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* Invalidate CPDMA Object */
    Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));
    /* Invalidate the heap descriptor, in case cppi_internal_heap_malloc is called */
    cppi_internal_heap_cache_begin (&cppiObject.obj.heapDesc);

    if ((cfg->channelNum != CPPI_PARAM_NOT_SPECIFIED) && (cfg->channelNum >= dmaObjPtr->maxTxCh))
    {
        /* End Critical Section. */
        Cppi_osalCsExit (key);
        return NULL;
    }

    if (cppiLObj.cppiRmServiceHandle)
    {
        if (cfg->channelNum == CPPI_PARAM_NOT_SPECIFIED)
        {
            if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaTxCh, &chNum, NULL))
            {
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
            else
            {
                cfg->channelNum = chNum;
                rmReturnedChannel = true;
            }
        }
    }

    if (cfg->channelNum >= 0)
    { 
        chObjPtr = (Cppi_ChObj *) cppi_list_get_head ((Cppi_ListNode **) &dmaObjPtr->txChHnd);
        while (chObjPtr != NULL)
        {
            /* Invalidate channel Object */
            Cppi_osalBeginMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

            if (chObjPtr->channelNum == cfg->channelNum)
            {
                if (cppiLObj.cppiRmServiceHandle)
                {
                    if (!rmReturnedChannel)
                    {
                        if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_USE, dmaObjPtr->rmCpdmaTxCh, &chNum, NULL))
                        {
                            /* End Critical Section. */
                            Cppi_osalCsExit (key);
                            return NULL;
                        }  
                    }
                }
                
                chObjPtr->refCnt++;
                dmaObjPtr->txChCnt++;
                *isAllocated = chObjPtr->refCnt;
            
                /* Writeback channel Object */
                Cppi_osalEndMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

                /* Writeback CPDMA Object */
                Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return (Cppi_ChHnd) chObjPtr;
            }
            /* Get next list entry */
            chObjPtr = (Cppi_ChObj *) cppi_list_get_next ((Cppi_ListNode *) chObjPtr);
        }
    }
    if (chObjPtr == NULL)
    {
        if (cppiLObj.cppiRmServiceHandle)
        {
            if (!rmReturnedChannel)
            {
                if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaTxCh, &chNum, NULL))
                {                 
                    /* End Critical Section. */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }

            /* Request approved */
            if (chNum >= dmaObjPtr->maxTxCh)
            {
                /* Returned resource not valid.  Free the resource and return */                
                cppiRmService(Rm_service_RESOURCE_FREE, dmaObjPtr->rmCpdmaTxCh, &chNum, NULL);
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
        }
        else
        {
            if (cfg->channelNum >= 0)
            {
                chNum = cfg->channelNum;  
            }
            else
            {
                for (chNum = 0; chNum < dmaObjPtr->maxTxCh; chNum++)
                {
                    index = chNum >> 5;
                    bitPos  = chNum & 0x1f;
                    mask = dmaObjPtr->txChMask[index];

                    if (mask == 0xffffffff) 
                    {
                      /* There are no available channels in this mask, so immediately
                       * skip to next mask to save cycles */
                      chNum += 31;
                      continue;
                    }
                    if ((mask & (1 << bitPos)) == 0) 
                    {
                        break; /* Found channel in chNum */
                    }
                }
                if (chNum == dmaObjPtr->maxTxCh) 
                {
                    /* Failed to allocate a channel -- all are used or permission denied */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }
        }

        /* We now have a valid channel number in chNum which was specified
         *  by user or found via search */
        chObjPtr = (Cppi_ChObj *) cppi_internal_heap_malloc (&cppiObject.obj.heapDesc, sizeof (Cppi_ChObj));
        if (chObjPtr == NULL)
        {
            /* End Critical Section. */
            Cppi_osalCsExit (key);
            return NULL;
        }

        /* Need to invalidate the new pointer, because it could be in same cache
         * line as "other CPPI data" since CPPI has dedicated heap.  Otherwise, old
         * data could be written on top of the "other CPPI data" if this line is
         * already in cache.
         * It is also critical at this point that no other items from the heap
         * are dirty, otherwise the new data would be destroyed.
         */
        Cppi_osalBeginMemAccess (chObjPtr, sizeof(Cppi_ChObj));
        /* Invalidate the whole list, to ensure no old data is left when linking
         * in cppi_list_cat.  Also, must be before chObjPtr is dirtied, else
         * new changes would be lost
         */
        cppi_list_cache_invalidate ((Cppi_ListNode *)dmaObjPtr->txChHnd);

        /* Initialize the allocated channel object */
        memset ((void *) chObjPtr, 0, sizeof (Cppi_ChObj));

        chObjPtr->refCnt++;
        dmaObjPtr->txChCnt++;

        index = chNum / 32;
        bitPos = chNum % 32;
        CSL_FINSR (dmaObjPtr->txChMask[index], bitPos, bitPos, 1);

        chObjPtr->channelNum = chNum;
        chObjPtr->dmaNum = dmaObjPtr->dmaNum;
        chObjPtr->chType = Cppi_ChType_TX_CHANNEL;
        chObjPtr->dmaObjHnd = (Cppi_DMAObj *) dmaObjPtr;
        *isAllocated = chObjPtr->refCnt;

        /* Add the channel object to channel list */
        cppi_list_cat ((Cppi_ListNode **) &dmaObjPtr->txChHnd, (Cppi_ListNode **) &chObjPtr);

    }

    if (*isAllocated == 1)
    {
        if (cfgHw)
        {
            CSL_FINS (dmaObjPtr->txSchedRegs->TX_CHANNEL_SCHEDULER_CONFIG_REG[chObjPtr->channelNum], 
                    CPPIDMA_TX_SCHEDULER_CONFIG_TX_CHANNEL_SCHEDULER_CONFIG_REG_PRIORITY, cfg->priority);

            CSL_FINS (dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_B, 
                CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_EINFO, cfg->filterEPIB);

            CSL_FINS (dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_B, 
                CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_FILT_PSWORDS, cfg->filterPS);

            CSL_FINS (dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_B, 
                CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE, cfg->aifMonoMode);

            if (cfg->txEnable)
            {
                uint32_t value = 0;
                CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, cfg->txEnable);
                dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A = value;
            }
        }
    }     

    /* Writeback channel Object */
    Cppi_osalEndMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

    /* Writeback CPDMA Object */
    Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    /* End Critical Section. */
    Cppi_osalCsExit (key);
    return (Cppi_ChHnd) chObjPtr;
}


/**
 *  @b Description
 *  @n  
 *      This function opens a CPPI transmit channel. 
 *      The channel can be opened in two ways:
 *          1) If channel number in cfg structure is set to CPPI_PARAM_NOT_SPECIFIED, then a new 
 *              available channel is allocated.
 *          2) If channel number in cfg structure is a valid channel i.e., >= 0, 
 *               then the channel is allocated if free 
 *               else
 *               a handle to a previously opened channel is returned.
 *      The channel is configured only if it is a new channel allocation. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *
 *  @param[in]  cfg
 *      Tx channel configuration specifying scheduler priority for the channel, channel number, 
 *      channel enable, PS and EPIB filtering control, special AIF 
 *      Monolithic Mode.
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested channel number is a new channel allocation(1).
 *      or was already allocated. If the channel was previously allocated this parameter returns 
 *      the reference count.
 *      
 * 
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      Transmit channel is allocated and configured.
 *
 *  @retval
 *      Success -   Channel Handle. Used as a input parameter in all other channel related APIs.
 *  @retval
 *      Failure -   NULL
 */
Cppi_ChHnd Cppi_txChannelOpen (Cppi_Handle hnd, Cppi_TxChInitCfg *cfg, uint8_t *isAllocated)
{
    return (Cppi_ChHnd) Cppi_txChannelOpenWithHwCfg (hnd, cfg, isAllocated, 1);
}

/**
 *  @b Description
 *  @n  
 *      This function opens a CPPI receive channel. 
 *      The channel can be opened in two ways:
 *          1) If channel number in cfg structure is set to CPPI_PARAM_NOT_SPECIFIED, then a new 
 *              available channel is allocated.
 *          2) If channel number in cfg structure is a valid channel i.e., >= 0, 
 *               then the channel is allocated if free 
 *               else
 *               a handle to a previously opened channel is returned.
 *      The channel is configured only if it is a new channel allocation. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *
 *  @param[in]  cfg
 *      Rx channel configuration specifying channel number, channel enable. 
 *
 *  @param[out]  isAllocated
 *      Indicates whether the requested channel number is a new channel allocation(1).
 *      or was already allocated. If the channel was previously allocated this parameter returns 
 *      the reference count.
 *
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @post  
 *      Receive channel is allocated and configured.
 *
 *  @retval
 *      Success -   Channel Handle. Used as a input parameter in all other channel related APIs.
 *  @retval
 *      Failure -   NULL
 */
Cppi_ChHnd Cppi_rxChannelOpen (Cppi_Handle hnd, Cppi_RxChInitCfg *cfg, uint8_t *isAllocated)
{
    Cppi_DMAObj     *dmaObjPtr;
    Cppi_ChObj      *chObjPtr = NULL;
    uint32_t        index=0, bitPos;
    void            *key;
    int32_t         chNum = -1;
    uint32_t        mask;
    bool            rmReturnedChannel = false;

    if (cppiLObj.cppiRmServiceHandle)
    {
        chNum = cfg->channelNum;
    }
    
    *isAllocated = 0;

    if ((hnd == NULL) || (cfg == NULL))
    {
        return NULL;
    }

    dmaObjPtr = (Cppi_DMAObj *) hnd;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* Invalidate CPDMA Object */
    Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));
    /* Invalidate the heap descriptor, in case cppi_internal_heap_malloc is called */
    cppi_internal_heap_cache_begin (&cppiObject.obj.heapDesc);

    if ((cfg->channelNum != CPPI_PARAM_NOT_SPECIFIED) && (cfg->channelNum >= dmaObjPtr->maxRxCh))
    {
        /* End Critical Section. */
        Cppi_osalCsExit (key);
        return NULL;
    }    

    if (cppiLObj.cppiRmServiceHandle)
    {
        if (cfg->channelNum == CPPI_PARAM_NOT_SPECIFIED)
        {
            if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaRxCh, &chNum, NULL))
            {
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
            else
            {
                cfg->channelNum = chNum;
                rmReturnedChannel = true;
            }
        }
    }

    if (cfg->channelNum >= 0)
    {
        /* Find the channel object.  No need for permissions check since non-NULL objects will already
          * have had their permissions verified */
        chObjPtr = (Cppi_ChObj *) cppi_list_get_head ((Cppi_ListNode **) &dmaObjPtr->rxChHnd);
        while (chObjPtr != NULL)
        {
            /* Invalidate channel Object */
            Cppi_osalBeginMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

            if (chObjPtr->channelNum == cfg->channelNum)
            {
                if (cppiLObj.cppiRmServiceHandle)
                {
                    if (!rmReturnedChannel)
                    {
                        if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_USE, dmaObjPtr->rmCpdmaRxCh, &chNum, NULL))
                        {
                            /* End Critical Section. */
                            Cppi_osalCsExit (key);
                            return NULL;
                        } 
                    }
                }
                
                chObjPtr->refCnt++;
                dmaObjPtr->rxChCnt++;
                *isAllocated = chObjPtr->refCnt;
                
                /* Writeback channel Object */
                Cppi_osalEndMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

                /* Writeback CPDMA Object */
                Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return (Cppi_ChHnd) chObjPtr;
            }
            /* Get next list entry */
            chObjPtr = (Cppi_ChObj *) cppi_list_get_next ((Cppi_ListNode *) chObjPtr);
        }
    }
    if (chObjPtr == NULL)
    {
        if (cppiLObj.cppiRmServiceHandle)
        {
            if (!rmReturnedChannel)
            {
                if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaRxCh, &chNum, NULL))
                {                 
                    /* End Critical Section. */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }

            /* Request approved */
            if (chNum >= dmaObjPtr->maxRxCh)
            {
                /* Returned resource not valid.  Free the resource and return */                
                cppiRmService(Rm_service_RESOURCE_FREE, dmaObjPtr->rmCpdmaRxCh, &chNum, NULL);
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
        }
        else
        {    
            if (cfg->channelNum >= 0)
            {
                chNum = cfg->channelNum;   
            }
            else
            {
                for (chNum = 0; chNum < dmaObjPtr->maxRxCh; chNum++)
                {
                    index = chNum >> 5;
                    bitPos  = chNum & 0x1f;
                    mask = dmaObjPtr->rxChMask[index];

                    if (mask == 0xffffffff) 
                    {
                      /* There are no available channels in this mask, so immediately
                       * skip to next mask to save cycles */
                      chNum += 31;
                      continue;
                    }
                    if ((mask & (1 << bitPos)) == 0) 
                    {
                        break; /* Found channel in chNum */
                    }
                }
                if (chNum == dmaObjPtr->maxRxCh) 
                {
                    /* Failed to allocate a channel -- all are used or permission denied */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }
        }

        /* We now have a valid channel number in chNum which was specified
         *  by user or found via search */
        chObjPtr = (Cppi_ChObj *) cppi_internal_heap_malloc (&cppiObject.obj.heapDesc, sizeof (Cppi_ChObj));
        if (chObjPtr == NULL)
        {
            /* End Critical Section. */
            Cppi_osalCsExit (key);
            return NULL;
        }

        /* Need to invalidate the new pointer, because it could be in same cache
         * line as "other CPPI data" since CPPI has dedicated heap.  Otherwise, old
         * data could be written on top of the "other CPPI data" if this line is
         * already in cache.
         * It is also critical at this point that no other items from the heap
         * are dirty, otherwise the new data would be destroyed.
         */
        Cppi_osalBeginMemAccess (chObjPtr, sizeof(Cppi_ChObj));
        /* Invalidate the whole list, to ensure no old data is left when linking
         * in cppi_list_cat.  Also, must be before chObjPtr is dirtied, else
         * new changes would be lost
         */
        cppi_list_cache_invalidate ((Cppi_ListNode *)&dmaObjPtr->txChHnd);

        /* Initialize the allocated channel object */
        memset ((void *) chObjPtr, 0, sizeof (Cppi_ChObj));

        chObjPtr->refCnt++;
        dmaObjPtr->rxChCnt++;

        index = chNum / 32;
        bitPos = chNum % 32;

        CSL_FINSR (dmaObjPtr->rxChMask[index], bitPos, bitPos, 1);

        chObjPtr->channelNum = chNum;
        chObjPtr->dmaNum = dmaObjPtr->dmaNum;
        chObjPtr->chType = Cppi_ChType_RX_CHANNEL;
        chObjPtr->dmaObjHnd = (Cppi_DMAObj *) dmaObjPtr;
        *isAllocated = chObjPtr->refCnt;
        
        /* Add the channel object to channel list */
        cppi_list_cat ((Cppi_ListNode **) &dmaObjPtr->rxChHnd, (Cppi_ListNode **) &chObjPtr);
    }

    if (*isAllocated == 1)
    {
        if (cfg->rxEnable)
        {
            uint32_t value = 0;
            CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, cfg->rxEnable);
            dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG = value;
        }
    }        

    /* Writeback channel Object */
    Cppi_osalEndMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));

    /* Writeback CPDMA Object */
    Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    /* End Critical Section. */
    Cppi_osalCsExit (key);
    return (Cppi_ChHnd) chObjPtr;
}

/**
 *  @b Description
 *  @n  
 *      This function enables a transmit or a receive channel. 
 *
 *  @param[in]  hnd
 *      Channel handle.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 */
Cppi_Result Cppi_channelEnable (Cppi_ChHnd hnd)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    uint32_t        value = 0;
    void            *key;

    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* No RM permissions check since this was done during channel open */

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    dmaObjPtr = chObjPtr->dmaObjHnd;

    if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
    {
        CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, (uint32_t) 1);
        dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG = value;
    }
    else
    {
        CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, (uint32_t) 1);
        dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A = value;
    }

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function disables a transmit or a receive channel. 
 *
 *  @param[in]  hnd
 *      Channel handle.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 */
Cppi_Result Cppi_channelDisable (Cppi_ChHnd hnd)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    void            *key;

    /* No RM permissions check since this was done during channel open */

    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    dmaObjPtr = chObjPtr->dmaObjHnd;
    
    if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
        dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG = 0;
    else
        dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A = 0;
  
    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function tears down a transmit or a receive channel by writing 
 *      to the channel teardown bit. 
 *      This function can do a blocking wait for the teardown to complete OR
 *      return immediately and it is up to the callee to check for channel 
 *      teardown status. The behavior is based on the "wait" input parameter
 *
 *  @param[in]  hnd
 *      Channel handle.
 *
 *  @param[in]  wait
 *      Specifies Wait or No Wait for teardown complete.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM 
 */
Cppi_Result Cppi_channelTeardown (Cppi_ChHnd hnd, Cppi_Wait wait)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    uint32_t        value = 0;
    void            *key;

    /* No RM permissions check since this was done during channel open */

    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();
    
    dmaObjPtr = chObjPtr->dmaObjHnd;

    if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
    {
        CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, (uint32_t) 1);
        CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_TEARDOWN, 1);
        dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG = value;
    }
    else
    {
        CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, (uint32_t) 1);
        CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_TEARDOWN, 1);
        dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A = value;
    }
 
    if (wait)
    {
        uint32_t  enable;
        do
        {
            if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
                enable = CSL_FEXT (dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG, 
                    CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE);
            else
                enable = CSL_FEXT (dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A, 
                    CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE);
        } while (enable);
    }

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function closes a transmit or a receive channel.
 *      The channel reference count is decremented. The channel is freed only if the 
 *      reference count is zero. 
 *
 *  @param[in]  hnd
 *      Channel handle.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @post
 *      Channel is freed if the reference count is zero and available for reallocation.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CHANNEL_NOT_OPEN
 *  @retval
 *      Failure -   CPPI_RM_ERR_FREEING_RESOURCE 
 */
Cppi_Result Cppi_channelClose (Cppi_ChHnd hnd)
{
    Cppi_DMAObj     *dmaObjPtr;
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    int32_t         refCount;
    void            *key;
    char            *rmResName;
    int32_t         chNum;
    int             closeChannel = 1;
    int             isAllocated = 0; /* == 0 implies non-RM case will always close hw */

    /* No RM check because permissions were checked at open */
    
    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();
    
    /* Invalidate channel Object */
    Cppi_osalBeginMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));
    /* Invalidate the heap descriptor, in case cppi_internal_heap_free is called */
    cppi_internal_heap_cache_begin (&cppiObject.obj.heapDesc);

    if (chObjPtr->refCnt == 0)
    {
        /* End Critical Section. */
        Cppi_osalCsExit (key);
        return CPPI_CHANNEL_NOT_OPEN;
    }
    dmaObjPtr = chObjPtr->dmaObjHnd;

    if (cppiLObj.cppiRmServiceHandle)
    {
        if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
        {
            rmResName = dmaObjPtr->rmCpdmaRxCh;
        }
        else
        {
            rmResName = dmaObjPtr->rmCpdmaTxCh;
        }
        chNum = chObjPtr->channelNum;
        closeChannel = cppiRmService(Rm_service_RESOURCE_FREE, rmResName, &chNum, &isAllocated);
    }                           

    if (closeChannel)
    {
        /* Invalidate CPDMA Object */
        Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));
       
        chObjPtr->refCnt--;
        refCount = chObjPtr->refCnt;

        /* Writeback channel Object */
        Cppi_osalEndMemAccess ((void *) chObjPtr, sizeof (Cppi_ChObj));
        
        if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
        {
            dmaObjPtr->rxChCnt--;
            /* If reference count is zero then free the object */
            if (refCount == 0)
            {
                uint32_t  index, bitPos;
                
                index = chObjPtr->channelNum / 32;
                bitPos = chObjPtr->channelNum % 32;

                /* Invalidate list for remove_node precondition */
                /* chObjPtr was written back to avoid loss during invalidate */
                cppi_list_cache_invalidate ((Cppi_ListNode *)dmaObjPtr->rxChHnd);

                /* Only disable the hardware if this is last channel to close per RM */
                if (isAllocated == 0)
                {
                    CSL_FINSR (dmaObjPtr->rxChMask[index], bitPos, bitPos, 0);
                }
                cppi_list_remove_node ((Cppi_ListNode **) &dmaObjPtr->rxChHnd, (Cppi_ListNode *) chObjPtr);
                cppi_internal_heap_free (&cppiObject.obj.heapDesc, chObjPtr);
            }      
        }
        else
        {
            dmaObjPtr->txChCnt--;
            /* If reference count is zero then free the object */
            if (refCount == 0)
            {
                uint32_t index, bitPos;

                index = chObjPtr->channelNum / 32;
                bitPos = chObjPtr->channelNum % 32;
                
                /* Invalidate list for remove_node precondition */
                /* chObjPtr was written back to avoid loss during invalidate */
                cppi_list_cache_invalidate ((Cppi_ListNode *)dmaObjPtr->txChHnd);

                CSL_FINSR (dmaObjPtr->txChMask[index], bitPos, bitPos, 0);
                cppi_list_remove_node ((Cppi_ListNode**) &dmaObjPtr->txChHnd, (Cppi_ListNode*) chObjPtr);
                cppi_internal_heap_free (&cppiObject.obj.heapDesc, chObjPtr);
            } 
        }       

        /* Writeback CPDMA Object */
        Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj)); 
    }
    else
    {
        refCount = CPPI_RM_ERR_FREEING_RESOURCE;
    }

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return refCount;
}

/**
 *  @b Description
 *  @n  
 *      This function pauses a transmit or a receive channel. 
 *
 *  @param[in]  hnd
 *      Channel handle.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Success -   CPPI_SOK
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 */
Cppi_Result Cppi_channelPause (Cppi_ChHnd hnd)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    uint32_t          value = 0;
    void            *key;

    /* No RM permissions check since this was done during channel open */

    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    dmaObjPtr = chObjPtr->dmaObjHnd;

    if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
    {
        CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, (uint32_t) 1);
        CSL_FINS (value, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_PAUSE, 1);
        dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG = value;
    }
    else
    {
        CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, (uint32_t) 1);
        CSL_FINS (value, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_PAUSE, 1);
        dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A = value;
    }  

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return CPPI_SOK;
}

/**
 *  @b Description
 *  @n  
 *      This function returns the enable or disable channel status. 
 *
 *  @param[in]  hnd
 *      Channel handle.
 * 
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Success -   Channel Status
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 */
Cppi_Result Cppi_channelStatus (Cppi_ChHnd hnd)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    void            *key;
    Cppi_Result      retVal;

    /* No RM check since this is a read only operation */

    if (chObjPtr == NULL)
        return CPPI_INVALID_PARAM;

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    dmaObjPtr = chObjPtr->dmaObjHnd;

    if (chObjPtr->chType == Cppi_ChType_RX_CHANNEL)
        retVal = (CSL_FEXT (dmaObjPtr->rxChRegs->RX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].RX_CHANNEL_GLOBAL_CONFIG_REG, 
            CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE));
    else
        retVal = (CSL_FEXT (dmaObjPtr->txChRegs->TX_CHANNEL_GLOBAL_CONFIG[chObjPtr->channelNum].TX_CHANNEL_GLOBAL_CONFIG_REG_A,
            CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE));

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return retVal;
}

/**
 *  @b Description
 *  @n  
 *      This function configures a receive flow. 
 *      The flow can be configured in two ways:
 *          1) If flow ID number in cfg structure is set to CPPI_PARAM_NOT_SPECIFIED, then a new 
 *              available flow is allocated.
 *          2) If flow ID number is cfg structure is a valid flow i.e., >= 0, 
 *               then the flow is allocated if free 
 *               else
 *               a handle to a previously opened flow is returned.

 *      The flow is configured only if it is a new flow allocation. 
 *
 *  @param[in]  hnd
 *      handle returned by Cppi_open API.
 *      
 *  @param[in]  cfg
 *      Rx flow configuration
 * 
 *  @param[out]  isAllocated
 *      Indicates whether the requested flow is a new flow allocation(1).
 *      or was already allocated. If the flow was previously allocated this parameter returns 
 *      the reference count.
 *       
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *       
 *  @post  
 *      Rx flow is allocated and configured.
 *
 *  @retval
 *      Success -   Flow Handle.
 *  @retval
 *      Failure -   NULL
 */
Cppi_FlowHnd Cppi_configureRxFlow (Cppi_Handle hnd, Cppi_RxFlowCfg *cfg, uint8_t *isAllocated)
{
    Cppi_DMAObj     *dmaObjPtr = (Cppi_DMAObj *) hnd;
    Cppi_FlowObj    *flowObjPtr = NULL;
    uint32_t        index=0, bitPos;
    void            *key;
    int32_t         flowId = -1;
    uint32_t        mask;
    bool            rmReturnedFlow = false;       

    if (cppiLObj.cppiRmServiceHandle)
    {
        flowId = cfg->flowIdNum;
    }

    *isAllocated = 0;

    if ((dmaObjPtr == NULL) || (cfg == NULL))
        return NULL;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* Invalidate CPDMA Object */
    Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    /* Invalidate the heap descriptor, in case cppi_internal_heap_malloc is called */
    cppi_internal_heap_cache_begin (&cppiObject.obj.heapDesc);

    if ((cfg->flowIdNum != CPPI_PARAM_NOT_SPECIFIED) && (cfg->flowIdNum >= dmaObjPtr->maxRxFlow))
    {
        /* End Critical Section. */
        Cppi_osalCsExit (key);
        return NULL;
    }

    if (cppiLObj.cppiRmServiceHandle)
    {
        if (cfg->flowIdNum == CPPI_PARAM_NOT_SPECIFIED)
        {
            if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaRxFlow, &flowId, NULL))
            {
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
            else
            {
                cfg->flowIdNum = flowId;
                rmReturnedFlow = true;
            }
        }
    }
    
    if (cfg->flowIdNum >= 0)
    {
        /* Find the flow object.  No need for permissions check since non-NULL objects will already
          * have had their permissions verified */
        flowObjPtr = (Cppi_FlowObj *) cppi_list_get_head ((Cppi_ListNode **) &dmaObjPtr->rxFlowHnd);
        while (flowObjPtr != NULL)
        {
            /* Invalidate flow Object */
            Cppi_osalBeginMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));

            if (flowObjPtr->flowId == cfg->flowIdNum)
            {
                if (cppiLObj.cppiRmServiceHandle)
                {
                    if (!rmReturnedFlow)
                    {                
                        if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_USE, dmaObjPtr->rmCpdmaRxFlow, &flowId, NULL))
                        {
                            /* End Critical Section. */
                            Cppi_osalCsExit (key);
                            return NULL;
                        }                
                    }
                }
                
                flowObjPtr->refCnt++;
                dmaObjPtr->rxFlowCnt++;
                *isAllocated = flowObjPtr->refCnt;

                /* Writeback flow Object */
                Cppi_osalEndMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));

                /* Writeback CPDMA Object */
                Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return (Cppi_FlowHnd) flowObjPtr;
            }
            /* Get next list entry */
            flowObjPtr = (Cppi_FlowObj *) cppi_list_get_next ((Cppi_ListNode *) flowObjPtr);
        }
    }

    /* New flow allocation - either specified or next available one */
    if (flowObjPtr == NULL)
    {
        if (cppiLObj.cppiRmServiceHandle)
        {
            if (!rmReturnedFlow)
            {
                if (!cppiRmService(Rm_service_RESOURCE_ALLOCATE_INIT, dmaObjPtr->rmCpdmaRxFlow, &flowId, NULL))
                {                 
                    /* End Critical Section. */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }

            /* Request approved */
            if (flowId >= dmaObjPtr->maxRxFlow)
            {
                /* Returned resource not valid.  Free the resource and return */                
                cppiRmService(Rm_service_RESOURCE_FREE, dmaObjPtr->rmCpdmaRxFlow, &flowId, NULL);
                /* End Critical Section. */
                Cppi_osalCsExit (key);
                return NULL;
            }
        }
        else
        {       
            if (cfg->flowIdNum >= 0)
            {
                flowId = cfg->flowIdNum;  
            }
            else
            {
                for (flowId = 0; flowId < dmaObjPtr->maxRxFlow; flowId++)
                {
                    index = flowId >> 5;
                    bitPos  = flowId & 0x1f;
                    mask = dmaObjPtr->rxFlowMask[index];

                    if (mask == 0xffffffff) 
                    {
                      /* There are no available flows in this mask, so immediately
                       * skip to next mask to save cycles */
                      flowId += 31;
                      continue;
                    }
                    if ((mask & (1 << bitPos)) == 0)
                    {
                        break; /* Found channel in flowId */
                    }
                }
                if (flowId == dmaObjPtr->maxRxFlow) 
                {
                    /* Failed to allocate a flow -- all are used or permission denied */
                    Cppi_osalCsExit (key);
                    return NULL;
                }
            }
        }

        /* We now have a valid channel number in chNum which was specified
         *  by user or found via search */
        flowObjPtr = (Cppi_FlowObj *) cppi_internal_heap_malloc (&cppiObject.obj.heapDesc, sizeof (Cppi_FlowObj));
        if (flowObjPtr == NULL)
        {
            /* End Critical Section. */
            Cppi_osalCsExit (key);
            return NULL;
        }

        /* Need to invalidate the new pointer, because it could be in same cache
         * line as "other CPPI data" since CPPI has dedicated heap.  Otherwise, old
         * data could be written on top of the "other CPPI data" if this line is
         * already in cache.
         * It is also critical at this point that no other items from the heap
         * are dirty, otherwise the new data would be destroyed.
         */
        Cppi_osalBeginMemAccess (flowObjPtr, sizeof(Cppi_FlowObj));
        /* Invalidate the whole list, to ensure no old data is left when linking
         * in cppi_list_cat.  Also, must be before chObjPtr is dirtied, else
         * new changes would be lost
         */
        cppi_list_cache_invalidate ((Cppi_ListNode *)dmaObjPtr->rxFlowHnd);

        /* Initialize the allocated flow object */
        memset ((void *) flowObjPtr, 0, sizeof (Cppi_FlowObj));

        flowObjPtr->refCnt++;
        dmaObjPtr->rxFlowCnt++;
        
        index = flowId / 32;
        bitPos = flowId % 32;
        CSL_FINSR (dmaObjPtr->rxFlowMask[index], bitPos, bitPos, 1);

        flowObjPtr->dmaNum = dmaObjPtr->dmaNum;
        flowObjPtr->flowId = flowId;
        flowObjPtr->dmaObjHnd = dmaObjPtr;
        *isAllocated = flowObjPtr->refCnt;
                
        /* Add the flow object to flow list */
        cppi_list_cat ((Cppi_ListNode **) &dmaObjPtr->rxFlowHnd, (Cppi_ListNode **) &flowObjPtr);

    }

    if (*isAllocated == 1)
    {
            uint32_t      reg, temp = 0;

            index = flowObjPtr->flowId;
            reg = 0;
                    /* Rx flow configuration register A */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_DEST_QNUM, cfg->rx_dest_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_DEST_QMGR, cfg->rx_dest_qmgr);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_SOP_OFFSET, cfg->rx_sop_offset);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_PS_LOCATION, cfg->rx_ps_location);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_DESC_TYPE, cfg->rx_desc_type);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_ERROR_HANDLING, cfg->rx_error_handling);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_PSINFO_PRESENT, cfg->rx_psinfo_present);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_A_RX_EINFO_PRESENT, cfg->rx_einfo_present);
                    dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_A = reg;

            reg = 0;
                    /* Rx flow configuration register B */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_LO, cfg->rx_dest_tag_lo);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_B_RX_DEST_TAG_HI, cfg->rx_dest_tag_hi);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_LO, cfg->rx_src_tag_lo);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_B_RX_SRC_TAG_HI, cfg->rx_src_tag_hi);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_B = reg;

            reg = 0;
            /* Rx flow configuration register C */                
            temp = ((cfg->rx_size_thresh0_en) | (cfg->rx_size_thresh1_en << 1) | (cfg->rx_size_thresh2_en << 2));

            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_C_RX_SIZE_THRESH_EN, temp);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_LO_SEL, cfg->rx_dest_tag_lo_sel);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_C_RX_DEST_TAG_HI_SEL, cfg->rx_dest_tag_hi_sel);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_LO_SEL, cfg->rx_src_tag_lo_sel);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_C_RX_SRC_TAG_HI_SEL, cfg->rx_src_tag_hi_sel);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_C = reg;

            reg = 0;
                        /* Rx flow configuration register D */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QNUM, cfg->rx_fdq1_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_D_RX_FDQ1_QMGR, cfg->rx_fdq1_qmgr);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QNUM, cfg->rx_fdq0_sz0_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_D_RX_FDQ0_SZ0_QMGR, cfg->rx_fdq0_sz0_qmgr);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_D = reg;

            reg = 0;
                        /* Rx flow configuration register E */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QNUM, cfg->rx_fdq3_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_E_RX_FDQ3_QMGR, cfg->rx_fdq3_qmgr);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QNUM, cfg->rx_fdq2_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_E_RX_FDQ2_QMGR, cfg->rx_fdq2_qmgr);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_E = reg;

            reg = 0;
                        /* Rx flow configuration register F */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH1, (cfg->rx_size_thresh1 >> 5));
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_F_RX_SIZE_THRESH0, (cfg->rx_size_thresh0  >> 5));
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_F = reg;
            
            reg = 0;
            /* Rx flow configuration register G */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QNUM, cfg->rx_fdq0_sz1_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_G_RX_FDQ0_SZ1_QMGR, cfg->rx_fdq0_sz1_qmgr);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_G_RX_SIZE_THRESH2, (cfg->rx_size_thresh2) >> 5);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_G = reg;

            reg = 0;
            /* Rx flow configuration register H */
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QNUM, cfg->rx_fdq0_sz3_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ3_QMGR, cfg->rx_fdq0_sz3_qmgr);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QNUM, cfg->rx_fdq0_sz2_qnum);
            CSL_FINS (reg, CPPIDMA_RX_FLOW_CONFIG_RX_FLOW_CONFIG_REG_H_RX_FDQ0_SZ2_QMGR, cfg->rx_fdq0_sz2_qmgr);
            dmaObjPtr->rxFlowRegs->RX_FLOW_CONFIG[index].RX_FLOW_CONFIG_REG_H = reg;

    }

    /* Writeback flow Object */
    Cppi_osalEndMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));

    /* Writeback CPDMA Object */
    Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

    /* End Critical Section. */
    Cppi_osalCsExit (key);
    return (Cppi_FlowHnd) flowObjPtr;
}

/**
 *  @b Description
 *  @n  
 *      This function closes a receive flow. 
 *      The flow reference count is decremented. The flow is freed only if the 
 *      reference count is zero. 
 *
 *  @param[in]  hnd
 *      flow handle.
 * 
 *  @pre  
 *      Cppi_configureRxFlowfunction should be called before calling this function.
 *
 *  @post
 *      Flow is freed if the reference count is zero and available for reallocation.
 *
 *  @retval
 *      Success -   Current reference count.
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_FLOW_NOT_OPEN
 *  @retval
 *      Failure -   CPPI_RM_ERR_FREEING_RESOURCE
 */
Cppi_Result Cppi_closeRxFlow (Cppi_FlowHnd hnd)
{
    Cppi_FlowObj    *flowObjPtr = (Cppi_FlowObj *) hnd;
    Cppi_DMAObj     *dmaObjPtr;
    int32_t         refCount;
    void            *key;
    int32_t         flowId;
    int             closeFlow = 1;

    if (hnd == NULL)
        return CPPI_INVALID_PARAM;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();
    
    /* Invalidate flow Object */
    Cppi_osalBeginMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));

    /* Invalidate the heap descriptor, in case cppi_internal_heap_free is called */
    cppi_internal_heap_cache_begin (&cppiObject.obj.heapDesc);

    if (flowObjPtr->refCnt == 0)
    {
        /* End Critical Section. */
        Cppi_osalCsExit (key);
        return CPPI_FLOW_NOT_OPEN;
    }
    dmaObjPtr = flowObjPtr->dmaObjHnd;

    if (cppiLObj.cppiRmServiceHandle)
    {
        /* Make sure the flow has been allocated to the RM instance requesting the free */
        flowId = flowObjPtr->flowId;
        /* Don't need isAllocated, because close only frees local memory, doesn't disable HW */
        closeFlow = cppiRmService(Rm_service_RESOURCE_FREE, dmaObjPtr->rmCpdmaRxFlow, &flowId, NULL);
    }             

    if (closeFlow)
    {
        /* Invalidate CPDMA Object */
        Cppi_osalBeginMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));

        /* This dirties flowObjPtr, so need to clean it before deleting! */
        flowObjPtr->refCnt--;
        refCount = flowObjPtr->refCnt;

        /* If reference count is zero then free the object */
        if (refCount == 0)
        {
            uint32_t  index, bitPos;

            index = flowObjPtr->flowId / 32;
            bitPos = flowObjPtr->flowId % 32;

            /* Writeback all dirty lines that belong to heap before calling cppi_list_cache_invalidate */
            Cppi_osalEndMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));
            /* Invalidate list for remove_node precondition */
            cppi_list_cache_invalidate ((Cppi_ListNode *)dmaObjPtr->rxFlowHnd);

            CSL_FINSR (dmaObjPtr->rxFlowMask[index], bitPos, bitPos, 0);
            cppi_list_remove_node ((Cppi_ListNode **) &dmaObjPtr->rxFlowHnd, (Cppi_ListNode *) flowObjPtr);
            cppi_internal_heap_free (&cppiObject.obj.heapDesc, flowObjPtr);
        }

        dmaObjPtr->rxFlowCnt--;

        /* Writeback flow Object */
        Cppi_osalEndMemAccess ((void *) flowObjPtr, sizeof (Cppi_FlowObj));

        /* Writeback CPDMA Object */
        Cppi_osalEndMemAccess ((void *) dmaObjPtr, sizeof (Cppi_DMAObj));
    }
    else
    {
        refCount = CPPI_RM_ERR_FREEING_RESOURCE;
    }

    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return refCount;
}

/** 
 *  @b Description
 *  @n  
 *      Given the channel handle the function returns the channel number
 *      corresponding to the handle
 *
 *  @param[in]  hnd      
 *      Channel handle
 *
 *  @pre  
 *      Cppi_rxChannelOpen or Cppi_txChannelOpen function should be called before 
 *      calling this function.
 *
 *  @retval
 *      Channel number
 */
uint32_t Cppi_getChannelNumber (Cppi_ChHnd hnd)
{
    Cppi_ChObj      *chObjPtr = (Cppi_ChObj *) hnd;

    /* No RM permissions check since this is a read only operation */
    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */
    /* No critical section because already atomic operation & shared object not written */

    return (chObjPtr->channelNum);
}

/** 
 *  @b Description
 *  @n  
 *      Given the flow handle the function returns the flow ID
 *      corresponding to the handle
 *
 *  @param[in]  hnd      
 *      Flow handle
 *
 *  @pre  
 *      Cppi_configureRxFlowfunction should be called before calling this function.
 *
 *  @retval
 *      Flow ID
 */
uint32_t Cppi_getFlowId (Cppi_FlowHnd hnd)
{
    Cppi_FlowObj    *flowObjPtr = (Cppi_FlowObj *) hnd;

    /* No RM permissions check since this is a read only operation */
    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */
    /* No critical section because already atomic operation & shared object not written */

    return (flowObjPtr->flowId);
}

/** 
 *  @b Description
 *  @n  
 *      This function enables or disables CPDMA loopback 
 *
 *  @param[in]  hnd      
 *      handle returned by Cppi_open API.
 *
 *  @param[in]  loopback      
 *      0 - To disable loopback
 *      1 - To enable loopback
 *
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @retval
 *      Success -   CPPI_SOK 
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_INITIALIZED
 */
Cppi_Result Cppi_setCpdmaLoopback (Cppi_Handle hnd, uint8_t loopback)
{
    Cppi_DMAObj     *dmaObjPtr;
    Cppi_Result      retVal;
    void            *key;

    /* No RM permissions check since this was done during channel open */

    if (hnd == NULL)
    {
        return CPPI_INVALID_PARAM;
    }

    dmaObjPtr = (Cppi_DMAObj *) hnd;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();

    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    if (dmaObjPtr->refCnt == 0)
    {
        retVal = CPPI_CPDMA_NOT_INITIALIZED;
        goto end;
    }

    CSL_FINS (dmaObjPtr->gblCfgRegs->EMULATION_CONTROL_REG, 
                    CPPIDMA_GLOBAL_CONFIG_EMULATION_CONTROL_REG_LOOPBACK_EN, loopback);

    retVal = CPPI_SOK;
end:
    /* End Critical Section. */
    Cppi_osalCsExit (key);

    return retVal;
}

/** 
 *  @b Description
 *  @n  
 *      This function returns the CPDMA loopback configuration
 *
 *  @param[in]  hnd      
 *      handle returned by Cppi_open API.
 *
 *  @pre  
 *      Cppi_open function should be called before calling this function.
 *
 *  @retval
 *      Success 
 *          0 - loopback is disabled 
 *          1 - loopback is enabled
 *          
 *  @retval
 *      Failure -   CPPI_INVALID_PARAM
 *  @retval
 *      Failure -   CPPI_CPDMA_NOT_INITIALIZED
 */
Cppi_Result Cppi_getCpdmaLoopback (Cppi_Handle hnd)
{
    Cppi_DMAObj     *dmaObjPtr;
    Cppi_Result      retVal;
    void            *key;

    /* No RM permissions check since this is a read only operation */

    if (hnd == NULL)
    {
        return CPPI_INVALID_PARAM;
    }

    dmaObjPtr = (Cppi_DMAObj *) hnd;

    /* Begin Critical Section before accessing shared resources. */
    key = Cppi_osalCsEnter ();
    /* No cache operation because it was done in Cppi_xxChannelOpen and old data is still valid */

    if (dmaObjPtr->refCnt == 0)
    {
        retVal = CPPI_CPDMA_NOT_INITIALIZED;
        goto end;
    }
    retVal = (CSL_FEXT (dmaObjPtr->gblCfgRegs->EMULATION_CONTROL_REG, 
                    CPPIDMA_GLOBAL_CONFIG_EMULATION_CONTROL_REG_LOOPBACK_EN));

end:
    /* End Critical Section. */
    Cppi_osalCsExit (key);
    return retVal;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the CPPI LLD.
 *
 *  @retval
 *      Version Information.
 */
uint32_t Cppi_getVersion (void)
{
    return CPPI_LLD_VERSION_ID;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version string for the CPPI LLD.
 *
 *  @retval
 *      Version String.
 */
const char* Cppi_getVersionStr (void)
{
    return cppiLldVersionStr;
}

/**
@}
*/

