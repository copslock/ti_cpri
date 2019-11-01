/*
 *
 * Header file for the Demo application for the EDMA3 Driver.
 *
 * Copyright (C) 2013-2016 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef HYPLNKINFRDMA_H_
#define HYPLNKINFRDMA_H_

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

#include "hyplnkResource.h"
#include "hyplnkLLDIFace.h"
#include "hyplnkLLDCfg.h"

#define ACC_ENTRY_SIZE              NUM_PACKETS + 1
#define CPPI_COMPLETION_QUE_MGR     0
#define CPPI_COMPLETION_QUE_NUM     1000

//Address of remote queue manager
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(DEVICE_K2E) || defined(SOC_K2K) || defined(SOC_K2H) || defined(SOC_K2E)
#define QMSS_DATA_ADDR CSL_QMSS_DATA_QM1_QUEUE_MANAGEMENT_REGS
#else
#define QMSS_DATA_ADDR CSL_QM_SS_DATA_QM_QUEUE_DEQUEUE_REGS
#endif
/* Memory block transfer buffer for QMSS */
typedef struct {
	uint32_t descriptor[MONOLITHIC_DESC_DATA_OFFSET/DATA_SIZE]; //descriptor requires 16 Bytes
	uint32_t dataBuffer[SIZE_DATA_BUFFER/NUM_PACKETS]; //Split up by number of packages
} hyplnkQMSSExamplePacket_t;

typedef struct {
	hyplnkQMSSExamplePacket_t txQmssBlock[NUM_PACKETS];
} hyplnkQMSSExamplePacketBlock_t;

typedef struct {
	uint32_t MRaddresses; //This will hold the address of the memory region
	uint32_t isRemote; //States if memory region is remote or not
} hyplnkMemRegionsAddress_t;

Cppi_Handle InfraDMAInit (int remoteQMSSPtr, int remoteMemRegionPtr, Qmss_Result *localRegion,
		Qmss_Result *remoteRegion, Cppi_Handle cppiHnd);
int InfraDMATransfer (Cppi_Handle cppiHnd, uint32_t transferValue, Qmss_Result localRegion,
		Qmss_Result remoteRegion,hyplnkQMSSExamplePacketBlock_t *HyperLinkMemRegAddr);
void InfraDMADeinit(Cppi_Handle cppiHnd);


#endif /* HYPLNKINFRDMA_H_ */
