/*
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

#ifndef PCIEEDMA_H_
#define PCIEEDMA_H_

#ifdef _TMS320C6X 
#include <c6x.h>
#endif
/* Include EDMA3 Driver */
#include <ti/sdo/edma3/drv/sample/bios6_edma3_drv_sample.h>
#include <pcie_sample.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BUILD_TDA2XX_MPU
/* To enable/disable the cache .*/
#define EDMA3_ENABLE_DCACHE                 (1u)
#endif

/* OPT Field specific defines */
#define OPT_SYNCDIM_SHIFT                   (0x00000002u)
#define OPT_TCC_MASK                        (0x0003F000u)
#define OPT_TCC_SHIFT                       (0x0000000Cu)
#define OPT_ITCINTEN_SHIFT                  (0x00000015u)
#define OPT_TCINTEN_SHIFT                   (0x00000014u)
#define OPT_TCCMOD_SHIFT                    (0x0000000Bu)

/**
 * EDMA3 Driver Handle, which is used to call all the Driver APIs.
 * It gets initialized during EDMA3 Initialization.
 */
extern EDMA3_DRV_Handle hEdma[];

extern void pcie_edma_cb_isr1 (uint32_t tcc, EDMA3_RM_TccStatus status,
                        void *appData);

signed char*  getGlobalAddr(signed char* addr);
/* Flag variable to check transfer completion on channel 1 */
extern volatile short irqRaised1;

/*
 * Note that not all of EDMAs are implemented. These
 * are only to show what other types of EDMAs can be integrated
 * with this example code.
 */
typedef enum {

	EDMA3            = 0,
	QDMA             = 1,
	EDMA3_WITH_LINK  = 2,
	QDMA_WITH_LINK   = 3,
	EDMA3_WITH_CHAIN = 4,
	EDMA3_POLL_MODE  = 5,
	EDMA3_PING_PONG  = 6,
	EDMA3_MISC       = 7

} EDMA3_Type;


/* Define to verify the default RM config.
 * Additional configuration required. Update the
 * gblCfgReqdArray[] to reflect the master/slave config.
 * In the case of multiple instances default configuration
 * may require more than one cores other than core 0 to be master.
 * #define EDMA3_DRV_USE_DEF_RM_CFG
 */

#define GLOBAL_ADDR(addr) (getGlobalAddr(addr))

/* This is a list of the type of EDMA transfers.
 * NOTE: Not all of these are configured. Only
 * EDMA3 and QDMA work
 * */


EDMA3_DRV_Handle edmaInit(EDMA3_DRV_Handle hEdma);

static inline uint64_t EdmaReadTime(void)
{
    uint64_t timeVal;
    uint32_t low;

#if defined (_TMS320C6X)
    uint32_t high = 0;
	low = TSCL;
    high = TSCH;
    timeVal = _itoll(high,low);
#elif __ARM_ARCH_7A__
    static uint32_t high = 0, last_low = 0;
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(low));
    if (low < last_low)
    {
        high++;
    }
    last_low = low;
    timeVal = (((uint64_t)high) << 32) | low;
#else
/* M4 specific implementation*/
  static uint32_t simuTimer = 0;
  simuTimer++;
  timeVal = (uint64_t)simuTimer;
#endif
    return timeVal;
}

/**
 *  \brief   EDMA3 mem-to-mem data copy test case, using a QDMA channel.
 *
 *  \param  hEdma       [IN]      EDMA handle
 *  \param  hEdma       [IN]      EDMA transfer type
 *  \param  srcBuff     [IN]      Source buffer address
 *  \param  dstBuff     [IN]      Destination buffer address
 *  \param  acnt        [IN]      Number of bytes in an array
 *  \param  bcnt        [IN]      Number of arrays in a frame
 *  \param  ccnt        [IN]      Number of frames in a block
 *  \param  syncType    [IN]      Synchronization type (A/AB Sync)
 *  \param  totalTime   [out]     Total time it takes to transfer data
 *
 *  \return  EDMA3_DRV_SOK or EDMA3_DRV Error Code
 */
void edmaTransfer(EDMA3_DRV_Handle hEdma,
		          EDMA3_Type EdmaType,
		          unsigned int* src,
		          unsigned int* dst,
		          unsigned int acnt,
                  unsigned int bcnt,
                  unsigned int ccnt,
                  EDMA3_DRV_SyncType syncType,
                  unsigned long* totalTime);

void edmaDeinit(EDMA3_DRV_Handle hEdma);

/**
 *  \brief   EDMA3 mem-to-mem data copy test case, using a DMA channel.
 *
 *  \param  hEdma       [IN]      EDMA handle
 *  \param  srcBuff     [IN]      Source buffer address
 *  \param  dstBuff     [IN]      Destination buffer address
 *  \param  acnt        [IN]      Number of bytes in an array
 *  \param  bcnt        [IN]      Number of arrays in a frame
 *  \param  ccnt        [IN]      Number of frames in a block
 *  \param  syncType    [IN]      Synchronization type (A/AB Sync)
 *  \param  totalTime   [out]     Total time it takes to transfer data
 *
 *  \return  EDMA3_DRV_SOK or EDMA3_DRV Error Code
 */
EDMA3_DRV_Result edma3_test(
		            EDMA3_DRV_Handle hEdma,
		            unsigned int* srcBuff,
		            unsigned int* dstBuff,
		            unsigned int acnt,
		            unsigned int bcnt,
		            unsigned int ccnt,
		            EDMA3_DRV_SyncType syncType,
		            unsigned long* totalTime);


#endif /* PCIEEDMA_H_ */
