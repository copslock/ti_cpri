/**
 *   @file  test_insRegion.c
 *
 *   @brief
 *      This is the QMSS unit test code.
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

#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#else
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif
#include <string.h>
#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif
/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

#ifndef __LINUX_USER_SPACE
/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>
/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>
#endif

#ifdef __LINUX_USER_SPACE
void initRm();
#else
#define DSP_USE_RM 1 /* instantiate rm locally */
#endif
#define USE_RM 1


/************************ USER DEFINES ********************/

#define  DESC_SIZE          64
#define  REGION0_BASE(x)    x
#define  REGION0_NUM_DESCS  64
#define  REGION0_START_IDX  0
#define  REGION1_BASE(x)    (REGION0_BASE(x) + (REGION0_NUM_DESCS*DESC_SIZE))
#define  REGION1_NUM_DESCS  128
#define  REGION1_START_IDX  (REGION0_START_IDX+REGION0_NUM_DESCS)
#define  REGION2_BASE(x)    (REGION1_BASE(x) + (REGION1_NUM_DESCS*DESC_SIZE))
#define  REGION2_NUM_DESCS  64
#define  REGION2_START_IDX  (REGION1_START_IDX+REGION1_NUM_DESCS)
#define  REGION3_BASE(x)    (REGION2_BASE(x) + (REGION2_NUM_DESCS*DESC_SIZE))
#define  REGION3_NUM_DESCS  64
/*       REGION3_START_IDX calc dynamically */
#define  REGION4_BASE(x)    (REGION3_BASE(x) + (REGION3_NUM_DESCS*DESC_SIZE))
#define  REGION4_NUM_DESCS  64
#define  REGION4_START_IDX  (QMSS_START_INDEX_NOT_SPECIFIED)
#define  REGION5_BASE(x)    (REGION4_BASE(x) + (REGION4_NUM_DESCS*DESC_SIZE))
#define  REGION5_NUM_DESCS  64
#define  REGION5_START_IDX  (QMSS_START_INDEX_INTERNAL)
#define  REGION6_BASE(x)    (REGION5_BASE(x) + (REGION5_NUM_DESCS*DESC_SIZE))
#define  REGION6_NUM_DESCS  64
#define  REGION6_START_IDX  (QMSS_START_INDEX_EXTERNAL)

#define  MEM_POOL_SIZE (REGION6_BASE(0) + (REGION6_NUM_DESCS*DESC_SIZE))

#define  EXT_LINKING_RAM_SIZE 0x1000 /* descriptors */
#define  MAX_DESC_TOT_FAKE (1024*1024*2) /* bigger than any device */
#define  MAX_DESC_REG_FAKE (1024*1024)   /* fits in MAX_DESC_FAKE */


/************************ GLOBAL VARIABLES ********************/

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
/* Descriptor pool [Size of descriptor * Number of descriptors] */
#pragma DATA_ALIGN (memPool, 16)
uint8_t  memPool[MEM_POOL_SIZE];
/* linking RAM */
#pragma DATA_ALIGN (linkingRAM1, 16)
uint64_t                linkingRAM1[EXT_LINKING_RAM_SIZE];
/* rely on linux to set up linking RAM when running in linux */
#else
uint8_t  memPool[MEM_POOL_SIZE] __attribute__ ((aligned (16)));
uint64_t  linkingRAM1[EXT_LINKING_RAM_SIZE] __attribute__ ((aligned (16)));
#endif
#else
uint8_t *memPool;
#endif

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;
/* Store the queue handle for destination queues on which allocated descriptors are stored */
Qmss_QueueHnd           QueHnd[QMSS_MAX_MEM_REGIONS];
/* Total tests */
uint32_t                testCount = 0;
#if DSP_USE_RM
Rm_Handle               rmHandle = NULL;
#endif

/************************ EXTERN VARIABLES ********************/
/* Error counter */
#if DSP_USE_RM
/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
#endif
extern uint32_t                 errorCount;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/**
 *  @b Description
 *  @n
 *      Utility function which converts a local GEM L2 memory address
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */
#ifndef __LINUX_USER_SPACE
static uint32_t l2_global_address (uint32_t addr)
{
#ifdef _TMS320C6X
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}
#else
static uint32_t l2_global_address (uint32_t addr)
{
    return addr;
}
#endif

static void validate (const char *msg, Qmss_Result expectedResult)
{
    int corenum;
#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
#else
    corenum = 0;
#endif
    Qmss_Result result;
    result = Qmss_insertMemoryRegion (&memInfo);

    if (expectedResult == QMSS_SOK)
    {
        if (result < QMSS_SOK)
        {
            System_printf ("Error Core %d: %s: failure %d but expected success\n",
                           corenum, msg, result);
            errorCount++;
        }
    }
    else if (result != expectedResult)
    {
        System_printf ("Error Core %d: %s: got failure %d "
                       "but expected failure %d\n",
                       corenum, msg, result, expectedResult);
        errorCount++;
    }
    testCount++;
}

/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

#ifdef NSS_LITE
    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set NSS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NSS);

    /* Enable the clocks for NSS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_NSS, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NSS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NSS));
#endif  
  
}

void testInsertMemRegion ()
{
    int                     i;
    Qmss_Result             result;
    uint32_t                corenum;
    uint32_t                intLinkRamSize;
    uint8_t                *memBase;
    Qmss_MemRegInfo         cleanMemInfoReg0;
    Qmss_MemRegInfo         cleanMemInfoReg1;
    Qmss_MemRegInfo         cleanMemInfoReg2;
#ifdef USE_RM
#if !defined(SOC_C6678) && !defined(SOC_C6657) /* ! K1 */
    Qmss_MemRegInfo         cleanMemInfoReg3;
    Qmss_MemRegInfo         cleanMemInfoReg4;
#endif
    Qmss_MemRegInfo         cleanMemInfoReg5;
    Qmss_MemRegInfo         cleanMemInfoReg6;
    Qmss_MemRegCfg          memRegCfg;
#endif
#if DSP_USE_RM
    /* RM configuration */
    Rm_InitCfg           rmInitCfg;
    char                 rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
    Rm_ServiceHandle     *rmServiceHandle;
    int32_t              rmResult;
#endif

    /* Get the core number. */
#ifndef __LINUX_USER_SPACE
    passPowerUp();
#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING DESCRIPTOR MEMORY REGIONS ************\n", corenum);
#if DSP_USE_RM
    /* Create the Server instance */
    memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
    rmInitCfg.instName = rmServerName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", corenum, rmResult);
        errorCount++;
        return;
    }

    rmServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", corenum, rmResult);
        errorCount++;
        return;
    }
#endif

    memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

#ifndef __LINUX_USER_SPACE
    /* Set up both internal and external linking RAM
     * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
     * Linking RAM1 is not used */
    #ifndef NSS_LITE 
    qmssInitConfig.linkingRAM0Base = NULL;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = l2_global_address ((uint32_t) linkingRAM1);
    qmssInitConfig.maxDescNum      = MAX_DESC_TOT_FAKE;
    #else
    qmssInitConfig.linkingRAM0Base = NULL;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = 0x800;
    #endif

    /* Negative test case -- call Qmss_start without init */
    result = Qmss_start ();
    if (result != QMSS_NOT_INITIALIZED)
    {
        System_printf ("Error Core %d : negative test of Qmss_start failed : %d\n", corenum, result);
        errorCount++;
    }
#if DSP_USE_RM
    qmssGblCfgParams.qmRmServiceHandle = rmServiceHandle;
#endif
    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", corenum, result);
        errorCount++;
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
    }
#else
    initRm();
    initQmss(MAX_DESC_TOT_FAKE, true);
    /* Allocate memory for descriptor region */
    memPool = (uint8_t *)fw_memAlloc(MEM_POOL_SIZE, CACHE_LINESZ);
    if (memPool == NULL) {
        System_printf ("Core %d : Error allocating monolithic descriptor memory\n", corenum);
        return;
    }
#endif
    intLinkRamSize = qmssLObj[Qmss_SubSys_GLOBAL].p.intLinkRamSize;
    /* Setup memory region for monolithic descriptors */
    memBase = (uint8_t *)l2_global_address ((uint32_t) memPool);
    memset (&cleanMemInfoReg0, 0, sizeof(cleanMemInfoReg0));

    /* Set up template configuration using region 0 */
    cleanMemInfoReg0.descBase       = (uint32_t *)(REGION0_BASE(memBase));
    cleanMemInfoReg0.startIndex     = REGION0_START_IDX;
    cleanMemInfoReg0.descNum        = REGION0_NUM_DESCS;
    cleanMemInfoReg0.memRegion      = Qmss_MemRegion_MEMORY_REGION0;
    cleanMemInfoReg0.descSize       = DESC_SIZE;
    cleanMemInfoReg0.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    /* Set up template configuration using region 1 */
    cleanMemInfoReg1                = cleanMemInfoReg0;
    cleanMemInfoReg1.descBase       = (uint32_t *)(REGION1_BASE(memBase));
    cleanMemInfoReg1.startIndex     = REGION1_START_IDX;
    cleanMemInfoReg1.descNum        = REGION1_NUM_DESCS;
    cleanMemInfoReg1.memRegion      = Qmss_MemRegion_MEMORY_REGION1;
    /* Set up template configuration using region 2 */
    cleanMemInfoReg2                = cleanMemInfoReg0;
    cleanMemInfoReg2.descBase       = (uint32_t *)(REGION2_BASE(memBase));
    cleanMemInfoReg2.startIndex     = REGION2_START_IDX;
    cleanMemInfoReg2.descNum        = REGION2_NUM_DESCS;
    cleanMemInfoReg2.memRegion      = Qmss_MemRegion_MEMORY_REGION2;
#if USE_RM
#if !defined(SOC_C6678) && !defined(SOC_C6657) /* ! K1 */
    /* K1 devices have ordering contraint.  Region 3 and 4 cause
     * ordering violation, so exclude these two */
    /* Set up template configuration using region 3 - spanning start index */
    cleanMemInfoReg3                = cleanMemInfoReg0;
    cleanMemInfoReg3.descBase       = (uint32_t *)(REGION3_BASE(memBase));
    /* force it to span internal and external */
    cleanMemInfoReg3.startIndex     = intLinkRamSize - (REGION3_NUM_DESCS >> 1);
    cleanMemInfoReg3.descNum        = REGION3_NUM_DESCS;
    cleanMemInfoReg3.memRegion      = Qmss_MemRegion_MEMORY_REGION3;
    /* Set up template configuration using region 4 - allocate anywhere */
    cleanMemInfoReg4                = cleanMemInfoReg0;
    cleanMemInfoReg4.descBase       = (uint32_t *)(REGION4_BASE(memBase));
    cleanMemInfoReg4.startIndex     = REGION4_START_IDX;
    cleanMemInfoReg4.descNum        = REGION4_NUM_DESCS;
    cleanMemInfoReg4.memRegion      = Qmss_MemRegion_MEMORY_REGION4;
#endif /* ! K1 */
    /* Set up template configuration using region 5 - allocate internal */
    cleanMemInfoReg5                = cleanMemInfoReg0;
    cleanMemInfoReg5.descBase       = (uint32_t *)(REGION5_BASE(memBase));
    cleanMemInfoReg5.startIndex     = REGION5_START_IDX;
    cleanMemInfoReg5.descNum        = REGION5_NUM_DESCS;
    cleanMemInfoReg5.memRegion      = Qmss_MemRegion_MEMORY_REGION5;
    /* Set up template configuration using region 6 - allocate external */
    cleanMemInfoReg6                = cleanMemInfoReg0;
    cleanMemInfoReg6.descBase       = (uint32_t *)(REGION6_BASE(memBase));
    cleanMemInfoReg6.startIndex     = REGION6_START_IDX;
    cleanMemInfoReg6.descNum        = REGION6_NUM_DESCS;
    cleanMemInfoReg6.memRegion      = Qmss_MemRegion_MEMORY_REGION6;
#endif /* USE_RM */

    memInfo             = cleanMemInfoReg1;
    memInfo.descSize    = 1;
    validate ("Checking bad descriptor size (1)",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descSize    = 63;
    validate ("Checking bad descriptor size (63)",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descNum     = 1;
    validate ("Checking bad number of descriptors (1)",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descNum     = 1;
    validate ("Checking bad number of descriptors (63)",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descNum     = 96; // should be power of 2
    validate ("Checking bad number of descriptors (96)",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descBase++;
    validate ("Checking unaligned descriptor base",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.descBase++;
    validate ("Checking unaligned start index",
              QMSS_INVALID_PARAM);

    memInfo             = cleanMemInfoReg1;
    memInfo.memRegion   = (Qmss_MemRegion)QMSS_MAX_MEM_REGIONS;
    validate ("Checking invalid memory region #",
              USE_RM ? QMSS_RESOURCE_MEM_REGION_INIT_DENIED : QMSS_MEMREGION_INVALID_INDEX);

    memInfo             = cleanMemInfoReg1;
    validate ("Good configuration of region 1",
              QMSS_SOK);

    validate ("Bad duplicate configuration into region 1",
              QMSS_MEMREGION_ALREADY_INITIALIZED);

    /* Now test errors regarding region 0 */
    memInfo             = cleanMemInfoReg0;
    memInfo.descBase    = (uint32_t *)(REGION1_BASE(memBase)) + 16;
    validate ("Test new region start overlap region 1",
              QMSS_MEMREGION_OVERLAP);

    memInfo             = cleanMemInfoReg0;
    memInfo.descBase    = (uint32_t *)(REGION1_BASE(memBase)) - 16;
    validate ("Test new region end overlap region 1",
              QMSS_MEMREGION_OVERLAP);

    memInfo             = cleanMemInfoReg0;
    memInfo.descBase    = (uint32_t *)(REGION1_BASE(memBase)) - 16;
    memInfo.descNum     = 256;
    validate ("Test region 1 base addr completely inside region 0",
              USE_RM ? QMSS_RESOURCE_LINKING_RAM_INIT_DENIED : QMSS_MEMREGION_OVERLAP);

    memInfo             = cleanMemInfoReg0;
    memInfo.startIndex  = REGION1_START_IDX + 32;
    validate ("Test new region start index overlap region 1",
              QMSS_MEMREGION_OVERLAP);

    memInfo             = cleanMemInfoReg0;
    memInfo.startIndex  = REGION1_START_IDX - 32;
    validate ("Test new region end index overlap region 1",
              USE_RM ? QMSS_RESOURCE_LINKING_RAM_INIT_DENIED : QMSS_MEMREGION_OVERLAP);

    memInfo             = cleanMemInfoReg0;
    memInfo.startIndex  = REGION1_START_IDX - 32;
    memInfo.descNum     = 256;
    validate ("Test region 1 index completely inside region 0",
              USE_RM ? QMSS_RESOURCE_LINKING_RAM_INIT_DENIED : QMSS_MEMREGION_OVERLAP);

    /* Peek inside the object to see if ordered regions need to be tested */
    if (qmssLObj[Qmss_SubSys_GLOBAL].p.orderedMemReg)
    {
        memInfo             = cleanMemInfoReg0;
        memInfo.startIndex  = REGION2_START_IDX;
        validate ("Validate start index strictly increasing region 0",
                  QMSS_MEMREGION_ORDERING);

        memInfo             = cleanMemInfoReg0;
        memInfo.descBase    = (uint32_t *)(REGION2_BASE(memBase));
        validate ("Validate base address strictly increasing for region 0",
                  QMSS_MEMREGION_ORDERING);

        /* Now test errors regarding region 2 */
        memInfo             = cleanMemInfoReg2;
        memInfo.startIndex  = REGION0_START_IDX;
        validate ("Validate start index strictly increasing region 2",
                  QMSS_MEMREGION_ORDERING);

        memInfo             = cleanMemInfoReg2;
        memInfo.descBase    = (uint32_t *)(REGION0_BASE(memBase));
        validate ("Validate base address strictly increasing for region 2",
                  QMSS_MEMREGION_ORDERING);
    }

    /* Now insert region 0 and 2 to make sure the configurations actually pass */
    memInfo             = cleanMemInfoReg0;
    validate ("Good configuration of region 0",
              QMSS_SOK);

    memInfo             = cleanMemInfoReg2;
    validate ("Good configuration of region 2",
              QMSS_SOK);

#ifdef USE_RM
#if !defined(SOC_C6678) && !defined(SOC_C6657) /* ! K1 */
    /* test regions 3-6 which use dynamic start index allocation which
     * requires RM */
#ifndef NSS_LITE  /* NSS_LITE device does not support external linking RAM */   
    memInfo             = cleanMemInfoReg3;
    /* enough to fail allocation for all resource tables; tests cleanup of first
     * part of allocaton */
    memInfo.descNum     = MAX_DESC_REG_FAKE;
    validate ("Bad configuration of region 3",
              QMSS_RESOURCE_LINKING_RAM_INIT_DENIED);

    memInfo             = cleanMemInfoReg3;
#if (defined(__LINUX_USER_SPACE) && \
     (defined(SOC_K2L) || defined(SOC_K2E) || \
      defined(SOC_K2H) || defined(SOC_K2K) ))
              /* These devices cant test spanning on linux, because
               * linux takes beginning of external (k2l/k2e) or end
               * of internal (k2h/k2k).  If this changes,
               * remove this #ifdef along with REGION3_VALID */
    validate ("Bad configuration of region 3 (linux collision)",
              QMSS_RESOURCE_LINKING_RAM_INIT_DENIED);
#else
#define REGION3_VALID
    validate ("Good configuration of region 3",
              QMSS_SOK);
#endif
#endif
    memInfo             = cleanMemInfoReg4;
    validate ("Good configuration of region 4",
              QMSS_SOK);
#endif /* ! K1 */

    memInfo             = cleanMemInfoReg5;
    validate ("Good configuration of region 5",
              QMSS_SOK);

#ifndef NSS_LITE  /* NSS_LITE device does not support external linking RAM */   
    memInfo             = cleanMemInfoReg6;
    validate ("Good configuration of region 6",
              QMSS_SOK);
#endif
    /* check startIndex landed in right spots */
    if ( (result = Qmss_getMemoryRegionCfg (&memRegCfg)) != QMSS_SOK)
    {
        System_printf ("Core %d : getMemoryRegionCfg failed %d\n", corenum, result);
        errorCount++;
    }
#if !defined(SOC_C6678) && !defined(SOC_C6657) /* ! K1 */
#if defined(REGION3_VALID) && !defined(NSS_LITE)
    if (memRegCfg.memRegInfo[3].startIndex >= intLinkRamSize)
    {
        System_printf ("Core %d : reg 3 startIdx (%d) ext expect int (<%d)\n",
                       corenum, memRegCfg.memRegInfo[3].startIndex,
                       intLinkRamSize);
        errorCount++;
    }
#endif
    if (memRegCfg.memRegInfo[4].startIndex >= intLinkRamSize)
    {
        System_printf ("Core %d : reg 4 startIdx (%d) ext expect int (<%d)\n",
                       corenum, memRegCfg.memRegInfo[4].startIndex,
                       intLinkRamSize);
        errorCount++;
    }
#endif /* ! K1 */
    if (memRegCfg.memRegInfo[5].startIndex >= intLinkRamSize)
    {
        System_printf ("Core %d : reg 5 startIdx (%d) ext expect int (<%d)\n",
                       corenum, memRegCfg.memRegInfo[5].startIndex,
                       intLinkRamSize);
        errorCount++;
    }
#ifndef NSS_LITE  /* NSS_LITE device does not support external linking RAM */   
    if (memRegCfg.memRegInfo[6].startIndex < intLinkRamSize)
    {
        System_printf ("Core %d : reg 6 startIdx (%d) int expect ext (>=%d)\n",
                       corenum, memRegCfg.memRegInfo[6].startIndex,
                       intLinkRamSize);
        errorCount++;
    }
#endif
#endif

    /* Now clear the regions */
    for (i = 0; i < ((USE_RM) ? 7 : 3); i++)
    {
#if (defined(USE_RM) && !defined(REGION3_VALID))
        if (i == 3)
        {
            continue;
        }
#endif
#if defined(SOC_C6678) || defined(SOC_C6657) /* K1 */
	if ( (i == 3) || (i == 4) )
	{
            continue;
	}
#endif /* K1 */

#ifdef NSS_LITE
        if (i == 6)
        {
            continue;
        }
#endif
        if ( (result = Qmss_removeMemoryRegion (i, 0)) != QMSS_SOK)
        {
            System_printf ("Error removing memory region %d: %d\n", i, result);
            errorCount++;
        }
    }

    System_printf ("Core %d : exit QM\n", corenum);
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error exiting QM: %d \n", corenum, result);
    }

    System_printf("Total tests: %d; %d passed; %d failed\n",
                  testCount, testCount - errorCount, errorCount);
#if RM
    {
        int32_t              rmResult;

        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            System_printf ("Error Core %d : Number of unfreed resources : %d\n", corenum, rmResult);
            errorCount++;
        }
        else
            System_printf ("Core %d : All resources freed successfully\n", corenum);
    }
#endif
    if (errorCount == 0)
    {
        System_printf ("\nCore %d : Memory region configuration tests Passed\n", corenum);
    }
    else
    {
        System_printf ("Core %d test failed with %d errors\n", corenum, errorCount);
    }
}

void run_test (void)
{
    testInsertMemRegion ();
}


