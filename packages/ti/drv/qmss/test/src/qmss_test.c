/**
 *   @file  qmss_test.c
 *
 *   @brief
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2012, Texas Instruments, Inc.
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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Mmu.h>
#endif

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#ifdef __ARM_ARCH_7A__
#include <ti/csl/cslr_msmc.h>
#endif


/* OSAL includes */
#include <qmss_osal.h>

/************************ GLOBAL VARIABLES ********************/

/* Error counter */
UInt32                  errorCount = 0;

/************************ EXTERN VARIABLES ********************/

extern Void run_test (Void);

Void main (Void)
{
    UInt32                  corenum;

#ifdef __ARM_ARCH_7A__
    /* Add MMU entries for MMR's required for PCIE example */
    Uint32 privid, index;
    CSL_MsmcRegs *msmc = (CSL_MsmcRegs *)CSL_MSMC_CFG_REGS;
    Mmu_DescriptorAttrs attrs;
    extern char ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;
    uint32_t addr = (uint32_t)&ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;

    Mmu_initDescAttrs(&attrs);

    attrs.type = Mmu_DescriptorType_TABLE;
    attrs.shareable = 0;            // non-shareable
    attrs.accPerm = 1;              // read/write at any privelege level
    attrs.attrIndx = 0;             // Use MAIR0 Register Byte 3 for
                                    // determining the memory attributes
                                    // for each MMU entry


    // Update the first level table's MMU entry for 0x80000000 with the
    // new attributes.
    Mmu_setFirstLevelDesc((Ptr)0x40000000, (UInt64)addr, &attrs);

    // Set up SES & SMS to make all masters coherent
    for (privid = 0; privid < 16; privid++)
    {
      for (index = 0; index < 8; index++)
      {
        uint32_t ses_mpaxh = msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH;
        uint32_t sms_mpaxh = msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH;
        if (CSL_FEXT (ses_mpaxh, MSMC_SES_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          ses_mpaxh &= ~0x80;
          msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH = ses_mpaxh;
        }
        if (CSL_FEXT (sms_mpaxh, MSMC_SMS_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          sms_mpaxh &= ~0x80;
          msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH = sms_mpaxh;
        }
      }
    }
#endif

    System_printf ("***********************************************************\n");
    System_printf ("******************* QMSS LLD Testing **********************\n");
    System_printf ("***********************************************************\n");

    /* Get the core number. */
#ifdef _TMS320C6X
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
#if 1
    System_printf ("********************Test running on Core %d ********************\n", corenum);

    System_printf ("Version string: %s\n", Qmss_getVersionStr());
    run_test ();
#endif

    System_printf ("***********************************************************\n");
    System_printf ("******************** QMSS LLD Testing DONE ****************\n");
    System_printf ("***********************************************************\n");

    if (errorCount == 0)
        System_printf ("\nQMSS LLD ALL TESTS PASSED\n");
    else
        System_printf ("\nQMSS LLD %d TESTS FAILED\n", errorCount);
}

