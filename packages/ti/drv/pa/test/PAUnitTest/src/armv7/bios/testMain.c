
/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/
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

#include "../../pautest.h"

#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Mmu.h>
#endif

#ifdef __ARM_ARCH_7A__
#include <ti/csl/cslr_msmc.h>
#endif

#define PAU_TEST_REPETITION   1
volatile int iteration;


/* NULL terminated The list of tests */
paTest_t  paTestList[] = {
#ifdef NSS_GEN2
#ifndef __LINUX_USER_SPACE
  { paTestEOAMFlow,       "Pa_addEoamFlow and Ethernet OAM Support",    PA_TEST_NOT_RUN },
#endif
#endif
	{ paTestUnconfigured, 	"Packet reception while unconfigured",        PA_TEST_NOT_RUN },
	{ paTestSrioRouting,    "Pa_addSrio and SRIO routing",                PA_TEST_NOT_RUN },
	{ paTestL2Routing,    	"Pa_addMac and L2 routing",                   PA_TEST_NOT_RUN },
	{ paTestL3Routing, 		  "Pa_addIp and L3 Routing",			              PA_TEST_NOT_RUN },
#ifdef NSS_GEN2
	{ paTestACL, 		        "Pa_addAcl and ACL filtering",		            PA_TEST_NOT_RUN },
  { paTestACLRescore,     "Pa_addAcl and ACL filtering with Rescoring", PA_TEST_NOT_RUN },
#endif
	{ paTestL4Routing, 		  "Pa_addPort and L4 Routing", 		              PA_TEST_NOT_RUN },
	{ paTestPatchRoute,     "Blind patch and route",                      PA_TEST_NOT_RUN },
	{ paTestTxFmtRt,      	"Tx checksum and routing",                    PA_TEST_NOT_RUN },
	{ paTestCustom,			    "Custom routing",					                    PA_TEST_NOT_RUN },
	{ paTestMultiRouting,   "Multi-routing",					                    PA_TEST_NOT_RUN },
	{ paTestIPv4FragReassem,"IPv4 Fragmentation and Reassembly",          PA_TEST_NOT_RUN },
	{ paTestIPv6FragReassem,"IPv6 Fragmentation and Reassembly",          PA_TEST_NOT_RUN },
#ifdef NSS_GEN2
	{ paTestEflow,  	      "Egress Flow and Packet Forwarding Test",     PA_TEST_NOT_RUN },
#endif
 	{ paTestUnconfigured, 	"Packet reception while unconfigured",        PA_TEST_NOT_RUN },
	{ NULL,                 NULL,                                         PA_TEST_NOT_RUN }
};

#define PAU_NUM_TESTS       ((sizeof(paTestList)/sizeof(paTest_t)) - 1)

void topLevelTest (UArg a0, UArg a1);

/* The exit code is a global. This is used so
 * the clock function can terminate the program with
 * the proper exit code */
int exitCode;

/* Creates a single task - the top level task. This is a low priority task that
 * spawns the individual tests */
#ifdef _TMS320C6X
extern cregister volatile unsigned int TSCL;
#endif

void main ()
{
	Task_Params tparams;

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

    /* The only initial task is the top level test */
	Task_Params_init (&tparams);
	tparams.instance->name = "Top Level Test";
	tparams.priority      = 1;

	Task_create (topLevelTest, &tparams, NULL);

#ifdef _TMS320C6X
	/* Start the cycle counter */
	TSCL = 1;
#endif

	BIOS_start ();
}


void clk1Fxn (UArg a0)
{
	BIOS_exit (exitCode);
}

/* Initialize the test framework and launch the individual tests */
void topLevelTest (UArg a0, UArg a1)
{
	Task_Params tparams;
	Task_Handle thandle;
	Task_Stat   tstat;

	Clock_Handle clkh;
    Clock_Params clkParams;

	int i;

	int passCount;
	int failCount;
	int notRunCount;

	/* For some reason some printfs are lost unless there is a delay between System_flush
	 * and System_exit, so delay is forced */
	Clock_Params_init(&clkParams);
    clkParams.period    = 0;
    clkParams.startFlag = FALSE;
    clkh = Clock_create(clk1Fxn, 1, &clkParams, NULL);

    System_printf ("\n\n ------- PA Unit Test Starting ---------\n");

    /* Initialize the PA, PA cpdma, QM and CPPI. Each test will use
     * the same framework */
    if (setupTestFramework ())  {
    	System_printf ("topLevelTest (%s:%d): setupTestFramework returned error, exiting\n", __FILE__, __LINE__);
    	System_flush ();
    	exitCode = -1;
    	Clock_start(clkh);
    	Task_exit ();
    }

    /* Make sure the setup matches what is expected */
    if (verifyTestFramework())  {
    	System_printf ("topLevelTest (%s:%d): verifyTestFramework returned error after initial framework setup, exiting\n", __FILE__, __LINE__);
    	System_flush();
    	exitCode = -1;
    	Clock_start(clkh);
    	Task_exit ();
    }

    /* Configure task parameters common to all test tasks */
    Task_Params_init (&tparams);
    tparams.priority = 2;
    tparams.arg0     = (UArg) &tFramework;

	/* Run the tests */
  for (iteration = 0; iteration < PAU_TEST_REPETITION; iteration++) {
  	for (i = 0; paTestList[i].testFunction != NULL; i++ )  {

  		tparams.arg1 = (UArg)&paTestList[i];
  		tparams.instance->name = paTestList[i].name;

  		thandle = Task_create (paTestList[i].testFunction, &tparams, NULL);

  		/* The test task will terminate upon completion. Verify that the
  		 * task has completed in case the task itself uses multiple tasks
  		 * that all wind up idle for a while. */
  		do  {
  			Task_stat (thandle, &tstat);
  		} while (tstat.mode != Task_Mode_TERMINATED);

  		Task_delete (&thandle);

  		if (paTestList[i].testStatus == PA_TEST_PASSED)
  		  System_printf ("%s:  PASSED\n", paTestList[i].name);
  		else
  		  System_printf ("%s:  FAILED\n", paTestList[i].name);

  		System_flush();

  		/* Do a quick check of the test framework */
  		if (verifyTestFramework ())  {
  			System_printf ("topLevelTest (%s:%d): verifyTestFramework returned error after test %s. Exiting.\n", __FILE__, __LINE__, paTestList[i].name);
  			exitCode = -1;
  			System_flush ();
  			Clock_start(clkh);
  		}
      }

	/* Summarize the test results */
	for (i = passCount = failCount = notRunCount = 0; paTestList[i].testFunction != NULL; i++)  {
		if (paTestList[i].testStatus == PA_TEST_PASSED)
			passCount += 1;
		else if (paTestList[i].testStatus == PA_TEST_FAILED)
			failCount += 1;
		else
			notRunCount += 1;
	}

	System_printf ("\n\nTest summary:\n\tTests Passed: %d\n\tTests Failed: %d\n\tTests not run: %d\n\n",
	  passCount, failCount, notRunCount);

    if(passCount == PAU_NUM_TESTS)
	    System_printf ("All tests have passed!\n\n");

    if (iteration)
      System_printf(" during iteration count  %d", iteration);
  }
	if (clearTestFramework ())
		System_printf ("\n\n ------- PA Unit Test Clean Failed ---------\n");

	System_printf ("\n\n ------- PA Unit Test Complete ---------\n");

	System_flush();

#if (RM) && !defined(__LINUX_USER_SPACE)
    {
        int32_t rmResult;

        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            System_printf ("Error : Number of unfreed resources : %d\n", rmResult);
            System_flush();
        }
        else
        {
            System_printf ("All resources freed successfully\n");
            System_flush();
        }
    }
#endif

	Clock_start(clkh);

	Task_exit();
}




