/******************************************************************************
 * FILE PURPOSE:  Main function routine for NWAL unit test
 ******************************************************************************
 * FILE NAME:   fw_main.c
 *
 * DESCRIPTION: Header file for unit test package
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
#include "fw_test.h"


#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/GateHwi.h>
#include <ti/csl/csl_cacheAux.h>
//#include <ti/csl/csl_psc.h>
//#include <ti/csl/csl_pscAux.h>
//#include <ti/csl/cslr_device.h>

void  nwalTest(UArg a0, UArg a1);

Void topLevelTest (UArg a0, UArg a1);


/* The exit code is a global. This is used so
 * the clock function can terminate the program with
 * the proper exit code */
Int exitCode;

/*****************************************************************************
 * FUNCTION PURPOSE: main() function
 *****************************************************************************
 * DESCRIPTION: Creates a single task - the top level task. This is a
 *              low priority task that spawns the individual tests
 *****************************************************************************/
extern cregister volatile unsigned int TSCL;
Void main ()
{
    Task_Params tparams;

    /* The only initial task is the top level test */
    Task_Params_init (&tparams);
    tparams.instance->name = "TopLevelTest";
    tparams.priority      = 1;
    tparams.arg0     = NULL;
    tparams.arg1 = NULL;
    tparams.stackSize =  0x2000;
    Task_create (topLevelTest, &tparams, NULL);

    /* Start the cycle counter */
    TSCL = 1;

    BIOS_start ();
}


Void clk1Fxn (UArg a0)
{
    BIOS_exit (exitCode);
}
#if 0
/********************************************************************s**********
 * FUNCTION PURPOSE: Configure power domain
 ******************************************************************************
 * DESCRIPTION: function used to power on a domain and the module clock.
 *              Power domain is OFF by default. It needs to be turned on
 *              before doing any  device register access.
 *
 * Input:
 *        uint16_t psc_pd    : PSC Power Domain Assignment  (PSC_PD)
 *        uint16_t psc_lpsc  : PSC LPSC Module Assignment   (PSC_LPSC)
 *****************************************************************************/
nwal_RetValue testNwDomainPowerUp(uint16_t psc_pd, uint16_t psc_lpsc)
{

  /* Set Power domain to ON */
  CSL_PSC_enablePowerDomain (psc_pd);

  /* Enable the clocks */
  CSL_PSC_setModuleNextState (psc_lpsc, PSC_MODSTATE_ENABLE);

  /* Start the state transition */
  CSL_PSC_startStateTransition (psc_pd);

  /* Wait until the state transition process is completed. */
  while (!CSL_PSC_isStateTransitionDone (psc_pd));

  /* Return PSC status */
  if ((CSL_PSC_getPowerDomainState(psc_pd) == PSC_PDSTATE_ON) &&
      (CSL_PSC_getModuleState (psc_lpsc) == PSC_MODSTATE_ENABLE))
  {
      printf("testNwDomainPowerUp returning OK\n");
      /* Power ON. Ready for use */
      return nwal_OK;
  }
  else
  {
      printf("testNwDomainPowerUp returning -16\n");
      /* Power on failed. Return error */
      return nwal_ERR_POWER_DOMAIN_FAIL;
  }
}

/******************************************************************************
 * FUNCTION PURPOSE: Power on PASS domains
 ******************************************************************************
 * DESCRIPTION: Function used to power up all necessary PASS domains
 *****************************************************************************/
nwal_RetValue testNwPassPowerUp(nwalGlobContext_t*   pIhandle,
                               nwal_Bool_t          enableSA)
{
  nwal_RetValue retVal = nwal_OK;

  if(pIhandle->cfg.paPowerOn == nwal_FALSE)
  {
      retVal = testNwDomainPowerUp(CSL_PSC_PD_NETCP,
                                   CSL_PSC_LPSC_PA);
      if(retVal != nwal_OK)
      {
        return retVal;
      }
      retVal = testNwDomainPowerUp(CSL_PSC_PD_NETCP,
                                   CSL_PSC_LPSC_CPGMAC);
      if(retVal != nwal_OK)
      {
        return retVal;
      }
  }
  
  if(enableSA)
  {
      if(pIhandle->cfg.saPowerOn == nwal_FALSE)
      {
          retVal = testNwDomainPowerUp(CSL_PSC_PD_NETCP,
                                       CSL_PSC_LPSC_SA);
          if(retVal != nwal_OK)
          {
            return retVal;
          }
      }
  }
  return retVal;
}
#endif
/*****************************************************************************
 * FUNCTION PURPOSE: topLevelTest() function
 *****************************************************************************
 * DESCRIPTION: Initialize the test framework and launch the individual tests
 *****************************************************************************/
Void topLevelTest (UArg a0, UArg a1)
{
    Task_Params tparams;
    Task_Handle thandle;
    Task_Stat   tstat;

    Clock_Handle clkh;
    Clock_Params clkParams;


    /* For some reason some printfs are lost unless there is a delay between System_flush
     * and System_exit, so delay is forced */
    Clock_Params_init(&clkParams);
    clkParams.period    = 0;
    clkParams.startFlag = nwal_FALSE;
    clkh = Clock_create(clk1Fxn, 1, &clkParams, NULL);

#ifdef DEVICE_UNDEFINED
    CACHE_setL2Size (CACHE_32KCACHE);
#else
#ifdef NWAL_TEST_DESC_GLOB_MEM
    CACHE_setL2Size (CACHE_32KCACHE);
#else
    /* Descriptors and Buffers for local core in L2. Disable the
     * cache for extra room
     */
    /* Disable L1 and L2 Cache */
    CACHE_wbAllL1d (CACHE_WAIT);
    CACHE_setL1DSize(CACHE_L1_0KCACHE);
    CACHE_setL1PSize(CACHE_L1_0KCACHE);
    CACHE_setL2Size(CACHE_0KCACHE);
#endif
#endif
    

    if (testNwInit ())  {
        System_printf ("testNwInit (%s:%d): setupTestFramework returned error, exiting\n", __FILE__, __LINE__);
        System_flush ();
        exitCode = -1;
        Clock_start(clkh);
        Task_exit ();
    }


    /* Configure task parameters common to all test tasks */
    Task_Params_init (&tparams);
    tparams.priority = 2;
    tparams.arg0     = NULL;
    tparams.arg1 = NULL;
    tparams.instance->name = "nwalTest";
    tparams.stackSize =  0x2000;

    thandle = Task_create (nwalTest, &tparams, NULL);
    /* The test task will terminate upon completion. Verify that the
     * task has completed in case the task itself uses multiple tasks
     * that all wind up idle for a while. */
    do  {
        Task_stat (thandle, &tstat);
    } while (tstat.mode != Task_Mode_TERMINATED);

    Task_delete (&thandle);

    System_flush();
    #if 0
    Clock_start(clkh);
    #endif
    Task_exit();
}




