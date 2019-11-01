/** 
 * @file test_lld.c
 *
 * @brief 
 *  FFTC MMR test code using FFTC LLD APIs.
 *  
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2008, 2009, Texas Instruments, Inc.
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
/* FFTC test application include */
#include "fftc_test.h"

#undef		FFTC_TEST_DEBUG

/**************************************************************
************************* GLOBAL VARIABLES ********************
***************************************************************/
extern UInt32 Fftc_DftBlockSizeTable[50];
extern UInt32 totalNumTestsPass, totalNumTestsFail;


/**************************************************************
************************* FUNCTIONS  **************************
***************************************************************/

static Void test_fftc_pidReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_PeripheralIdParams         pidCfg_r;

    /* Test FFTC PID Register Read and verify Defaults */
    if (Fftc_readPidReg (pFFTCLldObj, &pidCfg_r) != 0)
    {
        testsFailed ++;
    }
    else
    {
        /* Verify PID Register defaults */            
        if ((pidCfg_r.function != CSL_FFTC_PID_PID_RESETVAL) || (pidCfg_r.rtlVersion != CSL_FFTC_PID_RTL_RESETVAL) ||
            (pidCfg_r.majorNum != CSL_FFTC_PID_MAJOR_RESETVAL) || (pidCfg_r.customNum != CSL_FFTC_PID_CUSTOM_RESETVAL) ||
            (pidCfg_r.minorNum != CSL_FFTC_PID_MINOR_RESETVAL))
        {
#ifdef FFTC_TEST_DEBUG                
            Fftc_osalLog ("PID Register Defaults Not Correct, "
                   "Read fxn: %d rtlV: %d majNum: %d custNum: %d MinNum: %d \n",
                    pidCfg_r.function, pidCfg_r.rtlVersion, pidCfg_r.majorNum, 
                    pidCfg_r.customNum, pidCfg_r.minorNum
                  );
#endif
            testsFailed ++;
        }
    }

    if (1)
    {
        Fftc_osalLog ("Fftc_readPidReg() test                           PASSED \n");
        totalNumTestsPass ++;
    }

    /* Done testing */
    return;
}


Void test_fftc_configReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_GlobalCfg                  glblCfg_r, glblCfg_w;
    UInt32                          i;        

    /* FFTC Configuration Register Read and Defaults Test */
    if (Fftc_readGlobalConfigReg (pFFTCLldObj, &glblCfg_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        if ((glblCfg_r.starvationPeriodVal != CSL_FFTC_CONFIG_STARVATION_PERIOD_RESETVAL) ||
            (glblCfg_r.bDisableFFT != CSL_FFTC_CONFIG_FFT_DISABLE_RESETVAL) ||
            (glblCfg_r.queueFlowidOverwrite [3] != CSL_FFTC_CONFIG_Q3_FLOWID_OVERWRITE_RESETVAL) ||
            (glblCfg_r.queueFlowidOverwrite [2] != CSL_FFTC_CONFIG_Q2_FLOWID_OVERWRITE_RESETVAL) ||
            (glblCfg_r.queueFlowidOverwrite [1] != CSL_FFTC_CONFIG_Q1_FLOWID_OVERWRITE_RESETVAL) ||
            (glblCfg_r.queueFlowidOverwrite [0] != CSL_FFTC_CONFIG_Q0_FLOWID_OVERWRITE_RESETVAL) ||
            (glblCfg_r.queuePriority [3] != CSL_FFTC_CONFIG_QUEUE_3_PRIORITY_RESETVAL) ||
            (glblCfg_r.queuePriority [2] != CSL_FFTC_CONFIG_QUEUE_2_PRIORITY_RESETVAL) ||
            (glblCfg_r.queuePriority [1] != CSL_FFTC_CONFIG_QUEUE_1_PRIORITY_RESETVAL) ||
            (glblCfg_r.queuePriority [0] != CSL_FFTC_CONFIG_QUEUE_0_PRIORITY_RESETVAL)
            )
        {
#ifdef FFTC_TEST_DEBUG                
            Fftc_osalLog ("FFTC Configuration Register Defaults Not Correct \n");
#endif
            testsFailed ++;
        }    
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_readGlobalConfigReg() test                  FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readGlobalConfigReg() test                  PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    /* FFTC Configuration Register Write Test.
     *
     * Initialize the FFTC Global Configuration 
     */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        glblCfg_w.queueFlowidOverwrite [i] = 1;
        glblCfg_w.queuePriority [i] = 1;
    }
    glblCfg_w.starvationPeriodVal = 0x03;  
    glblCfg_w.bDisableFFT = 1;

    /* Test write */
    if (Fftc_writeGlobalConfigReg (pFFTCLldObj, &glblCfg_w) < 0)
    {
        testsFailed ++;
    }
    else
    {
        /* Verify write */            
        if (Fftc_readGlobalConfigReg (pFFTCLldObj, &glblCfg_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            if ((glblCfg_r.starvationPeriodVal != glblCfg_w.starvationPeriodVal) ||
                (glblCfg_r.bDisableFFT != glblCfg_w.bDisableFFT))
                testsFailed ++;

            for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
            {
                if ((glblCfg_w.queueFlowidOverwrite [i] != glblCfg_r.queueFlowidOverwrite [i]) ||
                    (glblCfg_w.queuePriority [i] != glblCfg_r.queuePriority [i]))
                    testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeGlobalConfigReg() test                 FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeGlobalConfigReg() test                 PASSED \n");
        totalNumTestsPass ++;
    }

    /* Done testing */
    return;
}

static Void test_fftc_controlReg (Fftc_LldObj*        pFFTCLldObj)
{
    /* FFTC Control Register Test */                
    /* No way to verify ?? */
    Fftc_doSoftwareReset (pFFTCLldObj);
    Fftc_osalLog ("Fftc_doSoftwareReset() test                      PASSED \n");
    totalNumTestsPass ++;

    Fftc_doSoftwareContinue (pFFTCLldObj);
    Fftc_osalLog ("Fftc_doSoftwareContinue() test                   PASSED \n");
    totalNumTestsPass ++;

    /* Done testing */
    return;
}

static Void test_fftc_statusReg (Fftc_LldObj*        pFFTCLldObj)
{
    /* FFTC Status Register Test */
    if (Fftc_isHalted (pFFTCLldObj))
    {
        Fftc_osalLog ("Fftc_isHalted() test                             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_isHalted() test                             PASSED \n");
        totalNumTestsPass ++;
    }

}

static Void test_fftc_emulationControlReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_EmulationControlParams     emulCtrl_r, emulCtrl_w;

    /* FFTC Emulation Control Register Test */
    if (Fftc_readEmulationControlReg (pFFTCLldObj, &emulCtrl_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        if ((emulCtrl_r.bEmuRtSel != CSL_FFTC_EMU_CONTROL_EMU_RT_SEL_RESETVAL) ||
            (emulCtrl_r.bEmuSoftStop != CSL_FFTC_EMU_CONTROL_EMU_SOFT_STOP_RESETVAL) ||
            (emulCtrl_r.bEmuFreeRun != CSL_FFTC_EMU_CONTROL_EMU_FREERUN_RESETVAL)
           )
        {
#ifdef FFTC_TEST_DEBUG                
            Fftc_osalLog ("FFTC Emulation Control Register Defaults Not Correct \n");
#endif
            testsFailed ++;
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_readEmulationControlReg() test              FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readEmulationControlReg() test              PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    /* Test emulation control register write and verify it works */
    emulCtrl_w.bEmuRtSel = 1;
    emulCtrl_w.bEmuSoftStop = 1;
    emulCtrl_w.bEmuFreeRun = 1;
    if (Fftc_writeEmulationControlReg (pFFTCLldObj, &emulCtrl_w) < 0)
    {
        testsFailed ++;
    }
    else
    {
        if (Fftc_readEmulationControlReg (pFFTCLldObj, &emulCtrl_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            if ((emulCtrl_r.bEmuRtSel != emulCtrl_w.bEmuRtSel) ||
                (emulCtrl_r.bEmuSoftStop != emulCtrl_w.bEmuSoftStop) ||
                (emulCtrl_r.bEmuFreeRun != emulCtrl_w.bEmuFreeRun)
               )
            {
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeEmulationControlReg() test             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeEmulationControlReg() test             PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_intRawStatusReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_ErrorParams                errParams_r;
    UInt32                          i;

    /* FFTC Error Interrupt Raw Status Register Read Test */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++) 
    {
        if (Fftc_readErrorIntRawStatusReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            switch (i)
            {
                case Fftc_QueueId_0:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T0_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG
                        Fftc_osalLog ("FFTC Error Interrupt Raw Status Register Defaults not correct for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_1:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T1_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG
                        Fftc_osalLog ("FFTC Error Interrupt Raw Status Register Defaults not correct for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_2:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T2_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG
                        Fftc_osalLog ("FFTC Error Interrupt Raw Status Register Defaults not correct for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_3:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T3_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG
                        Fftc_osalLog ("FFTC Error Interrupt Raw Status Register Defaults not correct for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_readErrorIntRawStatusReg() test             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readErrorIntRawStatusReg() test             PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_clearIntRawStatusReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_ErrorParams                errParams_r;
    UInt32                          i;

    /* FFTC Error Interrupt Clear Register Test */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++) 
    {
        /* Clear all error bits */            
        errParams_r.bIsIntOnEOP          = 1;
        errParams_r.bIsDebugHalt         = 1;
        errParams_r.bIsConfigWordError   = 1;
        errParams_r.bIsDescBufferError   = 1;
        errParams_r.bIsEopError          = 1;
        errParams_r.bIsConfigInvalError  = 1;
        if (Fftc_clearErrorIntRawStatusReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            testsFailed ++;
        }

        if (Fftc_readErrorIntRawStatusReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            switch (i)
            {
                case Fftc_QueueId_0:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T0_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T0_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Clear Register test failed for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_1:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T1_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T1_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Clear Register test failed for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_2:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T2_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T2_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Clear Register test failed for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_3:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_STAT_EOP_ERROR_STATUS_T3_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T3_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Clear Register test failed for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_clearErrorIntRawStatusReg() test            FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_clearErrorIntRawStatusReg() test            PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_intEnableSetReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_ErrorParams                errParams_r, errParams_w;
    UInt32                          i;

    /* FFTC Error Interrupt Enable and Set Register */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++) 
    {
        /* Enable all errors */            
        errParams_w.bIsIntOnEOP          = 1;
        errParams_w.bIsDebugHalt         = 1;
        errParams_w.bIsConfigWordError   = 1;
        errParams_w.bIsDescBufferError   = 1;
        errParams_w.bIsEopError          = 1;
        errParams_w.bIsConfigInvalError  = 1;
        if (Fftc_writeErrorIntEnableSetReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_w) < 0)
        {
#ifdef FFTC_TEST_DEBUG                
            Fftc_osalLog ("FFTC Error Interrupt Enable and Set Register Write FAILED for queue: %d \n", i);                
#endif
            testsFailed ++;
        }

        if (Fftc_readErrorIntEnableSetReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            Fftc_osalLog ("Fftc_readErrorIntEnableSetReg() test         FAILED \n");
            testsFailed ++;
        }
        else
        {
            if ((errParams_r.bIsIntOnEOP != errParams_w.bIsIntOnEOP) ||
                (errParams_r.bIsDebugHalt != errParams_w.bIsDebugHalt) ||
                (errParams_r.bIsConfigWordError != errParams_w.bIsConfigWordError) ||
                (errParams_r.bIsDescBufferError != errParams_w.bIsDescBufferError) ||
                (errParams_r.bIsEopError != errParams_w.bIsEopError) ||
                (errParams_r.bIsConfigInvalError != errParams_w.bIsConfigInvalError)
                )
            {
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeErrorIntEnableSetReg() test            FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readErrorIntEnableSetReg() test             PASSED \n");
        Fftc_osalLog ("Fftc_writeErrorIntEnableSetReg() test            PASSED \n");
        totalNumTestsPass ++;
    }

    return;        
}

static Void test_fftc_clearIntEnableSetReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_ErrorParams                errParams_r, errParams_w;
    UInt32                          i;

    /* FFTC Error Interrupt Enable Clear Register Test */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++) 
    {
        /* Clear all errors */            
        errParams_w.bIsIntOnEOP          = 1;
        errParams_w.bIsDebugHalt         = 1;
        errParams_w.bIsConfigWordError   = 1;
        errParams_w.bIsDescBufferError   = 1;
        errParams_w.bIsEopError          = 1;
        errParams_w.bIsConfigInvalError  = 1;
        if (Fftc_clearErrorIntEnableReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_w) < 0)
        {
            testsFailed ++;
        }

        if (Fftc_readErrorIntEnableSetReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            switch (i)
            {
                case Fftc_QueueId_0:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_EN_INT_ON_EOP_EN_T0_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_EN_DEBUG_HALT_EN_T0_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T0_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T0_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_EN_EOP_ERROR_EN_T0_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T0_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Enable Clear Register test FAILED for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_1:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_EN_INT_ON_EOP_EN_T1_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_EN_DEBUG_HALT_EN_T1_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T1_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T1_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_EN_EOP_ERROR_EN_T1_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T1_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Enable Clear Register test FAILED for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_2:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_EN_INT_ON_EOP_EN_T2_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_EN_DEBUG_HALT_EN_T2_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T2_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T2_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_EN_EOP_ERROR_EN_T2_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T2_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Enable Clear Register test FAILED for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
                case Fftc_QueueId_3:
                {
                    if ((errParams_r.bIsIntOnEOP != CSL_FFTC_ERROR_EN_INT_ON_EOP_EN_T3_RESETVAL) ||
                        (errParams_r.bIsDebugHalt != CSL_FFTC_ERROR_EN_DEBUG_HALT_EN_T3_RESETVAL) ||
                        (errParams_r.bIsConfigWordError != CSL_FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T3_RESETVAL) ||
                        (errParams_r.bIsDescBufferError != CSL_FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T3_RESETVAL) ||
                        (errParams_r.bIsEopError != CSL_FFTC_ERROR_EN_EOP_ERROR_EN_T3_RESETVAL) ||
                        (errParams_r.bIsConfigInvalError != CSL_FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T3_RESETVAL)
                       )
                    {
#ifdef FFTC_TEST_DEBUG                            
                        Fftc_osalLog ("FFTC Error Interrupt Enable Clear Register test FAILED for queue: %d \n", i);                            
#endif
                        testsFailed ++;
                    }
                    
                    break;
                }
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_clearErrorIntEnableReg() test               FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_clearErrorIntEnableReg() test               PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_haltOnErrorReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_ErrorParams                errParams_r, errParams_w;
    UInt32                          i;

    /* FFTC Halt on Error Register Test */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++) 
    {
        /* Enable halt for all errors */            
        errParams_w.bIsIntOnEOP          = 1;
        errParams_w.bIsDebugHalt         = 1;
        errParams_w.bIsConfigWordError   = 1;
        errParams_w.bIsDescBufferError   = 1;
        errParams_w.bIsEopError          = 1;
        errParams_w.bIsConfigInvalError  = 1;
        if (Fftc_writeHaltOnErrorReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_w) < 0)
        {
#ifdef FFTC_TEST_DEBUG                
            Fftc_osalLog ("FFTC Halt on Error Register Write FAILED for queue: %d \n", i);                
#endif
            testsFailed ++;
        }

        if (Fftc_readHaltOnErrorReg (pFFTCLldObj, (Fftc_QueueId) i, &errParams_r) < 0)
        {
            Fftc_osalLog ("Fftc_readHaltOnErrorReg() test                   FAILED \n");
            testsFailed ++;
        }
        else
        {
            if ((errParams_r.bIsIntOnEOP != errParams_w.bIsIntOnEOP) ||
                (errParams_r.bIsDebugHalt != errParams_w.bIsDebugHalt) ||
                (errParams_r.bIsConfigWordError != errParams_w.bIsConfigWordError) ||
                (errParams_r.bIsDescBufferError != errParams_w.bIsDescBufferError) ||
                (errParams_r.bIsEopError != errParams_w.bIsEopError) ||
                (errParams_r.bIsConfigInvalError != errParams_w.bIsConfigInvalError)
                )
            {
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeHaltOnErrorReg() test                  FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeHaltOnErrorReg() test                  PASSED \n");
        Fftc_osalLog ("Fftc_readHaltOnErrorReg() test                   PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_eoiReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        

    /* FFTC End of Interrupt Register Test */
    if (Fftc_readEoiReg (pFFTCLldObj) != CSL_FFTC_EOI_RESETVAL)
    {
        Fftc_osalLog ("Fftc_readEoiReg() test                           FAILED \n");
        testsFailed ++;
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readEoiReg() test                           PASSED \n");
        totalNumTestsPass ++;
    }

    /* Test EOI write */
    testsFailed = 0;
    if (Fftc_writeEoiReg (pFFTCLldObj, 1) < 0)
    {
        testsFailed ++;
    }
    else
    {
        if (Fftc_readEoiReg (pFFTCLldObj) != 1)
        {
            testsFailed ++;
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeEoiReg() test                          FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeEoiReg() test                          PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_clippingDetectReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0, i;        

    /* FFTC Queue X Clipping Detect Register */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        if (Fftc_readQueueClippingDetectReg (pFFTCLldObj, (Fftc_QueueId) i) != CSL_FFTC_CLIP_Q_CLIPPING_COUNT_RESETVAL)
        {
            testsFailed ++;
        }
    }
      
    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_readQueueClippingDetectReg() test           FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readQueueClippingDetectReg() test           PASSED \n");
        totalNumTestsPass ++;
    }

    /* Test Clipping detect register clear */
    testsFailed = 0;
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        Fftc_clearQueueClippingDetectReg (pFFTCLldObj, (Fftc_QueueId) i);

        if (Fftc_readQueueClippingDetectReg (pFFTCLldObj, (Fftc_QueueId) i) != CSL_FFTC_CLIP_Q_CLIPPING_COUNT_RESETVAL)
        {
            testsFailed ++;
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_clearQueueClippingDetectReg() test          FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_clearQueueClippingDetectReg() test          PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_qlocalRegs (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    Fftc_QLocalCfg                  qConfig_r, qConfig_w;
    UInt32                          i, tmp;        

    /* 13:  FFTC Queue Local Registers
     *      FFTC Queue X Destination Queue Register
     *      FFTC Queue X Scaling & Shifting Register
     *      FFTC Queue X Cyclic Prefix Register
     *      FFTC Queue X Control Register
     *      FFTC Queue X LTE Frequency Shift Register
     */
    memset (&qConfig_r, 0, sizeof (qConfig_r));
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        if (Fftc_readQueueConfigRegs (pFFTCLldObj, (Fftc_QueueId) i, &qConfig_r) < 0)
        {
            testsFailed ++;
        }
        else
        {
            /* Verify defaults */                
            tmp = 0;                
            if ((qConfig_r.destQRegConfig.bOutputFFTShift != CSL_FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_OUTPUT_RESETVAL) ||
                (qConfig_r.destQRegConfig.bInputFFTShift != CSL_FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_INPUT_RESETVAL) ||
                (qConfig_r.destQRegConfig.inputShiftVal != CSL_FFTC_Q0_DEST_FFTC_VARIABLE_SHIFT_INPUT_RESETVAL) ||
                (qConfig_r.destQRegConfig.cppiDestQNum != CSL_FFTC_Q0_DEST_DEFAULT_DEST_RESETVAL))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Destination Queue Register defaults not correct \n", i);
#endif
            }

            if ((qConfig_r.scalingShiftingRegConfig.bDynamicScaleEnable != CSL_FFTC_Q0_SCALE_SHIFT_DYNAMIC_SCALING_ENABLE_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.outputScaleVal != CSL_FFTC_Q0_SCALE_SHIFT_OUTPUT_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[0] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_0_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[1] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_1_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[2] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_2_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[3] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_3_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[4] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_4_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[5] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_5_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[6] != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_6_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingValLast != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_OUT_SCALING_RESETVAL) ||
                (qConfig_r.scalingShiftingRegConfig.freqShiftScaleVal != CSL_FFTC_Q0_SCALE_SHIFT_STAGE_LTE_SHIFT_SCALING_RESETVAL))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Scaling & Shifting Register defaults not correct \n", i);
#endif
            }

            if ((qConfig_r.cyclicPrefixRegConfig.cyclicPrefixAddNum != CSL_FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION_RESETVAL) ||
                (qConfig_r.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable != CSL_FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_EN_RESETVAL) ||
                (qConfig_r.cyclicPrefixRegConfig.cyclicPrefixRemoveNum != CSL_FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_OFFSET_RESETVAL))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Cyclic Prefix Register defaults not correct \n", i);
#endif
            }

            if ((qConfig_r.controlRegConfig.dftSize != Fftc_DftBlockSizeTable [CSL_FFTC_Q0_CONTROL_DFT_SIZE_RESETVAL]) ||
                (qConfig_r.controlRegConfig.dftMode != CSL_FFTC_Q0_CONTROL_DFT_IDFT_SELECT_RESETVAL) ||
                (qConfig_r.controlRegConfig.zeroPadMode != CSL_FFTC_Q0_CONTROL_ZERO_PAD_MODE_RESETVAL) ||
                (qConfig_r.controlRegConfig.zeroPadFactor != CSL_FFTC_Q0_CONTROL_ZERO_PAD_VAL_RESETVAL) ||
                (qConfig_r.controlRegConfig.bSupressSideInfo != CSL_FFTC_Q0_CONTROL_SUPPRESSED_SIDE_INFO_RESETVAL))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Control Register defaults not correct \n", i);
#endif
            }

            if ((qConfig_r.freqShiftRegConfig.bFreqShiftEnable != CSL_FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_EN_RESETVAL) ||
                (qConfig_r.freqShiftRegConfig.freqShiftIndex != CSL_FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_INDEX_RESETVAL) ||
                (qConfig_r.freqShiftRegConfig.freqShiftMultFactor != CSL_FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_FACTOR_RESETVAL) ||
                (qConfig_r.freqShiftRegConfig.freqShiftInitPhase != CSL_FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_PHASE_RESETVAL) ||
                (qConfig_r.freqShiftRegConfig.freqShiftDirection != CSL_FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_DIR_RESETVAL))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Freq Shift Configuration Register defaults not correct \n", i);
#endif
            }

            if (tmp > 0)
            {
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Configuration Register defaults not correct \n", i);
#endif
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_readQueueConfigRegs() test                  FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readQueueConfigRegs() test                  PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        qConfig_w.destQRegConfig.bOutputFFTShift                    = 0;
        qConfig_w.destQRegConfig.bInputFFTShift                     = 0;
        qConfig_w.destQRegConfig.inputShiftVal                      = 0;
        qConfig_w.destQRegConfig.cppiDestQNum                       = 800;

        qConfig_w.scalingShiftingRegConfig.bDynamicScaleEnable      = 0; /* radix shift vals are applicable only in static scaling mode */
        qConfig_w.scalingShiftingRegConfig.outputScaleVal           = 0x08;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[0]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[1]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[2]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[3]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[4]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[5]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingVal[6]       = 0x02;
        qConfig_w.scalingShiftingRegConfig.radixScalingValLast      = 0x03;
        qConfig_w.scalingShiftingRegConfig.freqShiftScaleVal        = 0x03; /* enable freq shift */

        qConfig_w.cyclicPrefixRegConfig.bCyclicPrefixAddEnable      = 1;
        qConfig_w.cyclicPrefixRegConfig.cyclicPrefixAddNum          = 0x06;
        qConfig_w.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable   = 1;
        qConfig_w.cyclicPrefixRegConfig.cyclicPrefixRemoveNum       = 0x06;

        qConfig_w.controlRegConfig.dftSize                          = 12;
        qConfig_w.controlRegConfig.dftMode                          = Fftc_DFTMode_DFT;
        qConfig_w.controlRegConfig.bEmulateDSP16x16                 = 0;
        qConfig_w.controlRegConfig.bZeroPadEnable                   = 1;
        qConfig_w.controlRegConfig.zeroPadMode                      = Fftc_ZeroPadMode_MULTIPLY;
        qConfig_w.controlRegConfig.zeroPadFactor                    = 0x02;
        qConfig_w.controlRegConfig.bSupressSideInfo                 = 1;

        qConfig_w.freqShiftRegConfig.bFreqShiftEnable               = 1;
        qConfig_w.freqShiftRegConfig.freqShiftIndex                 = Fftc_FreqShiftIndex_12288;
        qConfig_w.freqShiftRegConfig.freqShiftMultFactor            = 0x02;
        qConfig_w.freqShiftRegConfig.freqShiftInitPhase             = 0x02;
        qConfig_w.freqShiftRegConfig.freqShiftDirection             = Fftc_FreqShiftDir_PLUS;        

        if (Fftc_writeQueueConfigRegs (pFFTCLldObj, (Fftc_QueueId) i, &qConfig_w) < 0)
        {
#ifdef FFTC_TEST_DEBUG                    
            Fftc_osalLog ("13: FFTC Queue %d Configuration Register write failed \n", i);
#endif
            testsFailed ++;
        }
        else
        {
            /* verify write */
            memset (&qConfig_r, 0, sizeof (qConfig_r));
            Fftc_readQueueConfigRegs (pFFTCLldObj, (Fftc_QueueId) i, &qConfig_r);

            tmp = 0;                
            if ((qConfig_r.destQRegConfig.bOutputFFTShift != qConfig_w.destQRegConfig.bOutputFFTShift) ||
                (qConfig_r.destQRegConfig.bInputFFTShift != qConfig_w.destQRegConfig.bInputFFTShift) ||
                (qConfig_r.destQRegConfig.inputShiftVal != qConfig_w.destQRegConfig.inputShiftVal) ||
                (qConfig_r.destQRegConfig.cppiDestQNum != qConfig_w.destQRegConfig.cppiDestQNum))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Destination Queue Register write test failed \n", i);
#endif
            }

            if ((qConfig_r.scalingShiftingRegConfig.bDynamicScaleEnable != qConfig_w.scalingShiftingRegConfig.bDynamicScaleEnable) ||
                (qConfig_r.scalingShiftingRegConfig.outputScaleVal != qConfig_w.scalingShiftingRegConfig.outputScaleVal) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[0] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[0]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[1] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[1]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[2] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[2]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[3] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[3]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[4] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[4]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[5] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[5]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingVal[6] != qConfig_w.scalingShiftingRegConfig.radixScalingVal[6]) ||
                (qConfig_r.scalingShiftingRegConfig.radixScalingValLast != qConfig_w.scalingShiftingRegConfig.radixScalingValLast) ||
                (qConfig_r.scalingShiftingRegConfig.freqShiftScaleVal != qConfig_w.scalingShiftingRegConfig.freqShiftScaleVal))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Scaling & Shifting Register write test failed \n", i);
#endif
            }

            if ((qConfig_r.cyclicPrefixRegConfig.cyclicPrefixAddNum != qConfig_w.cyclicPrefixRegConfig.cyclicPrefixAddNum) ||
                (qConfig_r.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable != qConfig_w.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable) ||
                (qConfig_r.cyclicPrefixRegConfig.cyclicPrefixRemoveNum != qConfig_w.cyclicPrefixRegConfig.cyclicPrefixRemoveNum))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Cyclic Prefix Register write test failed \n", i);
#endif
            }

            if ((qConfig_r.controlRegConfig.dftSize != qConfig_w.controlRegConfig.dftSize) ||
                (qConfig_r.controlRegConfig.dftMode != qConfig_w.controlRegConfig.dftMode) ||
                (qConfig_r.controlRegConfig.zeroPadMode != qConfig_w.controlRegConfig.zeroPadMode) ||
                (qConfig_r.controlRegConfig.zeroPadFactor != qConfig_w.controlRegConfig.zeroPadFactor) ||
                (qConfig_r.controlRegConfig.bSupressSideInfo != qConfig_w.controlRegConfig.bSupressSideInfo))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Control Register write test failed \n", i);
#endif
            }

            if ((qConfig_r.freqShiftRegConfig.bFreqShiftEnable != qConfig_w.freqShiftRegConfig.bFreqShiftEnable) ||
                (qConfig_r.freqShiftRegConfig.freqShiftIndex != qConfig_w.freqShiftRegConfig.freqShiftIndex) ||
                (qConfig_r.freqShiftRegConfig.freqShiftMultFactor != qConfig_w.freqShiftRegConfig.freqShiftMultFactor) ||
                (qConfig_r.freqShiftRegConfig.freqShiftInitPhase != qConfig_w.freqShiftRegConfig.freqShiftInitPhase) ||
                (qConfig_r.freqShiftRegConfig.freqShiftDirection != qConfig_w.freqShiftRegConfig.freqShiftDirection))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Freq Shift Configuration Register write test failed \n", i);
#endif
            }

            if (tmp > 0)
            {
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue %d Configuration Registers write test failed \n", i);
#endif
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeQueueConfigRegs() test                 FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeQueueConfigRegs() test                 PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

Void test_fftc_dftSizeListReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0, dftListSize;        
    UInt16                          dftSizeList_r [FFTC_MAX_NUM_BLOCKS], dftSizeList_w [FFTC_MAX_NUM_BLOCKS];
    UInt32                          i;        

    /* DFT Size list group register read/write test */
    if (Fftc_readDftSizeListGroupReg (pFFTCLldObj, dftSizeList_r) < 0)
    {
        testsFailed ++;
    }
    for (i = 0; i < FFTC_MAX_NUM_BLOCKS; i ++)
    {
        if (dftSizeList_r [i] != Fftc_DftBlockSizeTable [CSL_FFTC_DFT_LIST_G_RESETVAL])
            break;                
    }
    if (i != FFTC_MAX_NUM_BLOCKS)
    {
        Fftc_osalLog ("Fftc_readDftSizeListGroupReg() test              FAILED \n");
        testsFailed ++;
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_readDftSizeListGroupReg() test              PASSED \n");
        totalNumTestsPass ++;
    }

	for (dftListSize = 1; dftListSize <=10; dftListSize ++)
	{
    	/* Test DFT size list write */
    	for (i = 0; i < dftListSize; i ++)
    	{
        	dftSizeList_w [i] = 0x80;
    	}
    	if (Fftc_writeDftSizeListGroupReg (pFFTCLldObj, dftSizeList_w, dftListSize) < 0)
    	{
        	testsFailed ++;
    	}
    	else
    	{
        	/* verify write */
        	if (Fftc_readDftSizeListGroupReg (pFFTCLldObj, dftSizeList_r) < 0)
        	{
            	testsFailed ++;
        	}
        	for (i = 0; i < dftListSize; i ++)
        	{
            	if (dftSizeList_w [i] != dftSizeList_r [i])
                	break;
        	}
        	if (i != dftListSize)
        	{
            	testsFailed ++;
        	}
    	}
	}

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_writeDftSizeListGroupReg() test             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_writeDftSizeListGroupReg() test             PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_blockStatusReg (Fftc_LldObj*        pFFTCLldObj)
{
    UInt32                          testsFailed = 0;        
    UInt32                          i;        
    Fftc_DestQStatusReg             destQStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_ScalingShiftingStatusReg   shiftStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_CyclicPrefixStatusReg      cyclicPrefixStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_ControlStatusReg           controlStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_PktSizeStatusReg           pktSizeStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_TagStatusReg               tagStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    Fftc_FreqShiftStatusReg         freqShiftStatus_r[FFTC_NUM_INTERNAL_BUFFERS];
    

    /* 15:  FFTC Block Status Registers.
     *      FFTC Block X Destination Queue Status Register
     *      FFTC Block X Scaling & Shifting Status Register
     *      FFTC Block X Cyclic Prefix Status Register
     *      FFTC Block X Control Status Register
     *      FFTC Block X LTE Freqency Shift Status Register
     *      FFTC Block X Packet Size Status Register
     *      FFTC Block X Tag Status Register.
     */
    if (Fftc_readBlockDestQStatusReg (pFFTCLldObj, destQStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((destQStatus_r[i].bOutputFFTShift != CSL_FFTC_B0_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_OUTPUT_RESETVAL) ||
                (destQStatus_r[i].bInputFFTShift != CSL_FFTC_B0_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_INPUT_RESETVAL) ||
                (destQStatus_r[i].inputShiftVal != CSL_FFTC_B0_DEST_STAT_FFTC_VARIABLE_SHIFT_INPUT_RESETVAL) ||
                (destQStatus_r[i].cppiDestQNum != CSL_FFTC_B0_DEST_STAT_DEFAULT_DEST_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockDestQStatusReg() test             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockDestQStatusReg() test             PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockShiftStatusReg (pFFTCLldObj, shiftStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((shiftStatus_r[i].bDynamicScaleEnable != CSL_FFTC_B0_SHIFT_STAT_DYNAMIC_SCALING_ENABLE_RESETVAL) ||
                (shiftStatus_r[i].outputScaleVal != CSL_FFTC_B0_SHIFT_STAT_OUTPUT_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[0] != CSL_FFTC_B0_SHIFT_STAT_STAGE_0_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[1] != CSL_FFTC_B0_SHIFT_STAT_STAGE_1_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[2] != CSL_FFTC_B0_SHIFT_STAT_STAGE_2_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[3] != CSL_FFTC_B0_SHIFT_STAT_STAGE_3_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[4] != CSL_FFTC_B0_SHIFT_STAT_STAGE_4_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[5] != CSL_FFTC_B0_SHIFT_STAT_STAGE_5_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingVal[6] != CSL_FFTC_B0_SHIFT_STAT_STAGE_6_SCALING_RESETVAL) ||
                (shiftStatus_r[i].radixScalingValLast != CSL_FFTC_B0_SHIFT_STAT_STAGE_OUT_SCALING_RESETVAL) ||
                (shiftStatus_r[i].freqShiftScaleVal != CSL_FFTC_B0_SHIFT_STAT_STAGE_LTE_SHIFT_SCALING_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockShiftStatusReg() test             FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockShiftStatusReg() test             PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockCyclicPrefixStatusReg (pFFTCLldObj, cyclicPrefixStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((cyclicPrefixStatus_r[i].cyclicPrefixAddNum != CSL_FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_ADDITION_RESETVAL) ||
                (cyclicPrefixStatus_r[i].bCyclicPrefixRemoveEnable != CSL_FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_EN_RESETVAL) ||
                (cyclicPrefixStatus_r[i].cyclicPrefixRemoveNum != CSL_FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_OFFSET_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockCyclicPrefixStatusReg() test      FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockCyclicPrefixStatusReg() test      PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockControlStatusReg (pFFTCLldObj, controlStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((controlStatus_r[i].dftSize != CSL_FFTC_B0_CNTRL_STAT_DFT_SIZE_RESETVAL) ||
                (controlStatus_r[i].dftMode != CSL_FFTC_B0_CNTRL_STAT_DFT_IDFT_SELECT_RESETVAL) ||
                (controlStatus_r[i].bSupressSideInfo != CSL_FFTC_B0_CNTRL_STAT_SUPPRESSED_SIDE_INFO_RESETVAL) ||
                (controlStatus_r[i].inputQNum != CSL_FFTC_B0_CNTRL_STAT_INPUT_QUEUE_NUM_RESETVAL) ||
                (controlStatus_r[i].bIsSOP != CSL_FFTC_B0_CNTRL_STAT_SOP_RESETVAL) ||
                (controlStatus_r[i].bIsEOP != CSL_FFTC_B0_CNTRL_STAT_EOP_RESETVAL) ||
                (controlStatus_r[i].bIsBlockError != CSL_FFTC_B0_CNTRL_STAT_BLOCK_ERROR_RESETVAL) ||
                (controlStatus_r[i].zeroPadFactor != CSL_FFTC_B0_CNTRL_STAT_ZERO_PAD_VAL_RESETVAL) ||
                (controlStatus_r[i].zeroPadMode != CSL_FFTC_B0_CNTRL_STAT_ZERO_PAD_MODE_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockControlStatusReg() test           FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockControlStatusReg() test           PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockFreqShiftStatusReg (pFFTCLldObj, freqShiftStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((freqShiftStatus_r[i].bFreqShiftEnable != CSL_FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_EN_RESETVAL) ||
                (freqShiftStatus_r[i].freqShiftIndex != CSL_FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_INDEX_RESETVAL) ||
                (freqShiftStatus_r[i].freqShiftMultFactor != CSL_FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_FACTOR_RESETVAL) ||
                (freqShiftStatus_r[i].freqShiftInitPhase != CSL_FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_PHASE_RESETVAL) ||
                (freqShiftStatus_r[i].freqShiftDirection != CSL_FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_DIR_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockFreqShiftStatusReg() test         FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockFreqShiftStatusReg() test         PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockPktSizeStatusReg (pFFTCLldObj, pktSizeStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((pktSizeStatus_r[i].pktSize != CSL_FFTC_B0_PSIZE_STAT_PACKET_SIZE_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }        
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockPktSizeStatusReg() test           FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockPktSizeStatusReg() test           PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_readBlockTagStatusReg (pFFTCLldObj, tagStatus_r) < 0)
    {
        testsFailed ++;
    }
    else
    {
        for ( i = 0; i < FFTC_NUM_INTERNAL_BUFFERS; i ++)
        {
            if ((tagStatus_r[i].destTag != CSL_FFTC_B0_DESTTAG_STAT_DEST_TAG_RESETVAL) || 
                (tagStatus_r[i].flowId != CSL_FFTC_B0_DESTTAG_STAT_FLOW_ID_RESETVAL) || 
                (tagStatus_r[i].srcId != CSL_FFTC_B0_DESTTAG_STAT_SRC_ID_RESETVAL))
            {
                testsFailed ++;
            }
        }
    }        
    if (testsFailed)
    {
        Fftc_osalLog ("fftc_read_blockTagStatusReg() test               FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_read_blockTagStatusReg() test               PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_compileqlocalfxn (Void)
{
    UInt32                          testsFailed = 0;      
    Fftc_QLocalCfg                  fftQLocalCfg;
    UInt8                           qCfgBuffer [32];
    UInt32                          len = 0, tmp;
    Fftc_QLocalCfgParams*           qCfgParams;

    memset (qCfgBuffer, 0, sizeof(UInt8) * 32);

    fftQLocalCfg.destQRegConfig.bOutputFFTShift   = 0;
    fftQLocalCfg.destQRegConfig.bInputFFTShift    = 0;
    fftQLocalCfg.destQRegConfig.inputShiftVal     = 0;
    fftQLocalCfg.destQRegConfig.cppiDestQNum      = 800;

    fftQLocalCfg.scalingShiftingRegConfig.bDynamicScaleEnable = 0;
    fftQLocalCfg.scalingShiftingRegConfig.outputScaleVal = 0x08;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[0] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[1] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[2] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[3] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[4] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[5] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[6] = 0x02;
    fftQLocalCfg.scalingShiftingRegConfig.radixScalingValLast = 0x03;
    fftQLocalCfg.scalingShiftingRegConfig.freqShiftScaleVal = 0x03;
 
    fftQLocalCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable = 1;
    fftQLocalCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum = 0x04;
    fftQLocalCfg.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable = 1;
    fftQLocalCfg.cyclicPrefixRegConfig.cyclicPrefixRemoveNum = 0x04;

    fftQLocalCfg.controlRegConfig.dftSize = 12;
    fftQLocalCfg.controlRegConfig.dftMode = Fftc_DFTMode_IDFT;
    fftQLocalCfg.controlRegConfig.bZeroPadEnable = 0;
    fftQLocalCfg.controlRegConfig.zeroPadMode = Fftc_ZeroPadMode_ADD;
    fftQLocalCfg.controlRegConfig.zeroPadFactor = 0x0;
    fftQLocalCfg.controlRegConfig.bSupressSideInfo = 1;

    fftQLocalCfg.freqShiftRegConfig.bFreqShiftEnable = 1;
    fftQLocalCfg.freqShiftRegConfig.freqShiftIndex = Fftc_FreqShiftIndex_12288;
    fftQLocalCfg.freqShiftRegConfig.freqShiftMultFactor = 0x02;
    fftQLocalCfg.freqShiftRegConfig.freqShiftInitPhase = 0x02;
    fftQLocalCfg.freqShiftRegConfig.freqShiftDirection = Fftc_FreqShiftDir_PLUS;

	if (len)	len = 0;
    if (Fftc_compileQueueLocalConfigParams (&fftQLocalCfg, qCfgBuffer, &len) < 0)
    {
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("Fftc_compileQueueLocalConfigParams () API failed \n");
#endif
        testsFailed ++;
    }
    else
    {
        if (len != 20)
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("not all q local registers configuration was added to data buffer \n");
#endif
            testsFailed ++;
        }
        else
        {
            qCfgParams = (Fftc_QLocalCfgParams *)qCfgBuffer;                

            tmp = 0;                
            if ((CSL_FEXT(qCfgParams->queuexDestQ,FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_OUTPUT) != 
                                    fftQLocalCfg.destQRegConfig.bOutputFFTShift) ||
                (CSL_FEXT(qCfgParams->queuexDestQ,FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_INPUT) != 
                                    fftQLocalCfg.destQRegConfig.bInputFFTShift) ||
                (CSL_FEXT(qCfgParams->queuexDestQ,FFTC_Q0_DEST_FFTC_VARIABLE_SHIFT_INPUT) != 
                                    fftQLocalCfg.destQRegConfig.inputShiftVal) ||
                (CSL_FEXT(qCfgParams->queuexDestQ,FFTC_Q0_DEST_DEFAULT_DEST) != fftQLocalCfg.destQRegConfig.cppiDestQNum))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Destination Queue Register compile test failed \n");
#endif
            }

            if ((CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_DYNAMIC_SCALING_ENABLE) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.bDynamicScaleEnable) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_OUTPUT_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.outputScaleVal) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_0_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[0]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_1_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[1]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_2_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[2]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_3_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[3]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_4_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[4]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_5_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[5]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_6_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingVal[6]) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_OUT_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.radixScalingValLast) ||
                (CSL_FEXT(qCfgParams->queuexScaleShift,FFTC_Q0_SCALE_SHIFT_STAGE_LTE_SHIFT_SCALING) != 
                                    fftQLocalCfg.scalingShiftingRegConfig.freqShiftScaleVal))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Scaling & Shifting Register compile test failed \n");
#endif
            }

            if ((CSL_FEXT(qCfgParams->queuexCyclicPrefix,FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION) != 
                                    fftQLocalCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum) ||
                (CSL_FEXT(qCfgParams->queuexCyclicPrefix,FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_EN) != 
                                    fftQLocalCfg.cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable) ||
                (CSL_FEXT(qCfgParams->queuexCyclicPrefix,FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_OFFSET) != 
                                    fftQLocalCfg.cyclicPrefixRegConfig.cyclicPrefixRemoveNum))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Cyclic Prefix Register compile test failed \n");
#endif
            }

            if ((CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_DFT_SIZE) != 
                                    fftQLocalCfg.controlRegConfig.dftSize) ||
                (CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_DFT_IDFT_SELECT) != 
                                    fftQLocalCfg.controlRegConfig.dftMode) ||
                (CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_ZERO_PAD_MODE) != 
                                    fftQLocalCfg.controlRegConfig.zeroPadMode) ||
                (CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_ZERO_PAD_VAL) != 
                                    fftQLocalCfg.controlRegConfig.zeroPadFactor) ||
                (CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_SUPPRESSED_SIDE_INFO) != 
                                    fftQLocalCfg.controlRegConfig.bSupressSideInfo))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Control Register compile test failed \n");
#endif
            }

            if ((CSL_FEXT(qCfgParams->queuexLteFreq,FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_EN) != 
                                    fftQLocalCfg.freqShiftRegConfig.bFreqShiftEnable) ||
                (CSL_FEXT(qCfgParams->queuexLteFreq,FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_INDEX) != 
                                    fftQLocalCfg.freqShiftRegConfig.freqShiftIndex) ||
                (CSL_FEXT(qCfgParams->queuexLteFreq,FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_FACTOR) != 
                                    fftQLocalCfg.freqShiftRegConfig.freqShiftMultFactor) ||
                (CSL_FEXT(qCfgParams->queuexLteFreq,FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_PHASE) != 
                                    fftQLocalCfg.freqShiftRegConfig.freqShiftInitPhase) ||
                (CSL_FEXT(qCfgParams->queuexLteFreq,FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_DIR) != 
                                    fftQLocalCfg.freqShiftRegConfig.freqShiftDirection))
            {
                tmp ++;                    
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Freq Shift Configuration Register compile test failed \n");
#endif
            }

            if (tmp > 0)
            {
#ifdef FFTC_TEST_DEBUG                    
                Fftc_osalLog ("FFTC Queue Configuration Registers compile test failed \n");
#endif
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_compileQueueLocalConfigParams() test        FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_compileQueueLocalConfigParams() test        PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    if (Fftc_recompileQueueLocalDFTParams (36, Fftc_DFTMode_DFT, qCfgBuffer) < 0)
    {
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("fftc_recompile_queueLocalDFTParams () API failed \n");
#endif
        testsFailed ++;
    }
    else
    {
        qCfgParams = (Fftc_QLocalCfgParams *)qCfgBuffer;                
        if ((Fftc_DftBlockSizeTable [CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_DFT_SIZE)] != 36) ||
            (CSL_FEXT(qCfgParams->queuexControl,FFTC_Q0_CONTROL_DFT_IDFT_SELECT) != Fftc_DFTMode_DFT))
        {
                    
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("DFT size / DFT enable bit reconfiguration failed \n");
#endif
            testsFailed ++;
        }

    }

    if (testsFailed)
    {
        Fftc_osalLog ("fftc_recompile_queueLocalDFTParams() test        FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("fftc_recompile_queueLocalDFTParams() test        PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;
    if (Fftc_recompileQueueLocalCyclicPrefixParams (8, qCfgBuffer) < 0)
    {
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("fftc_recompile_localCyclicPrefixParams () API failed \n");
#endif
        testsFailed ++;
    }
    else
    {
        qCfgParams = (Fftc_QLocalCfgParams *)qCfgBuffer;                
        if ((CSL_FEXT(qCfgParams->queuexCyclicPrefix,FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION) != 8))
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("Cyclic Prefix addition value reconfiguration failed \n");
#endif
            testsFailed ++;
        }
    }
    
    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_recompileQueueLocalCyclicPrefixParams()     FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_recompileQueueLocalCyclicPrefixParams()     PASSED \n");
        totalNumTestsPass ++;
    }

    /* Test done. */
    return;
}

static Void test_fftc_createDftSizeList ()
{
    UInt32                          testsFailed = 0;        
    UInt32                          dftSizeListBuffer [26];
    UInt16                          dftSizeList_w [FFTC_MAX_NUM_BLOCKS];
    UInt32                          i, j = 0,  listLen, numBlocks, numGroups;        

    /* Test DFT size list compilation */
    memset (dftSizeListBuffer, 0, 26 * sizeof(dftSizeListBuffer[0]));

    /* We'll test for varying DFT list lengths */
    for (numBlocks = 1; numBlocks <= 10; numBlocks ++)
    {
        numGroups = ((numBlocks % 5 > 0) ? (numBlocks / 5 + 1) : (numBlocks / 5));

        for (i = 0; i < numBlocks; i ++)
        {
            dftSizeList_w [i] = 0x80;       // Give a valid DFT size 
        }

        /* Initialize listlen */
        listLen = 0;
        if (Fftc_createDftSizeList (dftSizeList_w, numBlocks, (UInt8 *)dftSizeListBuffer, &listLen) < 0)
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("Fftc_createDftSizeList () API returned error\n");          
#endif
            testsFailed ++;
        }
        else
        {
            /* Each group is represented as a word */                    
            if (listLen != 4 * numGroups)            
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("Not all DFT blocks have been compiled into the buffer \n");
                Fftc_osalLog ("length returned: %d length should be: %d \n", listLen, numGroups * 4);                
#endif
                testsFailed ++;
            }
            else
            {
                for (i = 0, j = 0; i < numGroups - 1; i ++, j += 5)
                {
                    if ((Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_0)]  != dftSizeList_w [j + 0]) ||
                        (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_1)]  != dftSizeList_w [j + 1]) ||
                        (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_2)]  != dftSizeList_w [j + 2]) ||
                        (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_3)]  != dftSizeList_w [j + 3]) ||                                     (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_4)]  != dftSizeList_w [j + 4]))
                    {
#ifdef FFTC_TEST_DEBUG
                        Fftc_osalLog ("DFT block configurations do not match for blocks 0/1/2/3/4 in group %d \n", i);                
#endif
                        testsFailed ++;
                        break;
                    }
                }

                if (i == numGroups - 1)
                {
                    switch ((numBlocks - j) % 5)
                    {
                        case 1:
                        {
                            if ((Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_0)]  != dftSizeList_w [j + 0]))
                            {
#ifdef FFTC_TEST_DEBUG
                                Fftc_osalLog ("DFT block configurations do not match for block 0 in group %d \n", i);                
#endif
                                testsFailed ++;
                            }                                
                            break;
                        }
                        case 2:
                        {
                            if ((Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_0)]  != dftSizeList_w [j + 0]) ||
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_1)]  != dftSizeList_w [j + 1])) 
                            {
#ifdef FFTC_TEST_DEBUG
                                Fftc_osalLog ("DFT block configurations do not match for block 0/1 in group %d \n", i);                
#endif
                                testsFailed ++;
                            }                                
                            break;
                        }
                        case 3:
                        {
                            if ((Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_0)]  != dftSizeList_w [j + 0]) ||
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_1)]  != dftSizeList_w [j + 1]) || 
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_2)]  != dftSizeList_w [j + 2])) 
                            {
#ifdef FFTC_TEST_DEBUG
                                Fftc_osalLog ("DFT block configurations do not match for block 0/1/2 in group %d \n", i);                
#endif
                                testsFailed ++;
                            }                                
                            break;
                        }
                        case 4:
                        {
                            if ((Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_0)]  != dftSizeList_w [j + 0]) ||
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_1)]  != dftSizeList_w [j + 1]) || 
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_2)]  != dftSizeList_w [j + 2]) || 
                                (Fftc_DftBlockSizeTable [CSL_FEXT (dftSizeListBuffer[i], FFTC_DFT_LIST_G_DFT_SIZE_3)]  != dftSizeList_w [j + 3])) 
                            {
#ifdef FFTC_TEST_DEBUG
                                Fftc_osalLog ("DFT block configurations do not match for block 0/1/2/3 in group %d \n", i);                
#endif
                                testsFailed ++;
                            }                                
                            break;
                        }
                        default:
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_createDftSizeList() test                    FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_createDftSizeList() test                    PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

static Void test_fftc_compileControlHdr (Void)
{
    UInt32                          testsFailed = 0;        
    Fftc_ControlHdr                 ctrlHdr;
    UInt32                          ctrlHdrDataBuffer;
    UInt32                          dataLen = 0;

    /* Put together a control header sample */
    ctrlHdr.bLocalConfigPresent     =   1;
    ctrlHdr.bDFTSizeListPresent     =   1;
    ctrlHdr.bPSPassThruPresent      =   1;
    ctrlHdr.dftSizeListLen          =   26;
    ctrlHdr.psFieldLen              =   4;

    if (Fftc_createControlHeader (&ctrlHdr,  (UInt8 *)&ctrlHdrDataBuffer, &dataLen) < 0)
    {
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("Fftc_createControlHeader() API returned error \n");            
#endif
        testsFailed ++;
    }
    else
    {
        if (dataLen != 4)
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("Fftc_createControlHeader() API returned invalid control header length \n");            
#endif
            testsFailed ++;
        }
        else
        {
            /* Verify Control Header */            
            if ((CSL_FEXTR (ctrlHdrDataBuffer, 0, 0) != ctrlHdr.bLocalConfigPresent) ||
                (CSL_FEXTR (ctrlHdrDataBuffer, 1, 1) != ctrlHdr.bDFTSizeListPresent) ||
                (CSL_FEXTR (ctrlHdrDataBuffer, 2, 2) != ctrlHdr.bPSPassThruPresent) ||
                (CSL_FEXTR (ctrlHdrDataBuffer, 20, 16) != ctrlHdr.dftSizeListLen) ||
                (CSL_FEXTR (ctrlHdrDataBuffer, 28, 24) != ctrlHdr.psFieldLen))
            {
#ifdef FFTC_TEST_DEBUG
                Fftc_osalLog ("Invalid FFTC Control Header configuration compilation \n");            
#endif
                testsFailed ++;
            }
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_createControlHeader() test                  FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_createControlHeader() test                  PASSED \n");
        totalNumTestsPass ++;
    }

    testsFailed = 0;

    /* Flip the local Config present bit */
    ctrlHdr.bLocalConfigPresent     =   0;
    if (Fftc_modifyLocalCfgPresentControlHeader (ctrlHdr.bLocalConfigPresent,  (UInt8 *)&ctrlHdrDataBuffer) < 0)
    {
#ifdef FFTC_TEST_DEBUG
        Fftc_osalLog ("Fftc_modifyLocalCfgPresentControlHeader() API returned error \n");            
#endif
        testsFailed ++;
    }
    else
    {
        /* Verify Control Header */            
        if ((CSL_FEXTR (ctrlHdrDataBuffer, 0, 0) != ctrlHdr.bLocalConfigPresent))
        {
#ifdef FFTC_TEST_DEBUG
            Fftc_osalLog ("Invalid FFTC Control Header reconfiguration - local config present bit invalid \n");            
#endif
            testsFailed ++;
        }
    }

    if (testsFailed)
    {
        Fftc_osalLog ("Fftc_modifyLocalCfgPresentControlHeader()        FAILED \n");
        totalNumTestsFail ++;
    }
    else
    {
        Fftc_osalLog ("Fftc_modifyLocalCfgPresentControlHeader()        PASSED \n");
        totalNumTestsPass ++;
    }

    return;
}

/**
 * =============================================================================
 *  @n@b test_fftc_lld
 *
 *  @b  brief
 *  @n  Function to test all the FFTC Register Read/Write APIs exposed by FFTC
 *      Lower Layer Driver (LLD).      
 *
 *  @return
 *      None
 * =============================================================================
 */
Void test_fftc_lld()
{
    Fftc_LldObj             fftcLldObj;

    Fftc_osalLog ("**************************************************\n");
    Fftc_osalLog ("************* FFTC LLD Testing Start *************\n");
    Fftc_osalLog ("**************************************************\n");

    Fftc_osalLog ("*************** Testing Instance A ***************\n");

    if (Fftc_lldOpen (CSL_FFTC_0, (void *)CSL_FFTC_0_CFG_REGS, &fftcLldObj) != 0)
    {
        /* FFTC Instance A open failed */            
        Fftc_osalLog ("FFTC LLD Instance A open failed \n");
        return;
    }

    /* Run FFTC LLD testing using Instance A 
     *
     * Test all FFTC MMRs Read/Writes 
     */        
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - MMR Testing START! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    test_fftc_pidReg (&fftcLldObj);
    test_fftc_configReg (&fftcLldObj);
    test_fftc_controlReg (&fftcLldObj);
    test_fftc_statusReg (&fftcLldObj);
    test_fftc_emulationControlReg (&fftcLldObj);
    test_fftc_intRawStatusReg (&fftcLldObj);
    test_fftc_clearIntRawStatusReg (&fftcLldObj);
    test_fftc_intEnableSetReg (&fftcLldObj);
    test_fftc_clearIntEnableSetReg (&fftcLldObj);
    test_fftc_haltOnErrorReg (&fftcLldObj);
    test_fftc_eoiReg (&fftcLldObj);
    test_fftc_clippingDetectReg (&fftcLldObj);
    test_fftc_qlocalRegs (&fftcLldObj);
    test_fftc_dftSizeListReg (&fftcLldObj);
    test_fftc_blockStatusReg (&fftcLldObj);

    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - MMR Testing DONE! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    /* Reset the FFTC engine before proceeding with next test */
    Fftc_doSoftwareReset (&fftcLldObj);

    Fftc_lldClose (&fftcLldObj);

    Fftc_osalLog ("*************** Testing Instance B ***************\n");

    if (Fftc_lldOpen (CSL_FFTC_1, (void *)CSL_FFTC_1_CFG_REGS, &fftcLldObj) != 0)
    {
        /* FFTC Instance B open failed */            
        Fftc_osalLog ("FFTC LLD Instance B open failed \n");
        return;
    }

    /* Run FFTC LLD testing using Instance B 
     *
     * Test all FFTC MMRs Read/Writes 
     */        
    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - MMR Testing START! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    test_fftc_pidReg (&fftcLldObj);
    test_fftc_configReg (&fftcLldObj);
    test_fftc_controlReg (&fftcLldObj);
    test_fftc_statusReg (&fftcLldObj);
    test_fftc_emulationControlReg (&fftcLldObj);
    test_fftc_intRawStatusReg (&fftcLldObj);
    test_fftc_clearIntRawStatusReg (&fftcLldObj);
    test_fftc_intEnableSetReg (&fftcLldObj);
    test_fftc_clearIntEnableSetReg (&fftcLldObj);
    test_fftc_haltOnErrorReg (&fftcLldObj);
    test_fftc_eoiReg (&fftcLldObj);
    test_fftc_clippingDetectReg (&fftcLldObj);
    test_fftc_qlocalRegs (&fftcLldObj);
    test_fftc_dftSizeListReg (&fftcLldObj);
    test_fftc_blockStatusReg (&fftcLldObj);

    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - MMR Testing DONE! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    /* Reset the FFTC engine before proceeding with next test */
    Fftc_doSoftwareReset (&fftcLldObj);

    Fftc_lldClose (&fftcLldObj);

    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - Helper APIs Testing START! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    test_fftc_compileqlocalfxn ();
    test_fftc_createDftSizeList ();
    test_fftc_compileControlHdr ();

    Fftc_osalLog ("--------------------------------------------\n");        
    Fftc_osalLog ("FFTC LLD Test - Helper APIs Testing DONE! \n");
    Fftc_osalLog ("--------------------------------------------\n");        

    Fftc_osalLog ("**************************************************\n");
    Fftc_osalLog ("************** FFTC LLD Testing End **************\n");
    Fftc_osalLog ("**************************************************\n");

    return;        
}

