#if EVM_TYPE == 5 || EVM_TYPE == 7

#define __BOARDS_SYNC_C
#include "appletonScbpSync.h"
#include <ti/csl/csl.h>
#include <ti/csl/csl_bootcfgAux.h>
#if EVM_TYPE == 5
#include <ti/csl/csl_pllc.h>
#include <ti/csl/csl_pllcAux.h>
#endif
/*#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/csl_cpIntc.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>*/

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

BoardsSync_t BoardsSync;

static int timerpll;
static volatile int *LTCR; // = ((int *)(TIMER_BASE + TCR_OFFSET));
static volatile int *LTGCR; // = ((int *)(TIMER_BASE + TGCR_OFFSET));
static volatile int *LINTCTL_STAT; // = ((int *)(TIMER_BASE + INTCTL_STAT_OFFSET));
static volatile int *LCAP_LO; // = ((int *)(TIMER_BASE + CAP_LO_OFFSET));
static volatile int *LREL_LO; // = ((int *)(TIMER_BASE + REL_LO_OFFSET));
static volatile int *LREL_HI; // = ((int *)(TIMER_BASE + REL_HI_OFFSET));
static volatile int *LTIM_LO; // = ((int *)(TIMER_BASE + TIM_LO_OFFSET));
static volatile int *LTIM_HI; // = ((int *)(TIMER_BASE + TIM_HI_OFFSET));
static volatile int *LPRD_LO; // = ((int *)(TIMER_BASE + PRD_LO_OFFSET));
static volatile int *LPRD_HI; // = ((int *)(TIMER_BASE + PRD_HI_OFFSET));


#ifdef DBG
dbg_t     gDbg;
#endif


#ifdef DBG
void calculateHistogram(float err)
{
	if (err <= 5.0) {
		gDbg.movingAvgErrCounts0to5ppb++;
	}
	else if ((err > 5.0) && (err <= 10.0)) {
		gDbg.movingAvgErrCounts5to10ppb++;
	}
	else if ((err > 10.0) && (err <= 15.0)) {
		gDbg.movingAvgErrCounts10to15ppb++;
	}
	else if ((err > 15.0) && (err <= 20.0)) {
		gDbg.movingAvgErrCounts15to20ppb++;
	}
	else if (err > 20.0) {
		gDbg.movingAvgErrCountsHigherThan20ppb++;
	}
}

static void dbgLogState(BoardsSyncState_e state)
{
  int indx = gDbg.stateBufIndx++ & (STATE_BUF_SIZE - 1);
  
  gDbg.stateBuf[indx].state = state;
  gDbg.stateBuf[indx].timestampCount = gDbg.timestampCounter;
}

#endif

void OS_disable_interrupts(void)
{
#ifdef DSP_SYSBIOS
  Hwi_disable();
#endif

#ifdef LINUX
  TODO
#endif
}

void OS_enable_interrupts(void)
{
#ifdef DSP_SYSBIOS
  Hwi_enable();
#endif

#ifdef LINUX
  TODO
#endif
}

void switchOffPps(void)
{
#ifdef TINPSEL0
    //*TINPSEL = 0x00000100;
	CSL_BootCfgSetTimerInputSelection(0x00000100);
#endif

#ifdef TINPSEL1
    //*TINPSEL = 0x00000000;
	CSL_BootCfgSetTimerInputSelection(0);
#endif

}

void switchOnPps(void)
{
#ifdef TINPSEL0
    //*TINPSEL = 0x00000000;
    CSL_BootCfgSetTimerInputSelection(0);
#endif

#ifdef TINPSEL1
    //*TINPSEL = 1 << (2*timerpll);
    CSL_BootCfgSetTimerInputSelection((1 << (2*timerpll)));

#endif
}

int dac_write_val(int x)
{
#ifndef DBG_NO_DAC_WRITE
  if (x < DAC_LOW_VAL) x = DAC_LOW_VAL;
  
  if (x > DAC_HIGH_VAL) x = DAC_HIGH_VAL;

#ifdef DBG
  gDbg.dacinpBuf[gDbg.timestampCounter & (DBG_BUF_SIZE -1)] = x;
#endif  
  
#ifndef EVM
#if EVM_TYPE == 5
  if (spi_6614evm_dac_write_transaction(x) != 0) {
#elif EVM_TYPE == 7
	  if (spi_dac_write(x) != 0) {
#endif
	  printf("exception\n");
	  while(1);
  }
#endif
  
  return (x);
#endif  
} 

void routeTimerLowOutToTimerOut0Pin(void)
{

	//*TOUTPSEL = 2*timerpll;
	CSL_BootCfgSetTimerOutputSelection((uint8_t)(2*timerpll),NULL);

}

void routeTimerHighOutToTimerOut0Pin(void)
{

   //*TOUTPSEL = (2*timerpll) + 1;
   CSL_BootCfgSetTimerOutputSelection(((uint8_t)((2*timerpll)+1)),NULL);

}


static void configPll (BoardsSyncPll_t *pll, BoardsSyncPllState_e state)
{
    pll->numSamplesState = 0;

	switch (state) {
       case FAST:
			pll->errorGain = pll->gainCalculatedInCalibration * 1.0;
			pll->integratorGain = pll->gainCalculatedInCalibration * INTEGRATOR_GAIN_SCALE_FAST;
            pll->processMask = PROCESS_MASK_FAST;
            pll->derivativeGain = pll->gainCalculatedInCalibration * DERIVATIVE_GAIN_SCALE_FAST;
            if (pll->state == MEDIUM) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_MEDIUM / 
                                                                         INTEGRATOR_GAIN_SCALE_FAST);
            }
            if (pll->state == SLOW) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_SLOW / 
                                                                         INTEGRATOR_GAIN_SCALE_FAST);            
            }            
            pll->resetAvgDacOutFlag = 1;
		break;

	  case MEDIUM:
			pll->errorGain = pll->gainCalculatedInCalibration * ERROR_GAIN_SCALE_MEDIUM;
			pll->integratorGain = pll->gainCalculatedInCalibration * INTEGRATOR_GAIN_SCALE_MEDIUM;
            if (pll->state == FAST) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_FAST / 
                                                                         INTEGRATOR_GAIN_SCALE_MEDIUM);
            }
            if (pll->state == SLOW) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_SLOW / 
                                                                         INTEGRATOR_GAIN_SCALE_MEDIUM);            
            }
            pll->processMask = PROCESS_MASK_MEDIUM;
            pll->derivativeGain = pll->gainCalculatedInCalibration * DERIVATIVE_GAIN_SCALE_MEDIUM;
            pll->resetAvgDacOutFlag = 1;            
		break;

      case SLOW:
			pll->errorGain = pll->gainCalculatedInCalibration * ERROR_GAIN_SCALE_SLOW;
			pll->integratorGain = pll->gainCalculatedInCalibration * INTEGRATOR_GAIN_SCALE_SLOW;
            if (pll->state == MEDIUM) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_MEDIUM / 
                                                                         INTEGRATOR_GAIN_SCALE_SLOW);
            }
            if (pll->state == FAST) {
              pll->integratorValue = round((float)pll->integratorValue * INTEGRATOR_GAIN_SCALE_FAST / 
                                                                         INTEGRATOR_GAIN_SCALE_SLOW);
            }            
            pll->processMask = PROCESS_MASK_MEDIUM;
            pll->derivativeGain = pll->gainCalculatedInCalibration * DERIVATIVE_GAIN_SCALE_SLOW;
		break;
	}

#ifdef DBG
      gDbg.pllStateBuf[gDbg.pllStateBufIndx & (PLL_STATE_BUF_SIZE -1)].state = state;
      gDbg.pllStateBuf[gDbg.pllStateBufIndx++ & (PLL_STATE_BUF_SIZE -1)].timestampCount = gDbg.timestampCounter;
#endif

	pll->state = state;
}

void configPllPostCalibration (BoardsSyncPll_t *pll, int dacZeroVal, float gain)
{
  pll->dacMidVal = dacZeroVal;// - 10; //use dacZeroVal +/- 10 for testing integrator accumulation dynamics
  pll->avgDacOut = (float) pll->dacMidVal;
  pll->resetAvgDacOutFlag = 1;
  pll->gainCalculatedInCalibration = gain;

  pll->adjustmentLimit = round(pll->resyncStateMaxTimingAdjustmentRate * 1e-6 *
		                       pll->timerCountsInOneSec * gain);

  pll->state = FAST; //this line must precede configPll call
  configPll (pll, FAST);
}

static void initPllConvergenceEstimation (BoardsSyncPllEstimateConvergence_t *est,
                                          float avgErrThreshold, float maxErrThreshold)
{
  int i;
  
  memset (est, 0, sizeof(BoardsSyncPllEstimateConvergence_t));

  est->avgErrThreshold = avgErrThreshold;
  est->maxErrThreshold = maxErrThreshold;
  
  for (i = 0; i < BOARDSSYNC_PLL_ERROR_HISTORY_SIZE; i++) {
    est->errorHistory[i] = (int) est->maxErrThreshold;
  }
    
  est->errorAccumulator = BOARDSSYNC_PLL_ERROR_HISTORY_SIZE * (int) est->maxErrThreshold;
}

static void initPll (BoardsSyncPll_t *pll, int timerCountsInOneSec, int avgErrPpbRequirement,
                     int maxErrPpbRequirement,
                     float resyncStateMaxTimingAdjustmentRate)
{
    float avgErrThreshold, maxErrThreshold;
    
    memset(pll, 0, sizeof(BoardsSyncPll_t));
    pll->timerCountsInOneSec = timerCountsInOneSec;

    pll->resyncStateMaxTimingAdjustmentRate = resyncStateMaxTimingAdjustmentRate;
    
    avgErrThreshold = avgErrPpbRequirement;
    maxErrThreshold = maxErrPpbRequirement;                                          
    
    initPllConvergenceEstimation (&pll->est, avgErrThreshold, maxErrThreshold); 
    
    pll->dacMidVal = DAC_MID_VAL;
}


static void resetPllConvergenceEstimation (BoardsSyncPllEstimateConvergence_t *est)
{
  initPllConvergenceEstimation (est, est->avgErrThreshold, est->maxErrThreshold);
}

void BoardsSyncInit (BoardsSync_t *BoardsSync, int timerCountsInOneSec,
                  int avgErrPpbRequirement, int maxErrPpbRequirement,
                  int enableControlledRateOfAdjustment,
                  float resyncStateMaxTimingAdjustmentRate,
                  int timer)
{
#if EVM_TYPE == 5
    CSL_PllcHandle  hPllc;
    uint8_t           pllmVal;
#endif	

	timerpll = timer;
	LTCR = (int *)(TIMER_BASE + TCR_OFFSET + (timerpll<<16));
	LTGCR = (int *)(TIMER_BASE + TGCR_OFFSET + (timerpll<<16));
	LINTCTL_STAT = (int *)(TIMER_BASE + INTCTL_STAT_OFFSET + (timerpll<<16));
	LCAP_LO = (int *)(TIMER_BASE + CAP_LO_OFFSET + (timerpll<<16));
	LREL_LO = (int *)(TIMER_BASE + REL_LO_OFFSET + (timerpll<<16));
	LREL_HI = (int *)(TIMER_BASE + REL_HI_OFFSET + (timerpll<<16));
	LTIM_LO = (int *)(TIMER_BASE + TIM_LO_OFFSET + (timerpll<<16));
	LTIM_HI = (int *)(TIMER_BASE + TIM_HI_OFFSET + (timerpll<<16));
	LPRD_LO = (int *)(TIMER_BASE + PRD_LO_OFFSET + (timerpll<<16));
	LPRD_HI = (int *)(TIMER_BASE + PRD_HI_OFFSET + (timerpll<<16));

#if EVM_TYPE == 5
	// Recompute timerCountsInOneSec based on main PLL multiplier
	hPllc   = CSL_PLLC_open(0);
    pllmVal = CSL_PLLC_getPllMultiplierCtrlReg (hPllc);
    // 122.88 MHz * 20 / 1 / 2 = 1228.8 MHz so pllm reg field = 19
    // 122.88 MHz * 16 / 1 / 2 = 983.04 MHz so pllm reg field = 15
    if (pllmVal == 19) {
    	timerCountsInOneSec = (204800000/SCALE_SEC);
    } else if (pllmVal == 15) {
    	timerCountsInOneSec = (163840000/SCALE_SEC);
    } else { //we assume the 1198.08 MHz setting to comply with 1.2 GHz by u-boot, this is achieved by a combination of pllm and other factors (that are not unity)
        timerCountsInOneSec = (199680000/SCALE_SEC);
    }

#endif	

	memset (BoardsSync, 0, sizeof(BoardsSync_t));
    BoardsSync->state = PLL_CALIBRATE;
    BoardsSync->timerCountsInOneSec = timerCountsInOneSec;
    BoardsSync->enableControlledRateOfAdjustment = enableControlledRateOfAdjustment;
                    
    #ifdef DBG
    dbgLogState(BoardsSync->state);
    #endif

    BoardsSync->ppsLossTimeCount = timerCountsInOneSec + (timerCountsInOneSec >> 1);
    BoardsSync->firstIsr = 1;
	initPll(&BoardsSync->pll, timerCountsInOneSec, avgErrPpbRequirement,
			maxErrPpbRequirement, resyncStateMaxTimingAdjustmentRate);
   // routeTimerHighOutToTimerOut0Pin();
}


static void freezePll (BoardsSyncPll_t *pll)
{
	pll->accumTime = 0;
    pll->prevError = 0;
}

static void processPllConvergenceEstimation(BoardsSyncPll_t *pll, float error_ppb)
{
   int i, indx;
   float maxError, movingAverageError;
   
   BoardsSyncPllEstimateConvergence_t *est = &pll->est;
  
   error_ppb = abs(error_ppb);
    
   indx = est->errorHistoryIndex++ & (BOARDSSYNC_PLL_ERROR_HISTORY_SIZE - 1);
   est->errorAccumulator -= est->errorHistory[indx];
   est->errorHistory[indx] = error_ppb;
   est->errorAccumulator += error_ppb;
   
   movingAverageError = est->errorAccumulator/BOARDSSYNC_PLL_ERROR_HISTORY_SIZE;

#ifdef DBG
   if (gDbg.enableHistogramCalculation) {
     calculateHistogram(movingAverageError);
   }  
#endif
   
   for (i = 1, maxError = est->errorHistory[0]; i < BOARDSSYNC_PLL_ERROR_HISTORY_SIZE; i++) {
     if (maxError < est->errorHistory[i]) {
       maxError = est->errorHistory[i];
     }
   }
   
   if ((movingAverageError < est->avgErrThreshold) && 
       (maxError < est->maxErrThreshold)) {
#ifdef DBG
	  if (est->converged == 0) {
	    int indx = gDbg.convergenceIndex++ & (CONVERGENCE_BUF_SIZE - 1);
	    gDbg.convergence[indx].converged = 1;
	    gDbg.convergence[indx].maxError = maxError;
	    gDbg.convergence[indx].movingAverageError = movingAverageError;
	    gDbg.convergence[indx].timestampCount = gDbg.timestampCounter;
	  }
#endif
	   est->converged = 1;
       
#ifdef DBG
       /* after first convergence, enable histogram calculation */
       if (gDbg.enableHistogramCalculation == 0) {
         gDbg.enableHistogramCalculation = 1;
       }
#endif       
   }
   else {
#ifdef DBG
	  if (est->converged == 1) {
	    int indx = gDbg.convergenceIndex++ & (CONVERGENCE_BUF_SIZE - 1);
	    gDbg.convergence[indx].converged = 0;
	    gDbg.convergence[indx].maxError = maxError;
	    gDbg.convergence[indx].movingAverageError = movingAverageError;
	    gDbg.convergence[indx].timestampCount = gDbg.timestampCounter;
	  }
#endif
      if (est->converged == 1) {
        /* diverged, put PLL in medium state */
        //configPll(pll, MEDIUM);
      }
      est->converged = 0;
   }
}

static inline void updateAvgDacOut (float *avgDacOut, int dacout)
{
  *avgDacOut += 0.01 * ((float) dacout - *avgDacOut);
  
#ifdef DBG
  gDbg.avgDacOutBuf[gDbg.timestampCounter & (DBG_BUF_SIZE -1)] = *avgDacOut;  
#endif

}

static int isPllConverged (BoardsSyncPll_t *pll)
{
  return((pll->est.converged) && (pll->state == SLOW) && (pll->numSamplesState >= SAMPLES_IN_SLOW_STATE));
}

static int processPll (BoardsSyncPll_t *pll, int error,
                       int controlledRateOfAdjustment, int isUpdate)
{
   int timestampErr, dacout;
   float out;
   

   pll->accumTime += error;
   timestampErr = pll->accumTime;
      
   if (isUpdate) {
#if 0
     if ( (! controlledRateOfAdjustment) || 
          ((controlledRateOfAdjustment) && (! isTimeDriftTooMuch(timestampErr, pll)))
        ) {
#endif        
     switch (pll->state) {
       case FAST:
         if (pll->numSamplesState == SAMPLES_IN_FAST_STATE) {
            configPll(pll, MEDIUM);
         }
         pll->numSamplesState++;       
       break;

       case MEDIUM:  
         if (pll->numSamplesState == SAMPLES_IN_MEDIUM_STATE) {
            configPll(pll, SLOW);
         }
         pll->numSamplesState++;
       break;
          
       case SLOW:
         if (pll->numSamplesState <= SAMPLES_IN_SLOW_STATE) {
           pll->numSamplesState++;
         }
       break;
     }
   }
   
   if (! isUpdate) {
     return (timestampErr);
   }
   
#if 1 /* sometimes there is a false pulse between the 1 sec PPS, need to handle this without
         a complete resync, this code must be after the return above i.e when isUpdate is 1 */
   if (labs(timestampErr - pll->timerCountsInOneSec) <= (pll->timerCountsInOneSec >> 7) /* about 10 ms */) {
      pll->accumTime -= pll->timerCountsInOneSec;
      timestampErr = pll->accumTime;
   }
#endif

   if (! controlledRateOfAdjustment) {
     pll->integratorValue += timestampErr;
   }     

   if ((pll->processCount++ & pll->processMask) == 0) {
	 float tmp;

     tmp = (pll->errorGain * timestampErr) + 
           (pll->derivativeGain * (timestampErr - pll->prevError));
           
     if (controlledRateOfAdjustment) {
       if (tmp > pll->adjustmentLimit) {
         tmp = pll->adjustmentLimit;
       }
       if (tmp < -pll->adjustmentLimit) {
         tmp = -pll->adjustmentLimit;
       }
     }
     
     tmp += (pll->integratorGain * (float) pll->integratorValue);
     
     out = tmp + (float) pll->dacMidVal;

#ifdef ACCURATE_HOLDOVER
     if (pll->resetAvgDacOutFlag) {     
       pll->avgDacOut = out;
       pll->resetAvgDacOutFlag = 0;
     }  
          
     if (pll->avgDacOut > out) {
       dacout = round(out - .5);
     }
     else {
       dacout = round(out + .5);
     }
#else
     dacout = round(out);
#endif

     dacout = dac_write_val(dacout /*round(tmp) + pll->dacMidVal*/);
     
#ifdef ACCURATE_HOLDOVER
     updateAvgDacOut(&pll->avgDacOut, dacout);
     pll->lastDacOut = dacout;
#endif
     
     pll->prevError = timestampErr;
     
#ifdef DBG
   gDbg.errorBuf[gDbg.timestampCounter & (DBG_BUF_SIZE - 1)] = timestampErr;
   gDbg.integratorBuf[gDbg.timestampCounter & (DBG_BUF_SIZE - 1)] = pll->integratorValue;
#endif     
   }
   
   return(timestampErr);
}

static inline int isCapModeExpirationLo (void) {
  return (*LINTCTL_STAT & 0x00000008);
}

static inline int isPrdExpirationLo(void) {
 return (*LINTCTL_STAT & 0x00000002);
}

static inline int isPrdExpirationHi(void) {
 return (*LINTCTL_STAT & 0x00020000);
}

int isAnyInterrupt(void) {

#ifdef NO_INTERRUPTS
 if (BoardsSync.ignoreTimHiPrdExpiration) {
   return (isCapModeExpirationLo() || isPrdExpirationLo());
 }
 else {
   return (isCapModeExpirationLo() || isPrdExpirationLo() || isPrdExpirationHi()); 
 }
#else
 return (isCapModeExpirationLo() || isPrdExpirationLo() || isPrdExpirationHi());
 //*LINTCTL_STAT & (0x00000008 | 0x00000002 | 0x00020000));
#endif
 
}

static void  calculateParams(BoardsSyncPllCalibrate_t *pllCal)
{
  pllCal->gain = (float) (DAC_HIGH_VAL - DAC_LOW_VAL) / 
                 (float) (pllCal->highValCount - pllCal->lowValCount);
  
  pllCal->dacZeroVal = round((float)DAC_HIGH_VAL - (pllCal->gain * (float) pllCal->highValCount));
}

#ifdef NO_CSL
static inline setTimerMode (int timmode)
{
 *LTGCR = (*LTGCR & 0xFFFFFFFC) | (timmode << 2);
}

static inline setTimerPrdLo(int prd)
{
  *LPRD_LO = prd;
}

static inline setTimerPrdHi(int prd)
{
  *LPRD_HI = prd;
}

static inline startTimerLo(int enaMode)
{
  *LTGCR |= 0x00000001; //un-reset the timer lo
  *LTCR = (*LTCR & 0xFFFFFF3F) | (enaMode << 6);
}

static inline startTimerHi(int enaMode)
{
  *LTGCR |= 0x00000002; //un-reset the timer hi
  *LTCR = (*LTCR & 0xFF3FFFFF) | (enaMode << (6+16));
}

static inline stopTimerLo(void)
{
  startTimerLo(0);
}

static inline stopTimerHi(void)
{
  startTimerHi(0);
}

static inline resetTimerLo(void)
{
  *LTGCR &= 0xFFFFFFFE;
}

static inline resetTimerHi(void)
{
  *LTGCR &= 0xFFFFFFFD;
}
#endif

static inline setTimerReloadLo(int reloadVal)
{
  *LREL_LO = reloadVal;
}

static inline setTimerReloadHi(int reloadVal)
{
  *LREL_HI = reloadVal;
}

static inline int isPpsLoss()
{
#ifdef DBG
       if (isPrdExpirationLo()) {
          gDbg.prdExpirationCount++;
       }
#endif
         /* PRD expiration */
  return(isPrdExpirationLo());
}

static inline isTimeDriftTooMuch (int timestampErr, BoardsSyncPll_t *pll)
{
  return ( (labs(timestampErr) * pll->gainCalculatedInCalibration) > pll->adjustmentLimit );
}

static inline float calcPpbError (float numerator, float denominator)
{
   return (numerator / denominator * 1e9);
}

extern void routeTimerLowOutToTimerOut0Pin(void);

void TimerInterruptHandler (void *arg)
{
    int capture, timestampErr, error, isUpdate;
    float errorInterval;
    BoardsSyncPllCalibrate_t *pllCal = &BoardsSync.pllCalibrate;

#ifdef DBG
    /* Increment the number of interrupts detected. */
    gDbg.timestampCounter++;    

// goto isrExit;
#endif

    /* skip first ISR, it happened at an arbitrary time relative to the last PPS */
    if (BoardsSync.firstIsr) {
      BoardsSync.firstIsr = 0;
      goto isrExit;
    }  
    
    if (isPpsLoss()) {
      BoardsSync.residualExpirationError += BoardsSync.ppsLossTimeCount;
      while (BoardsSync.residualExpirationError >= BoardsSync.timerCountsInOneSec) {
        BoardsSync.residualExpirationError -= BoardsSync.timerCountsInOneSec;
        BoardsSync.numOneSecsDuringPpsLoss++;
      }      

      if (BoardsSync.state == IN_LOCK) {
        if (BoardsSync.numOneSecsDuringPpsLoss >= BOARDS_SYNC_PPS_LOSS_COUNT_LIMIT) /*||
           (isUpdate && isTimingAccuracyPoor() )*/ {
          BoardsSync.state = HOLDOVER;

          resetPllConvergenceEstimation(&BoardsSync.pll.est);
         
          #ifdef DBG
            dbgLogState(BoardsSync.state);
          #endif         
        }
        else {
          updateAvgDacOut(&BoardsSync.pll.avgDacOut, BoardsSync.pll.lastDacOut);
        }
      }  
          
#ifdef ACCURATE_HOLDOVER
      if (BoardsSync.state == HOLDOVER) {
        float out;
        int dacout;
        BoardsSyncPll_t *pll = &BoardsSync.pll;
        
        out = pll->integratorGain * pll->integratorValue + pll->dacMidVal;

        if (pll->avgDacOut > out) {
          dacout = round(out) - 1;
        }
        else {
          dacout = round(out) + 1;
        }

        dacout = dac_write_val(dacout /*round(tmp) + pll->dacMidVal*/);
        updateAvgDacOut(&pll->avgDacOut, dacout);
        pll->lastDacOut = dacout;        
      }
#endif
      
      goto isrExit;
    }
    
    capture = *LCAP_LO;
    errorInterval = (float) (BoardsSync.numOneSecsDuringPpsLoss + 1) * (float) BoardsSync.pll.timerCountsInOneSec +
                    BoardsSync.residualExpirationError;
    error = (BoardsSync.pll.timerCountsInOneSec - BoardsSync.residualExpirationError) - capture;
    
    /* if instantaneous error too big [GNS output sometimes shifts abrubtly by 1 or 2 ms
       then only accumulate error in pll but don't update pll integrator or affect output */
    isUpdate = (calcPpbError (abs((float)error), errorInterval) < MAX_PPB_ANOMALY);
    
#ifdef DBG
  gDbg.captureBuf[gDbg.timestampCounter & (DBG_BUF_SIZE-1)] = capture;
#endif

    if (isCapModeExpirationLo() && (! isUpdate) && (BoardsSync.numOneSecsDuringPpsLoss == 0)) {
#ifdef DBG
      {
      int indx = gDbg.invalidCaptureBufIndx++ & (INVALID_CAPTURE_BUF_SIZE - 1);
      gDbg.invalidCaptureCount++;
      gDbg.invalidCaptureBuf[indx] = capture;
      gDbg.invalidCaptureIndex[indx] = gDbg.timestampCounter;
      } 
#endif      
    }
    
    switch (BoardsSync.state) {
      
      case PLL_CALIBRATE:
           
           if (BoardsSync.numOneSecsDuringPpsLoss || (! isUpdate)) {
              pllCal->samples = 0;
              BoardsSync.firstIsr = 1;
              goto isrExit1;
           }
           
           if (pllCal->samples == 0) {
             dac_write_val(DAC_LOW_VAL);
           }
           else if (pllCal->samples == 1) { /* leave a gap */
             pllCal->errorAccumulator = 0;
           }
           else if (pllCal->samples == (1 + NUM_PLL_CAL_SAMPLES)) {
             pllCal->lowValCount = round((float)pllCal->errorAccumulator/(float)NUM_PLL_CAL_SAMPLES);
             pllCal->errorAccumulator = 0;
             dac_write_val(DAC_HIGH_VAL);
           }
           else if (pllCal->samples == (1+NUM_PLL_CAL_SAMPLES +1)) { /* leave a gap of 1 sample */
             pllCal->errorAccumulator = 0;
           }
           else if (pllCal->samples == (1+NUM_PLL_CAL_SAMPLES+1 +NUM_PLL_CAL_SAMPLES)) {
             
             pllCal->highValCount = round((float)pllCal->errorAccumulator/(float)NUM_PLL_CAL_SAMPLES);
             calculateParams (pllCal); 
             dac_write_val(pllCal->dacZeroVal);
             configPllPostCalibration (&BoardsSync.pll, pllCal->dacZeroVal, pllCal->gain);
           }
           else if (pllCal->samples == (1+NUM_PLL_CAL_SAMPLES+1+NUM_PLL_CAL_SAMPLES +1)) { /* leave a gap of 1 sample */
             BoardsSync.state = INITIAL_LOCK;
             
             #ifdef DBG
               dbgLogState(BoardsSync.state);
             #endif                
           }
           else {
             pllCal->errorAccumulator += -error;
           }
           
           pllCal->samples++;
      break;
      
      case INITIAL_LOCK:
     
            if (BoardsSync.numOneSecsDuringPpsLoss >= BOARDS_SYNC_PPS_LOSS_COUNT_LIMIT) {
               /* too much error, could be due to pps loss before for a long time */
               freezePll(&BoardsSync.pll);
               configPll(&BoardsSync.pll, FAST);
               BoardsSync.firstIsr = 1;
               goto isrExit1;
            }
            
            timestampErr = processPll(&BoardsSync.pll, error, 0, isUpdate);

            if (isUpdate) {
              processPllConvergenceEstimation(&BoardsSync.pll, calcPpbError((float)timestampErr, errorInterval));
            }  
            
#if 1
            if (isPllConverged(&BoardsSync.pll)) {
               
              /* enter sync state */
              BoardsSync.state = SYNC_OUTPUT;
              BoardsSync.syncStateCount = 0;

              #ifdef DBG
                dbgLogState(BoardsSync.state);
              #endif               
            }
#endif

      break;
      
      case SYNC_OUTPUT:
            
             if (BoardsSync.syncStateCount == 0) {

               int timestampErr;
               
               if (BoardsSync.numOneSecsDuringPpsLoss) {
                  
                  BoardsSync.state = INITIAL_LOCK;
                  #ifdef DBG
                    dbgLogState(BoardsSync.state);
                  #endif
                                    
                  goto isrExit;
               }
                
               BoardsSync.pll.accumTime += (BoardsSync.timerCountsInOneSec - capture);
               timestampErr = BoardsSync.pll.accumTime;

               // stomas routeTimerLowOutToTimerOut0Pin();
 
#ifndef NO_INTERRUPTS
               //enable prd interrupt for timer hi
               CSL_intcHwControl(BoardsSync.tmrIntcHandleHigh, CSL_INTC_CMD_EVTENABLE, NULL);
#else
               BoardsSync.ignoreTimHiPrdExpiration = 0;
#endif
               {                 
#ifndef NO_CSL
                 CSL_TmrEnamode TimeCountMode = (CSL_TmrEnamode) 3; //continous mode
                 Aif2Fl_Status                  status;
#endif
                 uint32_t LoadValue = (BoardsSync.timerCountsInOneSec >> 1) - 1;

                 /* 0.5 sec timer high reload */
                 setTimerReloadHi(LoadValue);

#ifndef NO_CSL
                 status = CSL_tmrHwControl(BoardsSync.hTmr, CSL_TMR_CMD_LOAD_PRDHI, (void *)&LoadValue);
                 CSL_tmrHwControl(BoardsSync.hTmr, CSL_TMR_CMD_START_TIMHI, (void *)&TimeCountMode);
#else
                 setTimerPrdHi(LoadValue);
                 startTimerHi(3);
#endif                 
               }                 

               
               {
                 register volatile int tmp, tmp1, tmp2; //volatility of tmp, tmp1 for enforcing sequence
                 int reload, i, sum;
                
                                
                 OS_disable_interrupts();
                 
                 //for loop for code/temp var caching, to reduce cycle variability
                 #pragma UNROLL(1) //prevent unrolling
                 for (i = 0, sum = 0; i < 5; i++) { //averaging may improve accuracy      
                   tmp = *LTIM_LO;
                   tmp1 = *LTIM_LO;
                 
                   tmp2 = tmp1 - tmp;
                
                   tmp = *LTIM_LO;
                   tmp1 = *LTIM_HI;
                   
                   if (i != 0) { //ignore first iteration due to caching
                     sum += (tmp + tmp2) - tmp1;
                   }
                 } 

                 OS_enable_interrupts();
                                                   
#ifdef DBG
      gDbg.timestampErr = timestampErr;
#endif                                  
                 reload = BoardsSync.timerCountsInOneSec - ((BoardsSync.timerCountsInOneSec >> 1) + (sum >> 2)) +
                          timestampErr - 1;
       
                 setTimerReloadHi(reload);
               }
            }   
           else if (BoardsSync.syncStateCount == 1) {
           
             setTimerReloadHi(BoardsSync.timerCountsInOneSec - 1);
           
             /* must be done after the pulse from the 0.5 sec prd expiration 
                (that isn't enabled for output) has elapsed, since timer is /6 of DSP
                clock, and pulse is 4 timer clocks wide, we need at least a wait of 24
                DSP cycles, this is probably the interrupt overhead and processing until
                this point but it is good to be safe */
             {
               volatile int tmp;
               for(tmp = 0; tmp < 100; tmp++);
             }
              
            // routeTimerHighOutToTimerOut0Pin();

             BoardsSync.state = IN_LOCK;
           
             #ifdef DBG
               dbgLogState(BoardsSync.state);
             #endif
           }
           BoardsSync.syncStateCount++;
           
      break;
      
      case IN_LOCK:
      
           /* from sync state */
           if (isPrdExpirationHi()) { /* will always fail on one success because inside code runs to
                                         either disable timer or disable timer interrupt, so should not
                                         affect regular in-lock processing */
#ifndef SYNC_OUTPUT_DEBUG
#ifndef NO_CSL
             //disable timer high for a one-shot pulse
             {
               CSL_TmrEnamode TimeCountMode = CSL_TMR_ENAMODE_DISABLE;
               CSL_tmrHwControl(BoardsSync.hTmr, CSL_TMR_CMD_START_TIMHI, (void *)&TimeCountMode);
             }
#else

#endif             
#else
#ifndef NO_INTERRUPTS
             //disable prd interrupt for timer hi
             CSL_intcHwControl(BoardsSync.tmrIntcHandleHigh, CSL_INTC_CMD_EVTDISABLE, NULL);
#else
             BoardsSync.ignoreTimHiPrdExpiration = 1;
#endif             
#endif      
           }

           timestampErr = processPll(&BoardsSync.pll, error, 0, isUpdate);
           if (isUpdate) {
             processPllConvergenceEstimation(&BoardsSync.pll, calcPpbError((float)timestampErr, errorInterval));
           }
           
      break;
      
      case HOLDOVER:           
           
           timestampErr = processPll(&BoardsSync.pll, error, 0, 0);

           if (isUpdate && isTimeDriftTooMuch(timestampErr, &BoardsSync.pll)) {
              BoardsSync.state = RE_SYNC;
              resetPllConvergenceEstimation(&BoardsSync.pll.est);

              configPll(&BoardsSync.pll, FAST);
             

              #ifdef DBG
                dbgLogState(BoardsSync.state);
              #endif                   
           }
           else {
              BoardsSync.state = IN_LOCK;

              #ifdef DBG
                dbgLogState(BoardsSync.state);
              #endif              
           }      
      break;
      
      case RE_SYNC:

           timestampErr = processPll(&BoardsSync.pll, error, BoardsSync.enableControlledRateOfAdjustment, isUpdate);
           
           if (isUpdate && isTimeDriftTooMuch(timestampErr, &BoardsSync.pll)) {
             /* prevent speed transition if time drift is not reduced sufficiently yet */
             BoardsSync.pll.numSamplesState = 0;
           }

           if (isUpdate) {
             processPllConvergenceEstimation(&BoardsSync.pll, calcPpbError((float)timestampErr, errorInterval));
           }  

           if (isPllConverged(&BoardsSync.pll)) {
              BoardsSync.state = IN_LOCK;

              #ifdef DBG
                dbgLogState(BoardsSync.state);
              #endif                   
           }
      
      break;
    }

isrExit1:

    BoardsSync.numOneSecsDuringPpsLoss = 0;
    BoardsSync.residualExpirationError = 0;
    
isrExit:
    
    *LINTCTL_STAT = *LINTCTL_STAT;
    
#if 0
#ifndef PRD_CMP_ONLY
    *LINTCTL_STAT |= 0x0000000A;
#else
    *LINTCTL_STAT |= 0x00000002;
#endif
#endif

#ifndef NO_INTERRUPTS
    /* Clear the event ID. */
    CSL_intcEventClear((CSL_IntcEventId)arg);
#endif    
}

static inline enableNewFeaturesOfTimer(void)
{
    /* enable new features of Timer64p, BW_COMPATIBLE = 1 */
    *LTGCR |= 0x00000010;
}

static inline configTimerLo(int LoadValue)
{
#if 0
    /* READRSTMODE = 1, CAPMODE = 1, CAPEVTMODE = 00 (rising edge) */
    *LTCR &= 0xFFFFCFFF;
    *LTCR |= 0x00000C00;
#else
    /* READRSTMODE = 0, CAPMODE = 1, CAPEVTMODE = 00 (rising edge) */
    *LTCR &= 0xFFFFCBFF;
    *LTCR |= 0x00000800;
#endif

    /* interrupt control and status register
     * CMP_INT_EN = 1, EVT_INT_EN = 1, CMP_INT_STAT = 1, EVT_INT_STAT = 1 */
    *LINTCTL_STAT |= 0x0000000F;

    setTimerReloadLo(LoadValue);

    /* CP_LO = 0 (pulse mode), PWID_LO = 11b (4 clock cycles) */
    *LTCR &= 0xFFFFFFF7;
    *LTCR |= 0x00000030;

#if 0 // clock mode
    *LTCR |= 0x00000008;
#endif    
}

static inline configTimerHi(int LoadValue)
{
    setTimerReloadHi(LoadValue);

    /* CAPMODE = 0 */
    *LTCR &= 0xF7FFFFFF;

    /* interrupt control and status register
     * CMP_INT_EN = 1, EVT_INT_EN = 0, CMP_INT_STAT = 1 */
    *LINTCTL_STAT &= 0xFFFBFFFF;
    *LINTCTL_STAT |= 0x00030000;

    /* CP_LO = 0 (pulse mode), PWID_LO = 11b (4 clock cycles) */
    *LTCR &= 0xFFF7FFFF;
    *LTCR |= 0x00300000;

#ifdef CLOCK_MODE // clock mode
    *LTCR |= 0x00080000;
#endif
}

#ifndef NO_INTERRUPTS
/**
 *  @b Description
 *  @n  
 *      The functions initializes the INTC module.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t intc_init (void)
{
    CSL_IntcGlobalEnableState   state;

    /* INTC module initialization */
    contextLow.eventhandlerRecord = EventHandlerLow;
    contextLow.numEvtEntries      = 10;
    if (CSL_intcInit(&contextLow) != AIF2FL_SOK) 
        return -1;

    contextHigh.eventhandlerRecord = EventHandlerHigh;
    contextHigh.numEvtEntries      = 10;
    if (CSL_intcInit(&contextHigh) != AIF2FL_SOK) 
        return -1;

    /* Enable NMIs */
    if (CSL_intcGlobalNmiEnable() != AIF2FL_SOK) 
        return -1;
 
    /* Enable global interrupts */
    if (CSL_intcGlobalEnable(&state) != AIF2FL_SOK) 
        return -1;

    /* INTC has been initialized successfully. */
    return 0;
}
#endif

#ifndef NO_CSL
int configTimer (uint8_t IntcInstance, int ppsLossTimeCount)
#else
int configTimer (int ppsLossTimeCount)
#endif
{
#ifndef NO_CSL
    CSL_TmrHandle               hTmr;
    Aif2Fl_Status                  status;
    CSL_TmrHwSetup              hwSetup = CSL_TMR_HWSETUP_DEFAULTS;
    CSL_TmrEnamode              TimeCountMode = (CSL_TmrEnamode) 3; //CSL_TMR_ENAMODE_CONT;
#endif

#ifndef NO_INTERRUPTS
    CSL_IntcEventHandlerRecord  EventRecord;
    CSL_IntcParam               vectId;
    CSL_IntcHandle              tmrIntcHandleLow, tmrIntcHandleHigh;
#endif

    uint32_t                      LoadValue;

    LoadValue = ppsLossTimeCount - 1;
    
#ifndef NO_CSL
    /* Clear local data structures */
    memset(&BoardsSync.TmrObj, 0, sizeof(CSL_TmrObj));
#endif    

#ifndef NO_INTERRUPTS
    /**************************************************************
     ********************** INTC related code *********************
     **************************************************************/

    /************** TIMER LO ***********************************/
    /* Open INTC */
    vectId = CSL_INTC_VECTID_12;    
    tmrIntcHandleLow = CSL_intcOpen(&tmrIntcObjLow, TINT_LO, &vectId, NULL);
    if (tmrIntcHandleLow == NULL)
        return -1;

    /* Bind ISR to Interrupt */
    EventRecord.handler = (CSL_IntcEventHandler)&TimerInterruptHandler;
    EventRecord.arg     = (void *)TINT_LO;
    CSL_intcPlugEventHandler(tmrIntcHandleLow, &EventRecord);


#if 1
    /************* TIMER HI **************************************/
    /* Open INTC */
    vectId = CSL_INTC_VECTID_13;
    tmrIntcHandleHigh = CSL_intcOpen(&tmrIntcObjHigh, TINT_HI, &vectId, NULL);
    if (tmrIntcHandleHigh == NULL)
        return -1;

    BoardsSync.tmrIntcHandleHigh = tmrIntcHandleHigh;
    
    /* Bind ISR to Interrupt */
    EventRecord.handler = (CSL_IntcEventHandler)&TimerInterruptHandler;
    EventRecord.arg     = (void *)TINT_HI;
    CSL_intcPlugEventHandler(tmrIntcHandleHigh, &EventRecord);

    /* Event Enable */
    CSL_intcHwControl(tmrIntcHandleHigh, CSL_INTC_CMD_EVTENABLE, NULL);
#endif
#endif

    /**************************************************************
     ********************** Timer related code ********************
     **************************************************************/
#ifndef NO_CSL
    /* Open the timer. */
    hTmr =  CSL_tmrOpen(&BoardsSync.TmrObj, IntcInstance, NULL, &status);
    if (hTmr == NULL)
        return -1;

    BoardsSync.hTmr = hTmr;
    
    /* Open the timer with the defaults. */
    CSL_tmrHwSetup(hTmr, &hwSetup);

    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_STOP_TIMLO, NULL);
    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_STOP_TIMHI, NULL);

    /* Reset the Timer */
    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_RESET_TIMLO, NULL);
    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_RESET_TIMHI, NULL);

    /* Set the timer mode to unchained dual mode */
    hwSetup.tmrTimerMode = CSL_TMR_TIMMODE_DUAL_UNCHAINED;
    CSL_tmrHwSetup(hTmr, &hwSetup);
#else
    stopTimerLo();
    stopTimerHi();
    resetTimerLo();
    resetTimerHi();
    setTimerMode(1);
#endif

    enableNewFeaturesOfTimer();
    
#ifndef NO_CSL
    /* Load the period register */
    status = CSL_tmrHwControl(hTmr, CSL_TMR_CMD_LOAD_PRDLO, (void *)&LoadValue);
#else
    setTimerPrdLo(LoadValue);
#endif

    configTimerLo(LoadValue);
    
#ifndef NO_CSL
    /* Load the period register */
    status = CSL_tmrHwControl(hTmr, CSL_TMR_CMD_LOAD_PRDHI, (void *)&LoadValue);
#else
    setTimerPrdHi(LoadValue);
#endif

    configTimerHi(LoadValue);

#ifndef NO_INTERRUPTS
    CSL_intcHwControl(tmrIntcHandleLow, CSL_INTC_CMD_EVTENABLE, NULL);

#if 1
    CSL_intcHwControl(tmrIntcHandleHigh, CSL_INTC_CMD_EVTENABLE, NULL);
#endif
#endif

#ifndef NO_CSL
    /* This should result in output pin showing 0 from previous high-z */
    TimeCountMode = CSL_TMR_ENAMODE_DISABLE;
    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_START_TIMHI, (void *)&TimeCountMode);

    TimeCountMode = (CSL_TmrEnamode)3; //continous mode
    CSL_tmrHwControl(hTmr, CSL_TMR_CMD_START_TIMLO, (void *)&TimeCountMode);
#else
    stopTimerHi();
    startTimerLo(3);
#endif    

    return(0);
}

#endif
