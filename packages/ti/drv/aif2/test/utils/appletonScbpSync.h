#if EVM_TYPE == 5 || EVM_TYPE == 7
#ifndef __BOARDS_SYNC_H
#define __BOARDS_SYNC_H

//enable one of the below
//#define EVM
#define C6614_EVM

#define NO_INTERRUPTS
#define NO_CSL
//#define DSP_SYSBIOS
//#define TEST_HOLDOVER

#define ACCURATE_HOLDOVER  1  //for experimental purposes only,
                           //useful if DAC resolution is poorer than VCXO and crystal's accuracy and stability.

#define BOARDSSYNC_AVG_ERR_PPB_REQUIREMENT 20 /* ppb */
#define BOARDSSYNC_MAX_ERR_PPB_REQUIREMENT 150 /* ppb */
#define BOARDSSYNC_RESYNC_STATE_MAX_TIMING_ADJUSTMENT_RATE 0.1 /* parts per millionth of second */
#define BOARDSSYNC_ENABLE_CONTROLLED_RATE_OF_TIMING_ADJUSTMENT 0 /* set to 1 to enable, 0 to disable */
#define SYNC_OUTPUT_DEBUG /* define to generate a square PPS output clock, comment this out to generate a one-time pulse */

//derivative gains are all set to 0 currently, not using derivative component,
//derivative component is meant for experimentation
#define INTEGRATOR_GAIN_SCALE_FAST (1.0/10.0)
#define DERIVATIVE_GAIN_SCALE_FAST 0

#define INTEGRATOR_GAIN_SCALE_MEDIUM (1.0/100.0)
#define DERIVATIVE_GAIN_SCALE_MEDIUM 0

#define INTEGRATOR_GAIN_SCALE_SLOW (1.0/500.0)
#define DERIVATIVE_GAIN_SCALE_SLOW 0

#define ERROR_GAIN_SCALE_MEDIUM (1.0/5.0)
#define ERROR_GAIN_SCALE_SLOW (1.0/50.0)

#define PROCESS_MASK_FAST     0
#define PROCESS_MASK_MEDIUM   0
#define PROCESS_MASK_SLOW     0

#define SAMPLES_IN_FAST_STATE 20
#define SAMPLES_IN_MEDIUM_STATE 40
#define SAMPLES_IN_SLOW_STATE 60

#define NUM_PLL_CAL_SAMPLES 16

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef DSP_SYSBIOS
#include <xdc/runtime/System.h>
#include <ti/sysbios/hal/Hwi.h>
#endif

#ifdef LINUX

#endif

#ifndef NO_CSL
#include <ti/csl/csl_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#include <ti/csl/cslr_device.h>
#else
#define CSL_TIMER_0_REGS         (0x02200000)
#define CSL_TIMER_1_REGS         (0x02210000)
#define CSL_TIMER_2_REGS         (0x02220000)
#define CSL_TIMER_3_REGS         (0x02230000)
#define CSL_TIMER_4_REGS         (0x02240000)
#define CSL_TIMER_5_REGS         (0x02250000)
#define CSL_TIMER_6_REGS         (0x02260000)
#define CSL_TIMER_7_REGS         (0x02270000)

#define CSL_GEM_TINTLN           (0x00000040)
#define CSL_GEM_TINTHN           (0x00000041)
#define CSL_GEM_TINT4L           (0x00000042)
#define CSL_GEM_TINT4H           (0x00000043)
#define CSL_GEM_TINT5L           (0x00000044)
#define CSL_GEM_TINT5H           (0x00000045)
#define CSL_GEM_TINT6L           (0x00000046)
#define CSL_GEM_TINT6H           (0x00000047)
#define CSL_GEM_TINT7L           (0x00000048)
#define CSL_GEM_TINT7H           (0x00000049)
#endif

#ifndef NO_INTERRUPTS
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/src/intc/csl_intcAux.h>
#endif

#include "scbpfpga_spi.h"

#ifdef SYNC_OUTPUT_DEBUG
#define CLOCK_MODE
#endif

#define TINPSEL1
//#define TINPSEL0

//#define TINPSEL ((volatile int *)0x02620300)

//#define TOUTPSEL ((volatile int *)0x02620304)

//#define CSL_BOOT_CFG_REGS        (0x02620000)

#define TCR_OFFSET 0x20
#define TGCR_OFFSET 0x24
#define INTCTL_STAT_OFFSET 0x44
#define CAP_LO_OFFSET 0x3C
#define REL_LO_OFFSET 0x34
#define REL_HI_OFFSET 0x38
#define TIM_LO_OFFSET 0x10
#define TIM_HI_OFFSET 0x14
#define PRD_LO_OFFSET 0x18
#define PRD_HI_OFFSET 0x1C

#define TIMER_BASE CSL_TIMER_0_REGS

#define TIMER_163PT8_MHZ

#define SCALE_SEC 16 /* must be power of 2 */
#define MAX_PPB_ANOMALY (70000) /* ppb, previously 50000 */

#ifdef TIMER_204PT8_MHZ /* 122.88*10/6 = 204.8 MHz */
#define TIMER_COUNTS_IN_1SEC  (204800000/SCALE_SEC)
//#define CAP_ACCURACY_MAX_DEVIATION (102/SCALE_SEC) /* 500 ppb */
#endif

#ifdef TIMER_163PT8_MHZ /* 122.88*8/6  = 163.84 MHz */
#define TIMER_COUNTS_IN_1SEC  (163840000/SCALE_SEC) //163840000
//#define CAP_ACCURACY_MAX_DEVIATION (82/SCALE_SEC) /* 500 ppb */
#endif

#define BOARDS_SYNC_PPS_LOSS_COUNT_LIMIT 10 /* seconds */

//#define CAP_MIN_TIMING_ACCURACY (TIMER_COUNTS_IN_1SEC - CAP_ACCURACY_MAX_DEVIATION)
//#define CAP_MAX_TIMING_ACCURACY (TIMER_COUNTS_IN_1SEC + CAP_ACCURACY_MAX_DEVIATION)

#if EVM_TYPE == 7
/* Vref = VCC5 = 5.0 V
   vout = vref/2 + vref x D/65536 [D = -32768..32767]
   Rakon OCXO needs 0.5 - 2.5 V range.
   vout = 0.5, D = -26214
   vout = 2.5, D = 0 
 */
#define DAC_LOW_VAL -26214 /* corresponding to 0.5 V */
#define DAC_HIGH_VAL 0 /* corresponding to 2.5 V */
#define DAC_MID_VAL ((DAC_LOW_VAL + DAC_HIGH_VAL) >> 1)
#define DAC_INIT_VAL DAC_MID_VAL
#elif EVM_TYPE == 5
#define DAC_LOW_VAL 500 /* corresponding to 0.5 V */
#define DAC_HIGH_VAL 2500 /* corresponding to 2.5 V */
#define DAC_MID_VAL ((DAC_LOW_VAL + DAC_HIGH_VAL) >> 1)
#define DAC_INIT_VAL DAC_MID_VAL
#endif
#define DAC_ADDR 0x0011

#define CHIP_LEVEL_REG  0x02620000
#define MAINPLLCTL	*(unsigned int*)(CHIP_LEVEL_REG + 0x0328)

//#define DBG_NO_DAC_WRITE
//#define DBG

//debug defs
//#define PRD_CMP_ONLY

#ifndef NO_INTERRUPTS
/* INTC Objects */ 
CSL_IntcObj                  tmrIntcObjLow;
CSL_IntcEventHandlerRecord   EventHandlerLow[30];
CSL_IntcContext              contextLow;

CSL_IntcObj                  tmrIntcObjHigh;
CSL_IntcEventHandlerRecord   EventHandlerHigh[30];
CSL_IntcContext              contextHigh;

/* Global Variable for the INTC Module; useful for debugging. */
CSL_IntcRegsOvly    gIntcRegisters    = (CSL_IntcRegsOvly)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS;
#endif

typedef enum {
    PLL_CALIBRATE,
	INITIAL_LOCK,
	SYNC_OUTPUT,
    IN_LOCK,
    HOLDOVER,
    RE_SYNC
} BoardsSyncState_e;

typedef enum {
  FAST,
  MEDIUM,
  SLOW
} BoardsSyncPllState_e;

typedef struct {
  int lowValCount;
  int highValCount;
  int errorAccumulator;
  int samples;
  int dacZeroVal;
  float gain;  
} BoardsSyncPllCalibrate_t;

#define BOARDSSYNC_PLL_ERROR_HISTORY_SIZE 32 /* must be power of 2 */

typedef struct {
  float errorAccumulator;
  float errorHistory[BOARDSSYNC_PLL_ERROR_HISTORY_SIZE];
  int errorHistoryIndex;
  float avgErrThreshold;
  float maxErrThreshold;
  int converged; /* 1 is converged, 0 otherwise */
} BoardsSyncPllEstimateConvergence_t;

typedef struct {
  BoardsSyncPllState_e state;
  int dacMidVal;
  int timerCountsInOneSec;
  int numSamplesState;
  int processCount;
  int processMask;
  int accumTime;
  int prevError;
  int integratorValue;
  float integratorGain;
  float derivativeGain;
  float errorGain;
  float adjustmentLimit;
  float resyncStateMaxTimingAdjustmentRate;
  float gainCalculatedInCalibration;

#ifdef ACCURATE_HOLDOVER
  float avgDacOut;
  int lastDacOut;
#endif
  
  int resetAvgDacOutFlag;
  BoardsSyncPllEstimateConvergence_t est;
} BoardsSyncPll_t;

typedef struct {
  BoardsSyncState_e state;
  int syncStateCount;
  int firstIsr;
  int timerCountsInOneSec;
  int numOneSecsDuringPpsLoss;
  int residualExpirationError;
  int ppsLossTimeCount;
  int enableControlledRateOfAdjustment;
  BoardsSyncPllCalibrate_t pllCalibrate;
  BoardsSyncPll_t pll;
  
#ifndef NO_INTERRUPTS
  CSL_IntcHandle tmrIntcHandleHigh;
#else
  int ignoreTimHiPrdExpiration;
#endif
  
#ifndef NO_CSL
  CSL_TmrObj     TmrObj;
  CSL_TmrHandle  hTmr;
#endif  
} BoardsSync_t;


#ifdef DBG
typedef struct {
  int converged;
  int timestampCount;
  float maxError;
  float movingAverageError;
} convergence_t;

typedef struct {
  BoardsSyncState_e state;
  int timestampCount;
} stateLog_t;

typedef struct {
  BoardsSyncPllState_e state;
  int timestampCount;
} pllStateLog_t;

/* Debug structure */
typedef struct {

#define DBG_BUF_SIZE 2048
  short dacinpBuf[DBG_BUF_SIZE];
  int captureBuf[DBG_BUF_SIZE];
  int errorBuf[DBG_BUF_SIZE];
  int integratorBuf[DBG_BUF_SIZE];
  
  #define STATE_BUF_SIZE 128
  stateLog_t stateBuf[STATE_BUF_SIZE];
  int stateBufIndx;

  #define PLL_STATE_BUF_SIZE 128
  pllStateLog_t pllStateBuf[PLL_STATE_BUF_SIZE];
  int pllStateBufIndx;
  
  #define INVALID_CAPTURE_BUF_SIZE 128
  int invalidCaptureCount;
  int invalidCaptureBuf[INVALID_CAPTURE_BUF_SIZE];
  int invalidCaptureIndex[INVALID_CAPTURE_BUF_SIZE];
  int invalidCaptureBufIndx;
  
  int prdExpirationCount;
  int timestampCounter;

  float movingAvgErrCounts0to5ppb;
  float movingAvgErrCounts5to10ppb;
  float movingAvgErrCounts10to15ppb;
  float movingAvgErrCounts15to20ppb;
  float movingAvgErrCountsHigherThan20ppb;
  int enableHistogramCalculation;

#define CONVERGENCE_BUF_SIZE 256  
  convergence_t convergence[CONVERGENCE_BUF_SIZE];
  int convergenceIndex;

  int timestampErr; //during sync generation
  
  float avgDacOutBuf[DBG_BUF_SIZE];
  
} dbg_t;
#endif

#ifdef DBG
extern dbg_t gDbg;
#endif
extern BoardsSync_t BoardsSync;


#ifndef __BOARDS_SYNC_C
extern
#endif
void BoardsSyncInit (BoardsSync_t *BoardsSync, int timerCountsInOneSec,
                  int avgErrPpbRequirement, int maxErrPpbRequirement,
                  int enableControlledRateOfAdjustment,
                  float resyncStateMaxTimingAdjustmentRate,
                  int timer);
#ifndef __BOARDS_SYNC_C
extern
#endif
int dac_write_val(int x);

#ifndef __BOARDS_SYNC_C
extern
#endif
void switchOffPps(void);

#ifndef __BOARDS_SYNC_C
extern
#endif
void switchOnPps(void);

#ifndef NO_CSL
#ifndef __BOARDS_SYNC_C
extern
#endif
int configTimer (uint8_t IntcInstance, int ppsLossTimeCount);
#else
#ifndef __BOARDS_SYNC_C
extern
#endif
int configTimer (int ppsLossTimeCount);
#endif

#ifndef __BOARDS_SYNC_C
extern
#endif
int isAnyInterrupt(void);

#ifndef __BOARDS_SYNC_C
extern
#endif
void TimerInterruptHandler (void *arg);

#endif //__BOARDS_SYNC_H

#endif
