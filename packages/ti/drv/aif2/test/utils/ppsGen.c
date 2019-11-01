#if EVM_TYPE == 4

/*   @file gpsSync.c
 *
 *   @brief   
 *      GPS synchronization software.
 *
 *   Copyright (c) 2011 Texas Instruments, Inc.
 *   All Rights Reserved.
 *
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//#include <xdc/runtime/System.h>
//#include <ti/sysbios/hal/Hwi.h>
#include <ti/csl/csl.h>



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

#define TIMER_0 //BIOS uses timer 0, timer 4 through 7 are global timers


#define TOUTPSEL ((volatile int *)0x02620304)

#define TCR_OFFSET 0x20
#define TGCR_OFFSET 0x24
#define TIM_LO_OFFSET 0x10
#define TIM_HI_OFFSET 0x14
#define PRD_LO_OFFSET 0x18
#define PRD_HI_OFFSET 0x1C

#ifdef TIMER_0
#define TIMER_BASE CSL_TIMER_0_REGS
#define TINT_LO CSL_GEM_TINTLN
#define TINT_HI CSL_GEM_TINTHN
#endif

#ifdef TIMER_4
#define TIMER_BASE CSL_TIMER_4_REGS
#define TINT_LO CSL_GEM_TINT4L
#define TINT_HI CSL_GEM_TINT4H
#endif


#define TCR ((volatile int *)(TIMER_BASE + TCR_OFFSET))
#define TGCR ((volatile int *)(TIMER_BASE + TGCR_OFFSET))
#define INTCTL_STAT ((volatile int *)(TIMER_BASE + INTCTL_STAT_OFFSET))
#define TIM_LO ((volatile int *)(TIMER_BASE + TIM_LO_OFFSET))
#define TIM_HI ((volatile int *)(TIMER_BASE + TIM_HI_OFFSET))
#define PRD_LO ((volatile int *)(TIMER_BASE + PRD_LO_OFFSET))
#define PRD_HI ((volatile int *)(TIMER_BASE + PRD_HI_OFFSET))

#define TIMER_163PT8_MHZ

#define SCALE_SEC 16 /* must be power of 2 */

#ifdef TIMER_204PT8_MHZ /* 122.88*10/6 = 204.8 MHz */
#define TIMER_COUNTS_IN_1SEC  (204800000 /SCALE_SEC)
#endif

#ifdef TIMER_163PT8_MHZ /* 122.88*8/6  = 163.84 MHz */
#define TIMER_COUNTS_IN_1SEC  (163840000/SCALE_SEC) //163840000
#endif

void routeTimerLowOutToTimerOut0Pin(void)
{
#ifdef TIMER_0
   *TOUTPSEL = 0;
#endif

#ifdef TIMER_4
   *TOUTPSEL = 8;
#endif   
}

static inline setTimerMode (int timmode)
{
 *TGCR = (*TGCR & 0xFFFFFFFC) | (timmode << 2);
}

static inline setTimerPrdLo(int prd)
{
  *PRD_LO = prd;
}

#if 0
static inline setTimerPrdHi(int prd)
{
  *PRD_HI = prd;
}
#endif

static inline startTimerLo(int enaMode)
{
  *TGCR |= 0x00000001; //un-reset the timer lo
  *TCR = (*TCR & 0xFFFFFF3F) | (enaMode << 6); 
}

#if 0
static inline startTimerHi(int enaMode)
{
  *TGCR |= 0x00000002; //un-reset the timer hi
  *TCR = (*TCR & 0xFF3FFFFF) | (enaMode << (6+16));
}
#endif

static inline stopTimerLo(void)
{
  startTimerLo(0);
}

#if 0
static inline stopTimerHi(void)
{
  startTimerHi(0);
}
#endif

static inline resetTimerLo(void)
{
  *TGCR &= 0xFFFFFFFE;
}

#if 0
static inline resetTimerHi(void)
{
  *TGCR &= 0xFFFFFFFD;
}
#endif

static inline configTimerLo(int LoadValue)
{
    setTimerPrdLo(LoadValue);
    
    /* chip reset value */ 
    //*TCR = 0;
     
    /* CP_LO = 0 (pulse mode), PWID_LO = 11b (4 clock cycles) */
    *TCR &= 0xFFFFFFF7;
    *TCR |= 0x00000030;

#if 1 // clock mode
    *TCR |= 0x00000008;
#endif
}

void chipResetTimer(void)
{
	*TCR = 0;
	*TGCR = 0;
}
int ppsGen (void)
{
    uint32_t  LoadValue;
	int timeCount=TIMER_COUNTS_IN_1SEC/2;
    LoadValue = timeCount - 1;
    chipResetTimer();
    stopTimerLo();
    resetTimerLo();
    setTimerMode(1);
    configTimerLo(LoadValue);
    stopTimerLo();
    startTimerLo(2);
  
    return(0);
}

#if 0
extern cregister volatile unsigned int TSCL;

#define PERIOD_BUF_SIZE 16384
int gPeriod[PERIOD_BUF_SIZE] = {0}, gPeriodIndx = 0;
int gCount = 0;
void main (void)
{
	volatile int temp, temp1;
	volatile unsigned int oldval, newval;

	TSCL = 0;

    routeTimerLowOutToTimerOut0Pin();
    ppsGen(TIMER_COUNTS_IN_1SEC/2); //divisiblity by 2 assumed

    //get to a transition
    while((*TCR & 1) == 0);
    while((*TCR & 1) == 1);
    oldval = 0;

    while(1) {
    	temp = (int)TSCL;
    	while ((newval = (*TCR & 1)) == oldval);
    	temp1 = (int)TSCL - temp;
    	oldval = newval;
    	gPeriod[gPeriodIndx++ & (PERIOD_BUF_SIZE - 1)] = temp1;
    	gCount++;
    }
}
#endif

#endif
