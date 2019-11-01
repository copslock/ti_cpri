/*****************************************************************************
*           TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION           
*                                                                            
*  Property of Texas Instruments 
*  For  Unrestricted  Internal  Use  Only 
*  Unauthorized reproduction and/or distribution is strictly prohibited.  
*  This product is protected under copyright law and trade secret law 
*  as an unpublished work.  
*  Created 2011, (C) Copyright 2011 Texas Instruments.  All rights reserved.
*------------------------------------------------------------------------------
*  Filename       : psc_util.c
*  Description    : Functions to enable specific module in PSC
*  History        : 
*  Ported by      : Renuka/Mousumi
*  Project        : Kepler, Lamarr
*  Date           : Oct 2011
*****************************************************************************/
#define CONFIG_SRC
#include <psc_util.h>
#include <psc.h>
#include <psc_vars.h>
#ifndef PSC_TIMEOUT
#define PSC_TIMEOUT 1000
#endif

#ifndef PSC_PD_TIMEOUT
  #define PSC_PD_TIMEOUT 50
#endif

#ifndef EXTRA_PWRDWN_DELAY
  #define EXTRA_PWRDWN_DELAY 800
#endif

#define MAX_PD		32
#include <constants.h>
const unsigned int max_pd = MAX_PD;
unsigned int mds_in_pd[MAX_PD][MDLIMIT_PER_PD] = {
	{1, 2, 3, 4, 0},
	{5, 6, 0},
	{8, 9, 7, 0},
	{10, 0},
	{11, 0},
	{12, 0},
	{13, 0},
	{14, 0},
	{15, 16, 0},
	{17, 0},
	{18, 0},
	{19, 0},
	{20, 21, 22, 0},
	{23, 0},
	{24, 25, 0},
	{26, 27, 0},
	{28, 0},
	{29, 0},
    {30, 0},
};

unsigned int enable_all()
{
   unsigned int pd_mask = 0x1;
    volatile unsigned int *ptstat = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTSTAT);
    volatile unsigned int *mdctl = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_MDCTL_BASE);
    volatile unsigned int *pdctl = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_PDCTL_BASE);
    int pdnum,mdnum;
    unsigned int loop_cnt = 0;

    // Get ready for enable_module if necessary
    #ifdef pre_enable_module
      pre_enable_module;
    #endif


    for(pdnum=0; pdnum < max_pd; pdnum++) {
	pdctl[pdnum] |= 1;
	pd_mask |= (1 << pdnum);
    }
    for(mdnum=0; mdnum < max_md; mdnum++) {
	mdctl[mdnum] |= 3;
    }
    // start the process and wait. but timeout in 1000 loops.
    *(volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTCMD) |= pd_mask;
#ifdef PSC_NO_WAIT
    return(PASS);
#endif

    while(((*ptstat & pd_mask) != 0) && (loop_cnt < PSC_TIMEOUT))
    	{
    		loop_cnt++;
    	}
    if ((*ptstat & pd_mask) == 0) return(PASS);
    return(FAIL);
}

unsigned int enable_module(unsigned int pdctl, unsigned int mdctl)
{
  unsigned int pd_mask = 0x1;
  unsigned int loop_cnt = 0;
    volatile unsigned int *ptstat = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTSTAT);
    volatile unsigned int *mdstat = (volatile unsigned int *)(PSC_START_ADDRESS +(mdctl-PSC_MDCTL_BASE+PSC_MDSTAT_BASE));

    if (mdctl == NULL) return(FAIL);

    // Get ready for enable_module if necessary
    #ifdef pre_enable_module
      pre_enable_module;
    #endif

    // Is module already enabled?
    if ((*mdstat & 0x3F) == 0x3) return(PASS);

    // program pdctl and mdctl to enable the module. 
    if (pdctl) {
        pd_mask = (0x1 << ((pdctl-PSC_PDCTL_BASE)>>2));
        *(volatile unsigned int *)(PSC_START_ADDRESS +pdctl) |= 0x1;
    }
    *(volatile unsigned int *)(PSC_START_ADDRESS +mdctl) |= 0x3;

    // start the process and wait. but timeout in 1000 loops.
    *(volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTCMD) = pd_mask;
#ifdef PSC_NO_WAIT
    return(PASS);
#endif

    while(((*ptstat & pd_mask) != 0) && (loop_cnt < PSC_TIMEOUT))
    	{
    		loop_cnt++;
    	}

    // report result. 
    return(((*mdstat & 0x3F) == 0x3)? PASS : FAIL);   
}

unsigned int disable_module(unsigned int pdctl, unsigned int mdctl)
{
   unsigned int pd_mask = 0x1;
   unsigned int loop_cnt = 0;
    unsigned int force_pwrdwn = 0;
    volatile unsigned int *pdstat;
    volatile unsigned int *ptstat = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTSTAT);
    volatile unsigned int *mdstat = (volatile unsigned int *)(PSC_START_ADDRESS +(mdctl-PSC_MDCTL_BASE+PSC_MDSTAT_BASE));

    if (mdctl == NULL) return(FAIL);

    // Get ready for disable_module if necessary
    #ifdef pre_disable_module
      pre_disable_module;
    #endif


    // Is module already powered down?
    if (pdctl) {
        pdstat = (volatile unsigned int *)(PSC_START_ADDRESS +(pdctl-PSC_PDCTL_BASE+PSC_PDSTAT_BASE));
        if ((*pdstat & 0x1F) == 0x0) return(PASS);
        pd_mask = (0x1 << ((pdctl-PSC_PDCTL_BASE)>>2));
	force_pwrdwn = 0;
    }

    // Is module already disabled
    if (((*mdstat & 0x3F) == 0x0) || ((*mdstat & 0x3F) == 0x2)) return(PASS);


    if (force_pwrdwn && pdctl) *(volatile unsigned int *)(PSC_START_ADDRESS +pdctl) &= ~0x1;
    *(volatile unsigned int *)(PSC_START_ADDRESS +mdctl) &= ~0x3F;

    // start the transitions and wait. but timeout in 1000 loops.
    *(volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTCMD) = pd_mask;
#ifdef PSC_NO_WAIT
    return(PASS);
#endif
    while(((*ptstat & pd_mask) != 0) && (loop_cnt < PSC_TIMEOUT))
    	{
    		loop_cnt++;
    	}

    return(((*mdstat & 0x3F) == 0x0)? PASS : FAIL);  
}

#ifdef PSC_LRST_SUPPORT
void reset_module(unsigned int pdctl, unsigned int mdctl)
{
}
#endif


#ifdef PSC_PD_SUPPORT
unsigned int power_down(unsigned int pdctl)
{
   //    unsigned int save0 = GPLYA, save1 = GPLYB;
    volatile unsigned int *ptstat = (volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTSTAT);
    volatile unsigned int mdctl;
    volatile unsigned int *pdstat = (volatile unsigned int *)(PSC_START_ADDRESS +(pdctl-PSC_PDCTL_BASE+PSC_PDSTAT_BASE));
    unsigned int i, pd_mask = 0x1, pd_num = ((pdctl-PSC_PDCTL_BASE)>>2);
    unsigned int loop_cnt = 0;

    //    GPLYA = pd_num;
    GPLYB = 0xFFFFFFFF;
    if (pd_num == NULL) return(FAIL);
    if ((*pdstat & 0x1F) == 0x0) return(PASS);

// From max_md down to 1, to power down PASS after CRYPTO and SGMII
    // 0 is alltime on; be careful, i is unsigned int
    for(i=(max_md-1); i > 0; i--) {
         GPLYB = md2pd[i];
      if (md2pd[i] == pd_num) {
          mdctl = (PSC_MDCTL_BASE + (i << 2));
          if (disable_module(pdctl, mdctl) != PASS) return(FAIL);
    }
    *(volatile unsigned int *)(PSC_START_ADDRESS +pdctl) &= ~0x1;

    pd_mask = (0x1 << pd_num);
    *(volatile unsigned int *)(PSC_START_ADDRESS +PSC_PTCMD) = pd_mask;
#ifdef PSC_NO_WAIT
    return(PASS);
#endif
    while(((*ptstat & pd_mask) != 0) && (loop_cnt < PSC_PD_TIMEOUT))
    	{
    		loop_cnt++;
    	}
     loop_cnt = 0;
     
while(((*ptstat & 0x1F) != 0) && (loop_cnt < PSC_TIMEOUT))
    	{
    		loop_cnt++;
    	}
    
    return(((*pdstat & 0x1F) == 0x0)? PASS : FAIL); 
}
#endif

