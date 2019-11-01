/*
 *
 * Copyright (C) 2010-2012 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *   @file  srio_laneconfig.c
 *
 *   @brief
 *      Utilities to setup the PLL, Lane rate and port configuration for SRIO.
 *
 */

/* Standard library and XDC Include Files. */
#include <stdio.h>
#include <stdlib.h>
#include <xdc/runtime/System.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* CSL Modules */
#include <ti/csl/csl_bootcfg.h>
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_tsc.h>
/* Application Include Files */
#include <srio_laneconfig.h>
#include <ti/csl/csl_serdes_srio.h>

#define MAX_MSG_LEN 128

#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)
/**********************************************************************
 *********************** DEVICE SRIO FUNCTIONS ***********************
 **********************************************************************/
 
/**
 *  @b Description
 *  @n
 *      The function is used to set the SRIO lane configuration.
 *      Prerequisite: Run setEnableSrioPllRxTx() first or it will overwrite the MSYNC setting done
 *                    by this function.
 *
 *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *  @param[in]  laneMode
 *      Mode number to set the lanes to a specific configuration.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0 (Invalid lane configuration mode specified)
 */
void serdes_setSrioLanes (CSL_SrioHandle hSrio, srioLanesMode_e laneMode)
{
    /* Stub Function. TBD for SERDES configuration */

}
  
int32_t srioDefSerdesSetup(CSL_SrioHandle hSrio, srioRefClockMhz_e refClockMhz, srioLaneRateGbps_e linkRateGbps, int isLoopbackMode)
{
    // Write 0 to Boot complete bit
    CSL_SRIO_SetBootComplete(hSrio,0);

	  uint32_t i;
	  CSL_SERDES_REF_CLOCK serdes_ref_clk;
	  CSL_SERDES_LINK_RATE serdes_link_rate;
	  CSL_SERDES_LANE_CTRL_RATE serdes_lane_ctrl_rate;
	  CSL_SERDES_LOOPBACK serdes_loopback;
	  CSL_SERDES_RESULT status;
	  CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
	  CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;

	  if (refClockMhz == srio_ref_clock_156p25Mhz)
		  serdes_ref_clk = CSL_SERDES_REF_CLOCK_156p25M;
	  else if (refClockMhz == srio_ref_clock_125p00Mhz)
		  serdes_ref_clk = CSL_SERDES_REF_CLOCK_125M;

	  if (linkRateGbps == srio_lane_rate_5p000Gbps)
	  {
		  serdes_link_rate = CSL_SERDES_LINK_RATE_5G;
		  serdes_lane_ctrl_rate = CSL_SERDES_LANE_FULL_RATE;
	  }		  
	  if (linkRateGbps == srio_lane_rate_3p125Gbps)
	  {
		  serdes_link_rate = CSL_SERDES_LINK_RATE_6p25G;
		  serdes_lane_ctrl_rate = CSL_SERDES_LANE_HALF_RATE;
	  }
	  if (linkRateGbps == srio_lane_rate_2p500Gbps)
	  {
		  serdes_link_rate = CSL_SERDES_LINK_RATE_5G;
		  serdes_lane_ctrl_rate = CSL_SERDES_LANE_HALF_RATE;
	  }
	  if (linkRateGbps == srio_lane_rate_1p250Gbps)
	  {
		  serdes_link_rate = CSL_SERDES_LINK_RATE_5G;
		  serdes_lane_ctrl_rate = CSL_SERDES_LANE_QUARTER_RATE;
	  }
	  
	  if (isLoopbackMode)
	  serdes_loopback = CSL_SERDES_LOOPBACK_ENABLED;
	  else
	  serdes_loopback = CSL_SERDES_LOOPBACK_DISABLED;	  	  		  

	    memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

	    serdes_lane_enable_params.base_addr = CSL_SRIO_SERDES_CFG_REGS;
	    serdes_lane_enable_params.ref_clock = serdes_ref_clk;
	    serdes_lane_enable_params.linkrate = serdes_link_rate;
	    serdes_lane_enable_params.num_lanes = 4;
	    serdes_lane_enable_params.phy_type = SERDES_SRIO;

	    /* When this flag is enabled, the RX auto adaptation is disabled */
	    /* Att and Boost values are obtained through Serdes Diagnostic PRBS calibration test */
	    /* For higher speeds PHY-A >= 5G or when the remote transmitter (PRODUCER) is not up */
	    /* at the same time as the receiver (CONSUMER), force the attenuation and boost values */
	    /* with the values obtained from Serdes Diagnostic PRBS calibration test  */
	    serdes_lane_enable_params.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_ENABLED;

	    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
	    {
	        serdes_lane_enable_params.loopback_mode[i] = serdes_loopback;
	        serdes_lane_enable_params.lane_ctrl_rate[i] = serdes_lane_ctrl_rate;

            /* When RX auto adaptation is on, these are the starting values used */
            /* for att, boost adaptation */
	        serdes_lane_enable_params.rx_coeff.att_start[i] = 7;
	        serdes_lane_enable_params.rx_coeff.boost_start[i] = 5;

	        /* For higher speeds PHY-A, force attenuation and boost values. */
	        /* See forceattboost description above for more details  */
	        serdes_lane_enable_params.rx_coeff.force_att_val[i] = 1;
	        serdes_lane_enable_params.rx_coeff.force_boost_val[i] = 6;

	        /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
	        serdes_lane_enable_params.tx_coeff.cm_coeff[i] = 0;
	        serdes_lane_enable_params.tx_coeff.c1_coeff[i] = 0;
	        serdes_lane_enable_params.tx_coeff.c2_coeff[i] = 0;
	        serdes_lane_enable_params.tx_coeff.tx_att[i] = 12;
	        serdes_lane_enable_params.tx_coeff.tx_vreg[i] = 4;
	    }
	    serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;

	    status = CSL_SrioSerdesInit(serdes_lane_enable_params.base_addr,
	                                    serdes_lane_enable_params.ref_clock,
	                                    serdes_lane_enable_params.linkrate);

	    if (status != 0)
	    {
	    	System_printf ("Invalid Serdes Init Params\n");
	    }

	    /* Common Init Mode */
        /* Iteration Mode needs to be set to Common Init Mode first with a
         * lane_mask value equal to the total number of lanes being configured.
         * For example, if there are a total of 2 lanes being configured, lane
         * mask needs to be set to 0x3 */
	    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
	    serdes_lane_enable_params.lane_mask = 0xF;
	    lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
	    if (lane_retval != 0)
	    {
	    	System_printf ("Invalid Serdes Common Init\n");
	        exit(0);
	    }
	    printf("SRIO Serdes Common Init Complete\n");

	    /* Lane Init Mode */
        /* Once CSL_SerdesLaneEnable is called with
         * iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to
         * be initialized by setting
         * iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask
         * equal to the specific lane being configured.  For example, if lane 0 is
         * being configured, lane mask needs to be set to 0x1. if lane 1 is being
         * configured, lane mask needs to be 0x2 etc */
	    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
	    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
	    {
	        serdes_lane_enable_params.lane_mask = 1<<i;
	        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
	        if (lane_retval != 0)
	        {
	        	System_printf ("Invalid Serdes Lane Enable Init\n");
	            exit(0);
	        }
	        System_printf("SRIO Serdes Lane %d Init Complete\n", i);
	    }

	    System_printf("SRIO Serdes Init Complete\n");

	  return 0;
}
#endif

/**
 *  @b Description
 *  @n
 *      The function is used to get the status of the SRIO lane configuration.
 *      Allows user to confirm setting of the lane configuration against the
 *      actual status shown for the lane configuration.
 *
 *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0 (Invalid status)
 */
int32_t displaySrioLanesStatus (CSL_SrioHandle hSrio)
{
	SRIO_LANE_STATUS	ptrLaneStatus;
	Int8				i;
	uint32_t			laneCode=0;
	int32_t				returnStatus=0;
	char				laneSetupText[MAX_MSG_LEN];

   	for (i = 3; i >= 0; i--)
   	{
   		CSL_SRIO_GetLaneStatus (hSrio,i,&ptrLaneStatus);
   		laneCode = laneCode | (ptrLaneStatus.laneNum << (i * 4));
   	}

	switch (laneCode)
	{
		case 0x0000:	/* four 1x ports */
			sprintf (laneSetupText,"four 1x ports");
			break;
		case 0x0010:	/* one 2x port and two 1x ports */
			sprintf (laneSetupText,"one 2x port and two 1x ports");
			break;
		case 0x1000:	/* two 1x ports and one 2x port */
			sprintf (laneSetupText,"two 1x ports and one 2x port");
			break;
		case 0x1010:	/* two 2x ports */
			sprintf (laneSetupText,"two 2x ports");
			break;
		case 0x3210:	/* one 4x port */
			sprintf (laneSetupText,"one 4x port");
			break;
		default: /* four 1x ports */
			sprintf (laneSetupText,"INVALID LANE COMBINATION FOUND");
			returnStatus = -1;
			break;
	}
	System_printf ("Debug:   Lanes status shows lanes formed as %s%s", laneSetupText, "\n");

	return returnStatus;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the SRIO lane configuration.
 *      Prerequisite: Run setEnableSrioPllRxTx() first or it will overwrite the MSYNC setting done
 *                    by this function.
 *
 *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *  @param[in]  laneMode
 *      Mode number to set the lanes to a specific configuration.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0 (Invalid lane configuration mode specified)
 */
int32_t setSrioLanes (CSL_SrioHandle hSrio, srioLanesMode_e laneMode)
{
	Uint8	port, pathMode;
	Uint8	bootCompleteFlag;
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(SOC_K2K) && !defined(SOC_K2H) 
	Uint32	serdesTXConfig;
	Uint32	msyncSet = 0x00100000;
#endif

	CSL_SRIO_GetBootComplete (hSrio, &bootCompleteFlag);

    if (bootCompleteFlag == 1)
    	/* Set boot complete to be 0; to enable writing to the SRIO registers. */
		CSL_SRIO_SetBootComplete (hSrio, 0);

    /* Set the path mode number to the lane configuration specified */
	switch (laneMode)
	{
		case srio_lanes_form_four_1x_ports:					/* four 1x ports (forms ports: 0,1,2 and 3) */
			pathMode = 0;
			break;
		case srio_lanes_form_one_2x_port_and_two_1x_ports:	/* one 2x port and two 1x ports (forms ports: 0, 2 and 3) */
			pathMode = 1;
			break;
		case srio_lanes_form_two_1x_ports_and_one_2x_port:	/* two 1x ports and one 2x port (forms ports: 0, 1 and 2) */
			pathMode = 2;
			break;
		case srio_lanes_form_two_2x_ports:					/* two 2x ports (forms ports: 0 and 2) */
			pathMode = 3;
			break;
		case srio_lanes_form_one_4x_port:					/* one 4x port (forms port: 0) */
			pathMode = 4;
			break;
		default:	/* Invalid lane configuration mode specified */
		    return -1;
	}

    /* Configure the path mode for all ports. */
    for (port = 0; port < 4; port++)
    {
        /* Configure the path mode for the port. */
        CSL_SRIO_SetPLMPortPathControlMode (hSrio, port, pathMode);
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(SOC_K2K) && !defined(SOC_K2H) 
        /* Get the current SERDES TX config */
        CSL_BootCfgGetSRIOSERDESTxConfig (port,&serdesTXConfig);

        /* Determine the MSYNC bit's setting according to laneMode being used */
    	switch (laneMode)
    	{
    		case srio_lanes_form_four_1x_ports:						/* four 1x ports (forms ports: 0,1,2 and 3) */
    			msyncSet = 0x00100000;
    			break;
    		case srio_lanes_form_one_2x_port_and_two_1x_ports:		/* one 2x port and two 1x ports (forms ports: 0, 2 and 3) */
    			msyncSet = (port != 1 ? 0x00100000 : 0xFFEFFFFF);
    			break;
    		case srio_lanes_form_two_1x_ports_and_one_2x_port:		/* two 1x ports and one 2x port (forms ports: 0, 1 and 2) */
    			msyncSet = (port != 3 ? 0x00100000 : 0xFFEFFFFF);
    			break;
    		case srio_lanes_form_two_2x_ports:						/* two 2x ports (forms ports: 0 and 2) */
    			msyncSet = (((port != 1) && (port != 3)) ? 0x00100000 : 0xFFEFFFFF);
    			break;
    		case srio_lanes_form_one_4x_port:						/* one 4x port (forms port: 0) */
    			msyncSet = (port == 0 ? 0x00100000 : 0xFFEFFFFF);
    			break;
    		default:	/* Invalid lane configuration mode specified */
    		    return -1;
    	}

    	/* Set msync for each port according to the lane mode (port width) specified */
    	if (msyncSet == 0x00100000)
        	serdesTXConfig |= msyncSet;				/* Set MSYNC bit */
        else
        	serdesTXConfig &= msyncSet;				/* Clear MSYNC bit */

    	/* Write SERDES TX MSYNC bit */
		CSL_BootCfgSetSRIOSERDESTxConfig (port, serdesTXConfig);
#endif
    }
    
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H) 
    serdes_setSrioLanes(hSrio,laneMode);
#endif

    if (bootCompleteFlag == 1)
		/* Set boot complete back to 1; configuration is complete. */
		CSL_SRIO_SetBootComplete (hSrio, 1);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get PLL value needed to set the proper multiplier
 *      for the SERDES PLL to give the proper SRIO lane link rate.
 *
 *  @param[in]  refClockMhz
 *      Frequency in Megahertz of the input clock to the SERDES PLL (enum)
 *  @param[in]  linkRateGbps
 *      Desired link rate in gigabits per second (enum)
 *
 *  @retval
 *      SRIO SERDES Configuration PLL value
 */
int32_t getPllValue (srioRefClockMhz_e refClockMhz, srioLaneRateGbps_e linkRateGbps)
{
	int32_t pllValue = 0;

	/* Get PLL value based on reference clock and lane rate */
	switch (refClockMhz)
	{
		case srio_ref_clock_125p00Mhz:
			switch (linkRateGbps)
			{
				case srio_lane_rate_3p125Gbps:
					pllValue = 0x265; //10 0110 010 1 (12.5x and set enable bit)
					break;
				default:
					pllValue = 0x2A1; //10 1010 000 1 (20x and set enable bit)
					break;
			}
			break;
		case srio_ref_clock_156p25Mhz:
			switch (linkRateGbps)
			{
				case srio_lane_rate_3p125Gbps:
					pllValue = 0x251; //10 0101 000 1 (10x and set enable bit)
					break;
				default:
					pllValue = 0x281; //10 1000 000 1 (16x and set enable bit)
					break;
			}
			break;
		case srio_ref_clock_250p00Mhz:
			switch(linkRateGbps)
			{
				case srio_lane_rate_3p125Gbps:
					pllValue = 0x233; //10 0011 001 1 (6.25x and set enable bit)
					break;
				default:
					pllValue = 0x251; //10 0101 000 1 (10x and set enable bit)
					break;
			}
			break;
		case srio_ref_clock_312p50Mhz:
			switch (linkRateGbps)
			{
				case srio_lane_rate_3p125Gbps:
					pllValue = 0x229; //10 0010 100 1 (5x and set enable bit)
					break;
				default:
					pllValue = 0x241; //10 0100 000 1 (8x and set enable bit)
					break;
			}
			break;
	}

	return pllValue;
}

/**
 *  @b Description
 *  @n
 *      The function is used to set the SRIO SERDES PLL value and the RX and TX
 *      SERDES configuration values for all lanes. Plus the ability to set loopback
 *      mode if needed.
 *
 *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *  @param[in]  refClockMhz
 *      Frequency in Megahertz of the input clock to the SERDES PLL (enum)
 *  @param[in]  linkRateGbps
 *      Desired link rate in gigabits per second (enum)
 *  @param[in]  loopbackMode
 *      Loopback setting for SRIO lane (enum)
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0 (Invalid pllValue or invalid linkRateGbps)
 */
int32_t setEnableSrioPllRxTx (CSL_SrioHandle hSrio, srioRefClockMhz_e refClockMhz, srioLaneRateGbps_e linkRateGbps, int isLoopbackMode)
{
	uint32_t pllValue = getPllValue (refClockMhz, linkRateGbps);
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(SOC_K2K) && !defined(SOC_K2H) 
	uint32_t rxConfig = 0;
	uint32_t txConfig = 0;
	Uint8	bootCompleteFlag;
#endif
	/* Return immediately if pllValue has not been identified and is still zero */
	if (pllValue == 0)
		return -1;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)
    srioDefSerdesSetup(hSrio,refClockMhz,linkRateGbps,isLoopbackMode);
#endif
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(SOC_K2K) && !defined(SOC_K2H) 
	/* Get Pll setting for single speed of 3.125Gbps or setting for all other speeds of 5.0Gbps, 2.5Gbps, and 1.25Gpbs */
    CSL_BootCfgSetSRIOSERDESConfigPLL (pllValue);

    /* Loop around until the SERDES PLL is locked. */
    while (1)
    {
        uint32_t    status;

        /* Get the SRIO SERDES Status */
        CSL_BootCfgGetSRIOSERDESStatus (&status);
        if (status & 0x1)
            break;
    }

	/* Get boot complete flag setting */
	CSL_SRIO_GetBootComplete (hSrio, &bootCompleteFlag);

    if (bootCompleteFlag == 1)
    	/* Set boot complete to be 0; to enable writing to the SRIO registers. */
		CSL_SRIO_SetBootComplete (hSrio, 0);

    /* Set rx/tx config values based on the lane rate specified */
	switch (linkRateGbps)
	{
		case srio_lane_rate_5p000Gbps:	/* Pll setting determines 5.0 Gpbs or 3.125 Gbps */
		case srio_lane_rate_3p125Gbps:	/* Same Tx and Rx settings for 5.0 Gbps or 3.125 Gbps */
			rxConfig = 0x00440495;
						// (0)     Enable Receiver
						// (1-3)   Bus Width 010b (20 bit)
						// (4-5)   Half rate. Two data samples per PLL output clock cycle
						// (6)     Normal polarity
						// (7-9)   Termination programmed to be 001
						// (10-11) Comma Alignment enabled
						// (12-14) Loss of signal detection disabled
						// (15-17) First order. Phase offset tracking up to +-488 ppm
						// (18-20) Fully adaptive equalization
						// (22)    Offset compensation enabled
						// (23-24) Loopback disabled
						// (25-27) Test pattern mode disabled
						// (28-31) Reserved

			txConfig = 0x00180795;
			//txConfig = 0x00181F15;
						// (0)     Enable Transmitter
						// (1-3)   Bus Width 010b (20 bit)
						// (4-5)   Half rate. Two data samples per PLL output clock cycle
						// (6)     Normal polarity
						// (7-10)  Swing max.
						// (11-13) Precursor Tap weight 0%
						// (14-18) Adjacent post cursor Tap weight 0%
						// (19)    Transmitter pre and post cursor FIR filter update
						// (20)    Synchronization master
						// (21-22) Loopback disabled
						// (23-25) Test pattern mode disabled
						// (26-31) Reserved
			break;
		case srio_lane_rate_2p500Gbps:	/* Tx and Rx settings for 2.50 Gbps */
			rxConfig = 0x004404A5;
						// (4-5)   Quarter rate. One data sample per PLL output clock cycle

			txConfig = 0x001807A5;
						// (4-5)   Quarter rate. One data sample per PLL output clock cycle
			break;
		case srio_lane_rate_1p250Gbps:	/* Tx and Rx settings for 1.25 Gbps */
			rxConfig = 0x004404B5;
						// (4-5)   Eighth rate. One data sample every two PLL output clock cycles

			txConfig = 0x001807B5;
						// (4-5)   Eighth rate. One data sample every two PLL output clock cycles

			break;
		default:	/* Invalid SRIO lane rate specified */
			return -1;
	}

	/* Return with error if rx/tx configuration values have not been determined and are still zero */
	if ((rxConfig == 0) || (txConfig == 0))
		return -1;

	/* If loop-back mode then set the appropriate bits */
	if (isLoopbackMode)
	{
		/* set RX and TX loop-back bits */
		rxConfig |= 0x01800000;
		txConfig |= 0x00600000;
	}

	/* Configure the SRIO SERDES Receive Configuration. */
	CSL_BootCfgSetSRIOSERDESRxConfig (0, rxConfig);
	CSL_BootCfgSetSRIOSERDESRxConfig (1, rxConfig);
	CSL_BootCfgSetSRIOSERDESRxConfig (2, rxConfig);
	CSL_BootCfgSetSRIOSERDESRxConfig (3, rxConfig);

	/* Configure the SRIO SERDES Transmit Configuration. */
	CSL_BootCfgSetSRIOSERDESTxConfig (0, txConfig);
	CSL_BootCfgSetSRIOSERDESTxConfig (1, txConfig);
	CSL_BootCfgSetSRIOSERDESTxConfig (2, txConfig);
	CSL_BootCfgSetSRIOSERDESTxConfig (3, txConfig);

    if (bootCompleteFlag == 1)
		/* Set boot complete back to 1; configuration is complete. */
		CSL_SRIO_SetBootComplete (hSrio, 0);
#endif
	return 0;
}

/**
 *  @b Description
 *  @n
 *      Wait for configured SRIO ports to show operational status.
 *
  *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *  @param[in]  laneMode
 *      Mode number to determine which ports to check for operational status.
 *
 *  @retval
 *      Success - 0
 */
int32_t waitAllSrioPortsOperational (CSL_SrioHandle hSrio, srioLanesMode_e laneMode)
{
	Uint8		port;
	Uint8		portsMask=0xF;		// 0b1111
	char		statusText[4]="";
    uint64_t	tscTemp;

    /* Set port mask to use based on the lane mode specified */
	switch (laneMode)
	{
		case srio_lanes_form_four_1x_ports:					/* check ports 0 to 3 */
			portsMask = 0xF;	// 0b1111
			break;
		case srio_lanes_form_one_2x_port_and_two_1x_ports:	/* check ports 0, 2, and 3 */
			portsMask = 0xD;	// 0b1101
			break;
		case srio_lanes_form_two_1x_ports_and_one_2x_port:	/* check ports 0, 1, and 2 */
			portsMask = 0x7;	// 0b0111
			break;
		case srio_lanes_form_two_2x_ports:					/* check ports 0 and 2 */
			portsMask = 0x5;	// 0b0101
			break;
		case srio_lanes_form_one_4x_port:					/* check port 0 */
			portsMask = 0x1;	// 0b0001
			break;
		default: /* check ports 0 to 3 */
			portsMask = 0xF;	// 0b1111
			break;
	}

    /* Wait for all SRIO ports for specified lane mode to be operational */
	System_printf ("Debug: Waiting for SRIO ports to be operational...  \n");
	tscTemp = CSL_tscRead () + 5000000000;
    for (port = 0; port < 4; port++)
    {
    	if ((portsMask >> port) & 0x1 == 1)
    	{
    		sprintf (statusText, "NOT ");
    		/* Check for good port status on each valid port, timeout if not received after 5 seconds */
    	    while (CSL_tscRead() < tscTemp)
    	    {
				if (CSL_SRIO_IsPortOk (hSrio, port) == TRUE)
				{
					sprintf (statusText,"");
					break;
				}
    	    }
    	    System_printf ("Debug: SRIO port %d is %soperational.\n", port, statusText);
    		tscTemp = CSL_tscRead() + 1000000000;
    	}
    }

    return 0;
}
