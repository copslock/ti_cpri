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
//#include <constants.h>
//#include <val_util.h>

#include <stdio.h>

#ifdef _TMS320C6X
#include <c6x.h>
#endif

#include "serdes_cfg.h"
#include "dfetest.h"
#include <ti/csl/cslver.h>
#include <ti/csl/csl_serdes_dfe.h>
#include <ti/csl/csl_serdes_iqn.h>

#define UINT    unsigned int
//#define csisc2_serdes0_cfg_base (0x02324000)
//#define csisc2_serdes1_cfg_base  (0x02326000)
//
//#define NES_LB      1
//#define LB_MODE     0
//
//
//
//extern void vbusp_write32(UINT addr, UINT data);
//extern uint32_t vbusp_read32  (uint32_t address);

//#include <serdes_setup_153p6_6p144.h>
//#include <serdes_setup_7p3728.h>
//#include <serdes_setup_iqn_4p9152.h>
//#include <serdes_setup_iqn_9p8304.h>
//#include <serdes_setup_6p144.h>
//#include <serdes_setup_9p8304_JESD.h>
//#include <serdes_setup_iqn_6p144.h>

//#ifndef LB_MODE
//  #define LB_MODE NES_LB
//#endif

//void serdes_write_byte(unsigned int addr, unsigned int data)
//{
//    UINT write_value;
//    write_value  = *(volatile UINT *)(addr & 0xFFFFFFFC);
//    write_value &= ~(0x000000FF << ((addr & 0x3) * 8));
//    write_value |= (data << ((addr & 0x3) * 8));
//    *(volatile UINT *)(addr & 0xFFFFFFFC) = write_value;
//}
//
//void serdes_write_mask(uint32_t addr,uint32_t mask,uint32_t write_data)
//{
//	uint32_t read_data, data;
//
//	read_data = (*(uint32_t *) (addr));
//	data = (write_data & ~mask ) | (read_data & mask);
//	(*(uint32_t *) (addr)) = data;
//}

//void serdes_cfg(unsigned int serdes_setup)
//{
//    UINT data;
//    switch(serdes_setup)
//    {
//    case DFE_REFCLK122P88MHz_6P144Gbps :
//    case DFE_REFCLK122P88MHz_7P3728Gbps :
//    case DFE_REFCLK122P88MHz_9P8304Gbps :
//    case DFE_REFCLK153P60MHz_6P144Gbps :
//    case IQN_REFCLK122P88MHz_9P8304Gbps : // Full Rate
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000020, 0xF3C0F030 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Full Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000024, 0xF3C0F030 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000020, 0xF3C0F030 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Full Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000024, 0xF3C0F030 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//        break;
//
//    case IQN_REFCLK122P88MHz_4P9152Gbps :
//    case IQN_REFCLK122P88MHz_6P144Gbps :
//    case DFE_REFCLK122P88MHz_3P072Gbps :
//    case DFE_REFCLK122P88MHz_3P6864Gbps :
//    case DFE_REFCLK122P88MHz_4P9152Gbps :
//    case DFE_REFCLK153P60MHz_3P072Gbps  : // Half Rate
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000020, 0xF7C0F430 ); // LANE0CTL_STS - Enable TX, Half Rate, 20-bit, Enable RX, Half Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000024, 0xF7C0F430 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000020, 0xF3C0F430 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Half Rate, 20-bit, TX Idle
////        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000024, 0xF3C0F430 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////    	serdes_write_mask(csisc2_serdes0_cfg_base+0x1fe0, 0x0000000F, 0xF000F400 );//lane 0
////    	serdes_write_mask(csisc2_serdes0_cfg_base+0x1fe4, 0x0000000F, 0xF000F400 );//lane 1
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000020, 0xF7C0F430 ); // LANE0CTL_STS - Enable TX, Half Rate, 20-bit, Enable RX, Half Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000024, 0xF7C0F430 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000020, 0xF3C0F430 ); // LANE0CTL_STS - Enable TX, Full Rate, 20-bit, Enable RX, Half Rate, 20-bit, TX Idle
////        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000024, 0xF3C0F430 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////    	serdes_write_mask(csisc2_serdes1_cfg_base+0x1fe0, 0x0000000F, 0xF000F400 );//lane 0
////    	serdes_write_mask(csisc2_serdes1_cfg_base+0x1fe4, 0x0000000F, 0xF000F400 );//lane 1
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//        break;
//
//    case DFE_REFCLK122P88MHz_2P4576Gbps : // Quad Rate
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000020, 0xFBC0F830 ); // LANE0CTL_STS - Enable TX, Quad Rate, 20-bit, Enable RX, Quad Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000024, 0xFBC0F830 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000020, 0xF7C0F830 ); // LANE0CTL_STS - Enable TX, Half Rate, 20-bit, Enable RX, Quad Rate, 20-bit, TX Idle
////        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000024, 0xF7C0F830 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////    	serdes_write_mask(csisc2_serdes0_cfg_base+0x1fe0, 0x0000000F, 0xF400F800 );//lane 0 (RJ: ok for BB loopback)
////    	serdes_write_mask(csisc2_serdes0_cfg_base+0x1fe4, 0x0000000F, 0xF400F800 );//lane 1
//        vbusp_write32( csisc2_serdes0_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000020, 0xFBC0F830 ); // LANE0CTL_STS - Enable TX, Quad Rate, 20-bit, Enable RX, Quad Rate, 20-bit, TX Idle
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000024, 0xFBC0F830 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000020, 0xF7C0F830 ); // LANE0CTL_STS - Enable TX, Half Rate, 20-bit, Enable RX, Quad Rate, 20-bit, TX Idle
////        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000024, 0xF7C0F830 ); // LANE1CTL_STS - Enable TX, Enable RX, TX Idle
////    	serdes_write_mask(csisc2_serdes1_cfg_base+0x1fe0, 0x0000000F, 0xF400F800 );//lane 0 (RJ: ok for BB loopback)
////    	serdes_write_mask(csisc2_serdes1_cfg_base+0x1fe4, 0x0000000F, 0xF400F800 );//lane 1
//        vbusp_write32( csisc2_serdes1_cfg_base + 0x00001FC0 + 0x00000034, 0xE0000000 ); // PLL_CTRL - Enable PLL
//        break;
//
//    default     :
//          //end_test(FAIL);
//          break;
//    }
//
//       // Selecting the proper .reg file depends on the serdes data speed
//    switch(serdes_setup)
//    {
//    case  DFE_REFCLK122P88MHz_6P144Gbps :
//    case  DFE_REFCLK122P88MHz_3P072Gbps :
//        serdes_setup_6p144(0);
//        serdes_setup_6p144(1);
//        break;
//
//    case DFE_REFCLK122P88MHz_7P3728Gbps :
//    case DFE_REFCLK122P88MHz_3P6864Gbps :
//        serdes_setup_7p3728(0);
//        serdes_setup_7p3728(1);
//        break;
//
//    case DFE_REFCLK122P88MHz_9P8304Gbps :
//    case DFE_REFCLK122P88MHz_4P9152Gbps :
//    case DFE_REFCLK122P88MHz_2P4576Gbps :
//        serdes_setup_9p8304_JESD(0);
//        serdes_setup_9p8304_JESD(1);
//        break;
//
//    case DFE_REFCLK153P60MHz_6P144Gbps :
//    case DFE_REFCLK153P60MHz_3P072Gbps :
//        serdes_setup_153p6_6p144(0);
//        serdes_setup_153p6_6p144(1);
//        break;
//
//    case IQN_REFCLK122P88MHz_4P9152Gbps :
//        serdes_setup_iqn_4p9152(0);
//        serdes_setup_iqn_4p9152(1);
//        break;
//
//    case IQN_REFCLK122P88MHz_6P144Gbps :
//        serdes_setup_iqn_6p144(0);
//        serdes_setup_iqn_6p144(1);
//        break;
//
//    case IQN_REFCLK122P88MHz_9P8304Gbps :
//        serdes_setup_iqn_9p8304(0);
//        serdes_setup_iqn_9p8304(1);
//        break;
//
//    default     :
//        //end_test(FAIL);
//              break;
//    }
//
//
////Loopback mode
//
//    if ( LB_MODE == NES_LB )
//    {
//    // Enable SERDES0 NES loopback (analog loopback)
//        data= vbusp_read32( csisc2_serdes0_cfg_base+ 0x200);
//        data = data | 0x40000000;
//        vbusp_write32( csisc2_serdes0_cfg_base+ 0x200, data );
//
//        // Enable SERDES1 NES loopback (analog loopback)
//        data=vbusp_read32( csisc2_serdes1_cfg_base+ 0x200);
//        data = data | 0x40000000;
//        vbusp_write32( csisc2_serdes1_cfg_base+ 0x200, data );
//    }
//}

extern uint32_t serdes_cfg0_base;
extern uint32_t serdes_cfg1_base;

#if CSL_VERSION_ID <= (0x02010006)
void serdes_cfg_csl(unsigned int serdes_setup)
{
//    UINT data;
    CSL_SERDES_RESULT retval;
    CSL_SERDES_STATUS status;
    UINT lane_num;

    /* Step 1: Apply Serdes Initialization */
    switch(serdes_setup)
    { 

        case DFE_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_3P072Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_DFESerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_6p144G);

            /* Serdes 1 Init */
            retval = CSL_DFESerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_6p144G);
            break;

        case DFE_REFCLK122P88MHz_7P3728Gbps:
        case DFE_REFCLK122P88MHz_3P6864Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_DFESerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_7p3728G);

            /* Serdes 1 Init */
            retval = CSL_DFESerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_7p3728G);
            break;

        case DFE_REFCLK122P88MHz_9P8304Gbps:
        case DFE_REFCLK122P88MHz_4P9152Gbps:
        case DFE_REFCLK122P88MHz_2P4576Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_DFESerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);

            /* Serdes 1 Init */
            retval = CSL_DFESerdesInit ( serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);
            break;

        case DFE_REFCLK153P60MHz_6P144Gbps:
        case DFE_REFCLK153P60MHz_3P072Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_DFESerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_153p6M, CSL_SERDES_LINK_RATE_6p144G);

            /* Serdes 1 Init */
            retval = CSL_DFESerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_153p6M, CSL_SERDES_LINK_RATE_6p144G);
            break;

        case IQN_REFCLK122P88MHz_4P9152Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_4p9152G);

            /* Serdes 1 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_4p9152G);
            break;

        case IQN_REFCLK122P88MHz_6P144Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_6p144G);

            /* Serdes 1 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_6p144G);
            break;

        case IQN_REFCLK122P88MHz_9P8304Gbps:
            /*@TODO: check retval for errors */
            /* Serdes 0 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg0_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);

            /* Serdes 1 Init */
            retval = CSL_IQNSerdesInit (serdes_cfg1_base,
                    CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);
            break;

        default:
            return;

    } /* End Switch */

    /* Step 2: Enable Lanes */
    switch(serdes_setup)
    {
        case DFE_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_7P3728Gbps:
        case DFE_REFCLK122P88MHz_9P8304Gbps:
        case DFE_REFCLK153P60MHz_6P144Gbps:
        case IQN_REFCLK122P88MHz_9P8304Gbps: //Full Rate
            /* Lane Enable: Serdes 0*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg0_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_FULL_RATE);
            }

            /* Lane Enable: Serdes 1*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg1_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_FULL_RATE);
            }
            break;

        case IQN_REFCLK122P88MHz_4P9152Gbps:
        case IQN_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_3P072Gbps:
        case DFE_REFCLK122P88MHz_3P6864Gbps:
        case DFE_REFCLK122P88MHz_4P9152Gbps:
        case DFE_REFCLK153P60MHz_3P072Gbps: //Half Rate
            /* Lane Enable: Serdes 0*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg0_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_HALF_RATE);
            }

            /* Lane Enable: Serdes 1*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg1_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_HALF_RATE);
            }
            break;

        case DFE_REFCLK122P88MHz_2P4576Gbps: // Quarter Rate
            /* Lane Enable: Serdes 0*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg0_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_QUARTER_RATE);
            }

            /* Lane Enable: Serdes 1*/
            for(lane_num = 0; lane_num < 2; lane_num++)
            {
                CSL_DFESerdesLaneEnable (serdes_cfg1_base, lane_num,
                        CSL_SERDES_LOOPBACK_ENABLED, CSL_SERDES_LANE_QUARTER_RATE);
            }
            break;

        default:
            return ;


    } /* End Switch */

    /* Step 3: Enable PLL */
    CSL_DFESerdesPllEnable(serdes_cfg0_base);
    CSL_DFESerdesPllEnable(serdes_cfg1_base);

    /* PLL status check : Serdes 0 */
    status = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
    while(status != CSL_SERDES_STATUS_PLL_LOCKED)
    {
        status = CSL_DFESerdesGetStatus (serdes_cfg0_base, 2);
    }

    /* PLL status check : Serdes 1 */ 
    status = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
    while(status != CSL_SERDES_STATUS_PLL_LOCKED)
    {
        status = CSL_DFESerdesGetStatus (serdes_cfg1_base, 2);
    }

}

#else

void serdes_cfg_csl(unsigned int serdes_setup)
{
//    UINT data;
    CSL_SERDES_RESULT retval;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;
    uint32_t i;

#ifdef _TMS320C6X
    // start CPU timestamp as it is needed by the SerDes CSL to compute delays
    TSCL=0;
#endif

    memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
    memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

    serdes_lane_enable_params1.base_addr = serdes_cfg0_base;
    serdes_lane_enable_params1.num_lanes = 2;
    serdes_lane_enable_params1.phy_type = SERDES_DFE;
    serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
    serdes_lane_enable_params1.lane_mask = 0x3;
    serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

    serdes_lane_enable_params2.base_addr = serdes_cfg1_base;
    serdes_lane_enable_params2.num_lanes = 2;
    serdes_lane_enable_params2.phy_type = SERDES_DFE;
    serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
    serdes_lane_enable_params2.lane_mask = 0x3;
    serdes_lane_enable_params2.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

    for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
    {
    serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;

    /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
    serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
    serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

    /* For higher speeds PHY-A, force attenuation and boost values  */
    serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
    serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

    /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
    serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
    serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
    serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
    serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
    serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
    }

    for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
    {
    serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_ENABLED;

    /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
    serdes_lane_enable_params2.rx_coeff.att_start[i] = 7;
    serdes_lane_enable_params2.rx_coeff.boost_start[i] = 5;

    /* For higher speeds PHY-A, force attenuation and boost values  */
    serdes_lane_enable_params2.rx_coeff.force_att_val[i] = 1;
    serdes_lane_enable_params2.rx_coeff.force_boost_val[i] = 1;

    /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
    serdes_lane_enable_params2.tx_coeff.cm_coeff[i] = 0;
    serdes_lane_enable_params2.tx_coeff.c1_coeff[i] = 0;
    serdes_lane_enable_params2.tx_coeff.c2_coeff[i] = 0;
    serdes_lane_enable_params2.tx_coeff.tx_att[i] = 12;
    serdes_lane_enable_params2.tx_coeff.tx_vreg[i] = 4;
    }

    switch(serdes_setup)
    {
        case DFE_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_3P072Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            retval = CSL_DFESerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_DFESerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case DFE_REFCLK122P88MHz_7P3728Gbps:
        case DFE_REFCLK122P88MHz_3P6864Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_7p3728G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_7p3728G;
            retval = CSL_DFESerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_DFESerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case DFE_REFCLK122P88MHz_9P8304Gbps:
        case DFE_REFCLK122P88MHz_4P9152Gbps:
        case DFE_REFCLK122P88MHz_2P4576Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_9p8304G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_9p8304G;
            retval = CSL_DFESerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_DFESerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case DFE_REFCLK153P60MHz_6P144Gbps:
        case DFE_REFCLK153P60MHz_3P072Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_153p6M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_153p6M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            retval = CSL_DFESerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_DFESerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case IQN_REFCLK122P88MHz_4P9152Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_4p9152G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_4p9152G;
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case IQN_REFCLK122P88MHz_6P144Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_6p144G;
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        case IQN_REFCLK122P88MHz_9P8304Gbps:
            serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_9p8304G;
            serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_122p88M;
            serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_9p8304G;
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params1.base_addr,
                    serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);
            retval = CSL_IQNSerdesInit (serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);
            if (retval != 0)
            {
                printf ("Invalid Serdes Init Params\n");
            }
            break;

        default:
            return;

    } /* End Switch */

    /* Step 2: Enable lanes */
    switch(serdes_setup)
    {
        case DFE_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_7P3728Gbps:
        case DFE_REFCLK122P88MHz_9P8304Gbps:
        case DFE_REFCLK153P60MHz_6P144Gbps:
        case IQN_REFCLK122P88MHz_9P8304Gbps: //Full Rate
        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
            {
                serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE;
            }
        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
            {
                serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE;
            }
            break;

        case IQN_REFCLK122P88MHz_4P9152Gbps:
        case IQN_REFCLK122P88MHz_6P144Gbps:
        case DFE_REFCLK122P88MHz_3P072Gbps:
        case DFE_REFCLK122P88MHz_3P6864Gbps:
        case DFE_REFCLK122P88MHz_4P9152Gbps:
        case DFE_REFCLK153P60MHz_3P072Gbps: //Half Rate
        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
            {
                serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_HALF_RATE;
            }
        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
            {
                serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_HALF_RATE;
            }
            break;

       case DFE_REFCLK122P88MHz_2P4576Gbps: // Quarter Rate
        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
            {
                serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            }
        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
            {
                serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            }
            break;

        default:
            return ;


    } /* End Switch */

    /* Common Init Mode */
    /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
    /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
    serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    serdes_lane_enable_params1.lane_mask = 0x3;
    lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

    serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    serdes_lane_enable_params2.lane_mask = 0x3;
    lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);

    /* Lane Init Mode */
    /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
       iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
    /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
    serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
    {
        serdes_lane_enable_params1.lane_mask = 1<<i;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
        if (lane_retval != 0)
        {
            printf ("Invalid Serdes Lane Enable Init\n");
        }
    }

    serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
    {
        serdes_lane_enable_params2.lane_mask = 1<<i;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
        if (lane_retval != 0)
        {
            printf ("Invalid Serdes Lane Enable Init\n");
        }
    }

}

#endif

//////////////////////////////////////
