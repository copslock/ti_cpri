/*
 * tps65023.h
*/

/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
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
/*
 * TI Booting and Flashing Utilities
 *
 * Provides device differentiation for the project files. This file MUST be
 * modified to match the device specifics.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* --------------------------------------------------------------------------
  AUTHOR      : Platform Support Group
 --------------------------------------------------------------------------- */
//I2C functions
#include "i2c.h"

/******************************************************
* Global Typedef declarations                         *
******************************************************/
 
//Structure to hold the TPS 65023 related details
typedef struct tps65023_pmic {
    I2C_ConfigObj i2cConfig;
    I2C_InfoHandle       hI2cInfo;
}TPS_ConfigObj;

#define TPS_MAX_NUM_DEVICES     1u
#define TPS_REG_OFFSET_VERSION  0x00u
#define TPS_REG_OFFSET_PGOODZ   0x01u
#define TPS_REG_OFFSET_MASK     0x02u
#define TPS_REG_OFFSET_REGCTRL  0x03u
#define TPS_REG_OFFSET_CONCTRL  0x04u
#define TPS_REG_OFFSET_CONCTRL2 0x05u
#define TPS_REG_OFFSET_DEFCORE  0x06u
#define TPS_REG_OFFSET_DEFSLEW  0x07u
#define TPS_REG_OFFSET_LDOCTRL  0x08u

/* Valid voltage ranges for DEFCORE VDCDC1 */
#define TPS_VDCDC1_MILLIVOLT_800    0x00u
#define TPS_VDCDC1_MILLIVOLT_825    0x01u
#define TPS_VDCDC1_MILLIVOLT_850    0x02u
#define TPS_VDCDC1_MILLIVOLT_875    0x03u
#define TPS_VDCDC1_MILLIVOLT_900    0x04u
#define TPS_VDCDC1_MILLIVOLT_925    0x05u
#define TPS_VDCDC1_MILLIVOLT_950    0x06u
#define TPS_VDCDC1_MILLIVOLT_975    0x07u
#define TPS_VDCDC1_MILLIVOLT_1000   0x08u
#define TPS_VDCDC1_MILLIVOLT_1025   0x09u
#define TPS_VDCDC1_MILLIVOLT_1050   0x0Au
#define TPS_VDCDC1_MILLIVOLT_1075   0x0Bu
#define TPS_VDCDC1_MILLIVOLT_1100   0x0Cu
#define TPS_VDCDC1_MILLIVOLT_1125   0x0Du
#define TPS_VDCDC1_MILLIVOLT_1150   0x0Eu
#define TPS_VDCDC1_MILLIVOLT_1175   0x0Fu
#define TPS_VDCDC1_MILLIVOLT_1200   0x10u
#define TPS_VDCDC1_MILLIVOLT_1225   0x11u
#define TPS_VDCDC1_MILLIVOLT_1250   0x12u
#define TPS_VDCDC1_MILLIVOLT_1275   0x13u
#define TPS_VDCDC1_MILLIVOLT_1300   0x14u
#define TPS_VDCDC1_MILLIVOLT_1325   0x15u
#define TPS_VDCDC1_MILLIVOLT_1350   0x16u
#define TPS_VDCDC1_MILLIVOLT_1375   0x17u
#define TPS_VDCDC1_MILLIVOLT_1400   0x18u
#define TPS_VDCDC1_MILLIVOLT_1425   0x19u
#define TPS_VDCDC1_MILLIVOLT_1450   0x1Au
#define TPS_VDCDC1_MILLIVOLT_1475   0x1Bu
#define TPS_VDCDC1_MILLIVOLT_1500   0x1Cu
#define TPS_VDCDC1_MILLIVOLT_1525   0x1Du
#define TPS_VDCDC1_MILLIVOLT_1550   0x1Eu
#define TPS_VDCDC1_MILLIVOLT_1600   0x1Fu




/***********************************************************
* Global Function Declarations                             *
***********************************************************/
Uint32 		   TPS65023_open(Uint32 instance);
Uint8          TPS65023_reg_read(Uint32 instance, Uint8 regOffset, Uint8 *buf );
Uint32         TPS65023_reg_write(Uint32 instance, Uint8 regOffset, Uint8 regVal);
Uint32 		   TPS65023_set_DCDC1_voltage(Uint32 instance, Uint16 volt);
