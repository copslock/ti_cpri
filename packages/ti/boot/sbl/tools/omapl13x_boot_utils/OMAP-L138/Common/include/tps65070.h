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
typedef struct tps65070_pmic {
    I2C_ConfigObj i2cConfig;
    I2C_InfoHandle       hI2cInfo;
}TPS_ConfigObj;

#define TPS_MAX_NUM_DEVICES             1u
#define TPS_REG_OFFSET_PPATH1           0x01u
#define TPS_REG_OFFSET_INT              0x02u
#define TPS_REG_OFFSET_CHGCONFIG0       0x03u
#define TPS_REG_OFFSET_CHGCONFIG1       0x04u
#define TPS_REG_OFFSET_CHGCONFIG2       0x05u
#define TPS_REG_OFFSET_CHGCONFIG3       0x06u
#define TPS_REG_OFFSET_ADCCONFIG        0x07u
#define TPS_REG_OFFSET_TSCMODE          0x08u
#define TPS_REG_OFFSET_ADRESULT_1       0x09u
#define TPS_REG_OFFSET_ADRESULT_2       0x0Au
#define TPS_REG_OFFSET_PGOOD            0x0Bu
#define TPS_REG_OFFSET_PGOODMASK        0x0Cu
#define TPS_REG_OFFSET_CONCTRL1         0x0Du
#define TPS_REG_OFFSET_CONCTRL2         0x0Eu
#define TPS_REG_OFFSET_CONCTRL3         0x0Fu
#define TPS_REG_OFFSET_DEFDCDC1         0x10u
#define TPS_REG_OFFSET_DEFDCDC2_LOW     0x11u
#define TPS_REG_OFFSET_DEFDCDC2_HIGH    0x12u
#define TPS_REG_OFFSET_DEFDCDC3_LOW     0x13u
#define TPS_REG_OFFSET_DEFDCDC3_HIGH    0x14u
#define TPS_REG_OFFSET_DEFSLEW          0x15u
#define TPS_REG_OFFSET_LDOCTRL1         0x16u
#define TPS_REG_OFFSET_DEFLDO2          0x17u
#define TPS_REG_OFFSET_WLED_CTRL1       0x18u
#define TPS_REG_OFFSET_WLED_CTRL2       0x19u




/* Valid voltage ranges for DEFCORE VDCDC1 */
#define TPS_VDCDC3_MILLIVOLT_725    0x00u
#define TPS_VDCDC3_MILLIVOLT_750    0x01u
#define TPS_VDCDC3_MILLIVOLT_775    0x02u
#define TPS_VDCDC3_MILLIVOLT_800    0x03u
#define TPS_VDCDC3_MILLIVOLT_825    0x04u
#define TPS_VDCDC3_MILLIVOLT_850    0x05u
#define TPS_VDCDC3_MILLIVOLT_875    0x06u
#define TPS_VDCDC3_MILLIVOLT_900    0x07u
#define TPS_VDCDC3_MILLIVOLT_925    0x08u
#define TPS_VDCDC3_MILLIVOLT_950    0x09u
#define TPS_VDCDC3_MILLIVOLT_975    0x0Au
#define TPS_VDCDC3_MILLIVOLT_1000   0x0Bu
#define TPS_VDCDC3_MILLIVOLT_1025   0x0Cu
#define TPS_VDCDC3_MILLIVOLT_1050   0x0Du
#define TPS_VDCDC3_MILLIVOLT_1075   0x0Eu
#define TPS_VDCDC3_MILLIVOLT_1100   0x0Fu
#define TPS_VDCDC3_MILLIVOLT_1125   0x10u
#define TPS_VDCDC3_MILLIVOLT_1150   0x11u
#define TPS_VDCDC3_MILLIVOLT_1175   0x12u
#define TPS_VDCDC3_MILLIVOLT_1200   0x13u
#define TPS_VDCDC3_MILLIVOLT_1225   0x14u
#define TPS_VDCDC3_MILLIVOLT_1250   0x15u
#define TPS_VDCDC3_MILLIVOLT_1275   0x16u
#define TPS_VDCDC3_MILLIVOLT_1300   0x17u
#define TPS_VDCDC3_MILLIVOLT_1325   0x18u
#define TPS_VDCDC3_MILLIVOLT_1350   0x19u
#define TPS_VDCDC3_MILLIVOLT_1375   0x1Au
#define TPS_VDCDC3_MILLIVOLT_1400   0x1Bu
#define TPS_VDCDC3_MILLIVOLT_1425   0x1Cu
#define TPS_VDCDC3_MILLIVOLT_1450   0x1Du
#define TPS_VDCDC3_MILLIVOLT_1475   0x1Eu
#define TPS_VDCDC3_MILLIVOLT_1500   0x1Fu
#define TPS_VDCDC3_MILLIVOLT_1550   0x20u
#define TPS_VDCDC3_MILLIVOLT_1600   0x21u
#define TPS_VDCDC3_MILLIVOLT_1650   0x22u
#define TPS_VDCDC3_MILLIVOLT_1700   0x23u
#define TPS_VDCDC3_MILLIVOLT_1750   0x24u
#define TPS_VDCDC3_MILLIVOLT_1800   0x25u
#define TPS_VDCDC3_MILLIVOLT_1850   0x26u
#define TPS_VDCDC3_MILLIVOLT_1900   0x27u
#define TPS_VDCDC3_MILLIVOLT_1950   0x28u
#define TPS_VDCDC3_MILLIVOLT_2000   0x29u
#define TPS_VDCDC3_MILLIVOLT_2050   0x2Au
#define TPS_VDCDC3_MILLIVOLT_2100   0x2Bu
#define TPS_VDCDC3_MILLIVOLT_2150   0x2Cu
#define TPS_VDCDC3_MILLIVOLT_2200   0x2Du
#define TPS_VDCDC3_MILLIVOLT_2250   0x2Eu
#define TPS_VDCDC3_MILLIVOLT_2300   0x2Fu
#define TPS_VDCDC3_MILLIVOLT_2350   0x30u
#define TPS_VDCDC3_MILLIVOLT_2400   0x31u
#define TPS_VDCDC3_MILLIVOLT_2450   0x32u
#define TPS_VDCDC3_MILLIVOLT_2500   0x33u
#define TPS_VDCDC3_MILLIVOLT_2550   0x34u
#define TPS_VDCDC3_MILLIVOLT_2600   0x35u
#define TPS_VDCDC3_MILLIVOLT_2650   0x36u
#define TPS_VDCDC3_MILLIVOLT_2700   0x37u
#define TPS_VDCDC3_MILLIVOLT_2750   0x38u
#define TPS_VDCDC3_MILLIVOLT_2800   0x39u
#define TPS_VDCDC3_MILLIVOLT_2850   0x3Au
#define TPS_VDCDC3_MILLIVOLT_2900   0x3Bu
#define TPS_VDCDC3_MILLIVOLT_3000   0x3Cu
#define TPS_VDCDC3_MILLIVOLT_3100   0x3Du
#define TPS_VDCDC3_MILLIVOLT_3200   0x3Eu
#define TPS_VDCDC3_MILLIVOLT_3300   0x3Fu








/***********************************************************
* Global Function Declarations                             *
***********************************************************/
Uint32 		   TPS65070_open(Uint32 instance);
Uint8          TPS65070_reg_read(Uint32 instance, Uint8 regOffset, Uint8 *buf );
Uint32         TPS65070_reg_write(Uint32 instance, Uint8 regOffset, Uint8 regVal);
Uint32 		   TPS65070_set_DCDC3_voltage(Uint32 instance, Uint16 volt);
