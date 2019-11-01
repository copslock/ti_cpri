/*----------------------------------------------------------------------------*/
/* File: mcuXiplinker.cmd                                                           */
/* Description:								      */
/*    Link command file for Maxwell Multicore Testcase			      */
/*                                                                            */
/*    Platform: R5 Cores on AM65xx                                            */
/* (c) Texas Instruments 2019, All rights reserved.                           */
/*----------------------------------------------------------------------------*/

--entry_point=_sblTestResetVectors
--stack_size=0x1000

MEMORY
{
    OSPI_MCU1_CPU0_XIP_VEC      : origin=0x500e0000 length=0x40			/* 64B */
    OSPI_MCU1_CPU0_XIP	        : origin=0x500e0040 length=0x2000 - 0x40	/* 8KB - 64B*/
    MSMC3_MCU1_CPU0	        : origin=0x70010800 length=0x2000 - 0x800	/* 8KB */
}

SECTIONS
{
    .rstvectors			: {} palign(8) 		> OSPI_MCU1_CPU0_XIP_VEC
    .sbl_mcu_1_0_resetvector	: {} palign(8) 		> OSPI_MCU1_CPU0_XIP
    .stack 			: {} palign(8) 		> MSMC3_MCU1_CPU0
}

