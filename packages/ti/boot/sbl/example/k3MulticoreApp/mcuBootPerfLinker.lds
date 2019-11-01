/*----------------------------------------------------------------------------*/
/* File: k3m4_r5f_linker.cmd                                                  */
/* Description:																  */
/*    Link command file for AM65XX M4 MCU 0 view							  */
/*	  TI ARM Compiler version 15.12.3 LTS or later							  */
/*                                                                            */
/*    Platform: QT                                                            */
/* (c) Texas Instruments 2017, All rights reserved.                           */
/*----------------------------------------------------------------------------*/
/*  History:															      *'
/*    Aug 26th, 2016 Original version .......................... Loc Truong   */
/*    Aug 01th, 2017 new TCM mem map  .......................... Loc Truong   */
/*    Nov 07th, 2017 Changes for R5F Init Code.................. Vivek Dhande */
/*    Mar 26th, 2019 Changes for R5F startup Code............... Vivek Dhande */
/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
/* Standard linker options													  */
--retain="*(.bootCode)"
--retain="*(.startupCode)"
--retain="*(.startupData)"
--retain="*(.intvecs)"
--retain="*(.intc_text)"
--retain="*(.rstvectors)"
--retain="*(.irqStack)"
--retain="*(.fiqStack)"
--retain="*(.abortStack)"
--retain="*(.undStack)"
--retain="*(.svcStack)"
--retain="_boot_perf_app_pad_0"
--retain="_boot_perf_app_pad_1"
--retain="_boot_perf_app_pad_2"
--retain="_boot_perf_app_pad_3"
--retain="_boot_perf_app_pad_4"
--retain="_boot_perf_app_pad_5"
--fill_value=0
--stack_size=0x2000
--heap_size=0x1000
--entry_point=_resetvectors		/* Default C RTS boot.asm	*/


-stack  0x2000                              /* SOFTWARE STACK SIZE           */
-heap   0x2000                              /* HEAP AREA SIZE                */

/* Stack Sizes for various modes */
__IRQ_STACK_SIZE = 0x1000;
__FIQ_STACK_SIZE = 0x1000;
__ABORT_STACK_SIZE = 0x1000;
__UND_STACK_SIZE = 0x1000;
__SVC_STACK_SIZE = 0x1000;

/* interface with SBL */
sblProfileLogAddr = 0x41c001f0;
sblProfileLogIndxAddr = 0x41c001f4;
sblProfileLogOvrFlwAddr = 0x41c001f8;
syfw_image = 0x41C3E000;

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY
{
        /* Used for SYSFW initlization */
        OCMRAM_SBL_SYSFW (RWIX): origin=0x41C3E000 length=0x40000
	OCMRAM_SBL_UNUSED (X)  : origin=0x41C7F000 length=0x1000

        /* App code resides here  */
	RESET_VECTORS (X)      : origin=0x70100000 length=0x100
	MSMC3_H                : origin=0x70100100 length=0x50000 - 0x1100 /* 256KB */
	VECTORS (X)            : origin=0x701FF000 length=0x1000

        /* Dummy buffers to pad the app size */
	MSMC3_PAD_0            : origin=0x70000000 length=0x40000 /* 256KB */
	MSMC3_PAD_1            : origin=0x70040000 length=0x40000 /* 256KB */
	MSMC3_PAD_2            : origin=0x70080000 length=0x40000 /* 256KB */
	MSMC3_PAD_3            : origin=0x70150000 length=0x40000 /* 256KB */
	MSMC3_PAD_4            : origin=0x70190000 length=0x40000 /* 256KB */


/* Additional memory settings	*/

}  /* end of MEMORY */

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */

SECTIONS
{
/* 'intvecs' and 'intc_text' sections shall be placed within                  */
/* a range of +\- 16 MB                                                       */
    .intvecs 	: {} palign(8) 		> VECTORS
    .intc_text 	: {} palign(8) 		> VECTORS
    .rstvectors : {} palign(8) 		> RESET_VECTORS
    .bootCode    	: {} palign(8) 		> MSMC3_H
    .startupCode 	: {} palign(8) 		> MSMC3_H
    .startupData 	: {} palign(8) 		> MSMC3_H, type = NOINIT
    .sbl_mcu_1_0_resetvector : {} palign(8) > MSMC3_H
    .text    	: {} palign(8) 		> MSMC3_H
    .const   	: {} palign(8) 		> MSMC3_H
    .cinit   	: {} palign(8) 		> MSMC3_H
    .pinit   	: {} palign(8) 		> MSMC3_H
    .bss     	: {} align(4)  		> MSMC3_H
    .data    	: {} palign(128) 	> MSMC3_H
    .boardcfg_data  : {} palign(128)    > MSMC3_H

    .sysmem  	: {} 			> MSMC3_H
    .stack  	: {} align(8)		> MSMC3_H  (HIGH)
    .irqStack  	: {. = . + __IRQ_STACK_SIZE;} align(4)		> MSMC3_H  (HIGH)
    RUN_START(__IRQ_STACK_START)
    RUN_END(__IRQ_STACK_END)
    .fiqStack  	: {. = . + __FIQ_STACK_SIZE;} align(4)		> MSMC3_H  (HIGH)
    RUN_START(__FIQ_STACK_START)
    RUN_END(__FIQ_STACK_END)
    .abortStack  : {. = . + __ABORT_STACK_SIZE;} align(4)       > MSMC3_H  (HIGH)
    RUN_START(__ABORT_STACK_START)
    RUN_END(__ABORT_STACK_END)
    .undStack  	: {. = . + __UND_STACK_SIZE;} align(4)		> MSMC3_H  (HIGH)
    RUN_START(__UND_STACK_START)
    RUN_END(__UND_STACK_END)
    .svcStack  	: {. = . + __SVC_STACK_SIZE;} align(4)		> MSMC3_H  (HIGH)
    RUN_START(__SVC_STACK_START)
    RUN_END(__SVC_STACK_END)

    .imgPad_0    	: {} palign(8) 		> MSMC3_PAD_0
    .imgPad_1    	: {} palign(8) 		> MSMC3_PAD_1
    .imgPad_2    	: {} palign(8) 		> MSMC3_PAD_2
    .imgPad_3    	: {} palign(8) 		> MSMC3_PAD_3
    .imgPad_4    	: {} palign(8) 		> MSMC3_PAD_4
/* Additional sections settings 	*/
    .sysfw_data_cfg_board      : {} palign(128) > OCMRAM_SBL_UNUSED (HIGH)
    .sysfw_data_cfg_board_rm   : {} palign(128) > OCMRAM_SBL_UNUSED (HIGH)
    .sysfw_data_cfg_board_sec  : {} palign(128) > OCMRAM_SBL_UNUSED (HIGH)

}  /* end of SECTIONS */

/*----------------------------------------------------------------------------*/
/* Misc linker settings                                                       */


/*-------------------------------- END ---------------------------------------*/
