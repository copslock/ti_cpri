/*----------------------------------------------------------------------------*/
/* File: linker.cmd                                                           */
/* Description:								      */
/*    Link command file for Maxwell SBL                                       */
/*                                                                            */
/*    Platform: R5 Cores on AM65xx                                            */
/* (c) Texas Instruments 2018, All rights reserved.                           */
/*----------------------------------------------------------------------------*/
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
--fill_value=0
--stack_size=0x2000
--heap_size=0x2000
--entry_point=_sblResetVectors		/* SBL entry in SBL_init.asm	*/

-stack  0x2000                              /* SOFTWARE STACK SIZE           */
-heap   0x2000                              /* HEAP AREA SIZE                */

/* Stack Sizes for various modes */
__IRQ_STACK_SIZE = 0x1000;
__FIQ_STACK_SIZE = 0x1000;
__ABORT_STACK_SIZE = 0x800;
__UND_STACK_SIZE = 0x800;
__SVC_STACK_SIZE = 0x2000;

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY
{
    /*  Reset Vectors base address(RESET_VECTORS) should be 64 bytes aligned  */
	RESET_VECTORS (X)  			: origin=0x41C00100 length=0x100

    /* MCU0 memory used for SBL. Available to app for dynamic use ~160KB */
    /* RBL uses 0x41C58000 and beyond. SBL, at load cannot cross this */
    OCMRAM_SBL    (RWIX)   : origin=0x41C00200 length=0x3E000-0x200

    /* Used by SBL at runtime to load SYSFW. Available to app for dynamic use */
    OCMRAM_SBL_SYSFW (RWIX)   : origin=0x41C3E000 length=0x40000

}  /* end of MEMORY */

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */

SECTIONS
{
/* 'intvecs' and 'intc_text' sections shall be placed within                  */
/* a range of +\- 16 MB                                                       */
    .rstvectors           : {} palign(8)                            > RESET_VECTORS
    .bootCode    	      : {} palign(8)                     		> OCMRAM_SBL
    .startupCode 	      : {} palign(8)                     		> OCMRAM_SBL
    .startupData 	      : {} palign(8)                     		> OCMRAM_SBL, type = NOINIT
    .sbl_profile_info     : {} palign(8)                            > RESET_VECTORS  (HIGH)
    .text    	          : {} palign(8)                            > OCMRAM_SBL
    .const   	          : {} palign(8)                            > OCMRAM_SBL
    .cinit                : {} palign(8)                            > OCMRAM_SBL
    .pinit                : {} palign(8)                            > OCMRAM_SBL
    .boardcfg_data	  : {} palign(128)                          > OCMRAM_SBL

    .data                 : {} palign(128)                          > OCMRAM_SBL
    .bss     	          : {} align(4)                             > OCMRAM_SBL
    .sysmem  	          : {}                                      > OCMRAM_SBL

	.stack  	          : {} align(4)                             > OCMRAM_SBL  (HIGH)
	.irqStack  	          : {. = . + __IRQ_STACK_SIZE;} align(4)    > OCMRAM_SBL  (HIGH)
                            RUN_START(__IRQ_STACK_START)
                            RUN_END(__IRQ_STACK_END)
    .fiqStack  	          : {. = . + __FIQ_STACK_SIZE;} align(4)    > OCMRAM_SBL  (HIGH)
                            RUN_START(__FIQ_STACK_START)
                            RUN_END(__FIQ_STACK_END)
    .abortStack  	      : {. = . + __ABORT_STACK_SIZE;} align(4)  > OCMRAM_SBL  (HIGH)
                            RUN_START(__ABORT_STACK_START)
                            RUN_END(__ABORT_STACK_END)
    .undStack  	          : {. = . + __UND_STACK_SIZE;} align(4)    > OCMRAM_SBL  (HIGH)
                            RUN_START(__UND_STACK_START)
                            RUN_END(__UND_STACK_END)
    .svcStac              : {. = . + __SVC_STACK_SIZE;} align(4)    > OCMRAM_SBL  (HIGH)
                            RUN_START(__SVC_STACK_START)
                            RUN_END(__SVC_STACK_END)
    .firmware             : {} palign(8)                            > OCMRAM_SBL_SYSFW

}  /* end of SECTIONS */

/*----------------------------------------------------------------------------*/
/* Misc linker settings                                                       */


/*-------------------------------- END ---------------------------------------*/
