/*----------------------------------------------------------------------------*/
/* File: linker_r5f_mcu2_1_sysbios.lds                                        */
/* Description:								      */
/*    Link command file for J7ES MCU2_1 view				      */
/*	  TI ARM Compiler version 15.12.3 LTS or later			      */
/*                                                                            */
/* (c) Texas Instruments 2018, All rights reserved.                           */
/*----------------------------------------------------------------------------*/
/*  History:								      */
/*    Aug 26th, 2016 Original version .......................... Loc Truong   */
/*    Aug 01th, 2017 new TCM mem map  .......................... Loc Truong   */
/*    Nov 07th, 2017 Changes for R5F Init Code.................. Vivek Dhande */
/*    Sep 17th, 2018 Added DDR sections for IPC................. J. Bergsagel */
/*    Sep 26th, 2018 Extra mem sections for IPC resource table.. J. Bergsagel */
/*    Nov 07th, 2018 Add linker cmd file for MCU2_1............. J. Bergsagel */
/*    Apr 23th, 2019 Changes for R5F startup Code............... Vivek Dhande */
/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
/* Standard linker options						      */
--retain="*(.bootCode)"
--retain="*(.startupCode)"
--retain="*(.startupData)"
--fill_value=0
--stack_size=0x2000
--heap_size=0x1000

-stack  0x2000                              /* SOFTWARE STACK SIZE           */
-heap   0x2000                              /* HEAP AREA SIZE                */

#define DDR0_ALLOCATED_START 0xA0000000

#define MCU1_0_EXT_DATA_BASE     (DDR0_ALLOCATED_START + 0x00100000)
#define MCU1_0_R5F_MEM_TEXT_BASE (DDR0_ALLOCATED_START + 0x00200000)
#define MCU1_0_R5F_MEM_DATA_BASE (DDR0_ALLOCATED_START + 0x00300000)
#define MCU1_0_DDR_SPACE_BASE    (DDR0_ALLOCATED_START + 0x00400000)

#define MCU1_1_ALLOCATED_START   DDR0_ALLOCATED_START + 0x01000000
#define MCU1_1_EXT_DATA_BASE     (MCU1_1_ALLOCATED_START + 0x00100000)
#define MCU1_1_R5F_MEM_TEXT_BASE (MCU1_1_ALLOCATED_START + 0x00200000)
#define MCU1_1_R5F_MEM_DATA_BASE (MCU1_1_ALLOCATED_START + 0x00300000)
#define MCU1_1_DDR_SPACE_BASE    (MCU1_1_ALLOCATED_START + 0x00400000)

#define MCU2_0_ALLOCATED_START   DDR0_ALLOCATED_START + 0x02000000
#define MCU2_0_EXT_DATA_BASE     (MCU2_0_ALLOCATED_START + 0x00100000)
#define MCU2_0_R5F_MEM_TEXT_BASE (MCU2_0_ALLOCATED_START + 0x00200000)
#define MCU2_0_R5F_MEM_DATA_BASE (MCU2_0_ALLOCATED_START + 0x00300000)
#define MCU2_0_DDR_SPACE_BASE    (MCU2_0_ALLOCATED_START + 0x00400000)

#define MCU2_1_ALLOCATED_START   DDR0_ALLOCATED_START + 0x03000000
#define MCU2_1_EXT_DATA_BASE     (MCU2_1_ALLOCATED_START + 0x00100000)
#define MCU2_1_R5F_MEM_TEXT_BASE (MCU2_1_ALLOCATED_START + 0x00200000)
#define MCU2_1_R5F_MEM_DATA_BASE (MCU2_1_ALLOCATED_START + 0x00300000)
#define MCU2_1_DDR_SPACE_BASE    (MCU2_1_ALLOCATED_START + 0x00400000)

#define ATCM_START 0x00000000

-e __VECS_ENTRY_POINT

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY
{
    /* MCU2_R5F_1 local view  */
    MCU_ATCM (RWX)		: origin=ATCM_START	length=0x8000
    MCU2_R5F1_TCMB1 (RWIX)	: origin=0x41010000	length=0x8000

    /* MCU2_R5F_1 SoC view  */
    MCU2_R5F1_ATCM (RWIX)  	: origin=0x05d00000 length=0x8000
    MCU2_R5F1_BTCM (RWIX) 	: origin=0x05d10000 length=0x8000

    /* MCUSS RAM */
    OCMRAM_MCU1_0 (RWIX)        : origin=0x41c29caa length=0x14cd5
    OCMRAM_MCU1_1 (RWIX)        : origin=0x41c3e97f length=0x14cd5
    OCMRAM_MCU2_0 (RWIX)        : origin=0x41c53654 length=0x14cd5
    OCMRAM_MCU2_1 (RWIX)        : origin=0x41c68329 length=0x14cd5

    DDR0_RESERVED    (RWIX)  	: origin=0x80000000 length=0x20000000	  	/* 512MB */
    MCU1_0_IPC_DATA (RWIX)	: origin=DDR0_ALLOCATED_START     length=0x00100000	/*   1MB */
    MCU1_0_EXT_DATA  (RWIX)	: origin=MCU1_0_EXT_DATA_BASE     length=0x00100000	/*   1MB */
    MCU1_0_R5F_MEM_TEXT (RWIX)	: origin=MCU1_0_R5F_MEM_TEXT_BASE length=0x00100000	/*   1MB */
    MCU1_0_R5F_MEM_DATA (RWIX)	: origin=MCU1_0_R5F_MEM_DATA_BASE length=0x00100000	/*   1MB */
    MCU1_0_DDR_SPACE (RWIX)	: origin=MCU1_0_DDR_SPACE_BASE    length=0x00C00000	/*  12MB */
    MCU1_1_IPC_DATA (RWIX)	: origin=MCU1_1_ALLOCATED_START   length=0x00100000	/*   1MB */
    MCU1_1_EXT_DATA  (RWIX)	: origin=MCU1_1_EXT_DATA_BASE     length=0x00100000	/*   1MB */
    MCU1_1_R5F_MEM_TEXT (RWIX)	: origin=MCU1_1_R5F_MEM_TEXT_BASE length=0x00100000	/*   1MB */
    MCU1_1_R5F_MEM_DATA (RWIX)	: origin=MCU1_1_R5F_MEM_DATA_BASE length=0x00100000	/*   1MB */
    MCU1_1_DDR_SPACE (RWIX)	: origin=MCU1_1_DDR_SPACE_BASE    length=0x00C00000	/*  12MB */
    MCU2_0_IPC_DATA (RWIX)	: origin=MCU2_0_ALLOCATED_START   length=0x00100000	/*   1MB */
    MCU2_0_EXT_DATA  (RWIX)	: origin=MCU2_0_EXT_DATA_BASE     length=0x00100000	/*   1MB */
    MCU2_0_R5F_MEM_TEXT (RWIX)	: origin=MCU2_0_R5F_MEM_TEXT_BASE length=0x00100000	/*   1MB */
    MCU2_0_R5F_MEM_DATA (RWIX)	: origin=MCU2_0_R5F_MEM_DATA_BASE length=0x00100000	/*   1MB */
    MCU2_0_DDR_SPACE (RWIX)	: origin=MCU2_0_DDR_SPACE_BASE    length=0x00C00000	/*  12MB */
    MCU2_1_IPC_DATA (RWIX)	: origin=MCU2_1_ALLOCATED_START   length=0x00100000	/*   1MB */
    MCU2_1_EXT_DATA  (RWIX)	: origin=MCU2_1_EXT_DATA_BASE     length=0x00100000	/*   1MB */
    MCU2_1_R5F_MEM_TEXT (RWIX)	: origin=MCU2_1_R5F_MEM_TEXT_BASE length=0x00100000	/*   1MB */
    MCU2_1_R5F_MEM_DATA (RWIX)	: origin=MCU2_1_R5F_MEM_DATA_BASE length=0x00100000	/*   1MB */
    MCU2_1_DDR_SPACE (RWIX)	: origin=MCU2_1_DDR_SPACE_BASE    length=0x00C00000	/*  12MB */

    SHARED_DDR_SPACE (RWIX)	: origin=0xAA000000 length=0x01C00000           /*  28MB */

}  /* end of MEMORY */

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */

SECTIONS
{
    .vecs : {
         *(.vecs)
    } palign(8) > ATCM_START
    .vecs       : {
        __VECS_ENTRY_POINT = .;
    } palign(8) > MCU_ATCM
    .init_text  : {
                     boot.*(.text)
                     *(.text:ti_sysbios_family_arm_MPU_*)
                     *(.text:ti_sysbios_family_arm_v7r_Cache_*)
                  }  > MCU_ATCM
    .text:xdc_runtime_Startup_reset__I : {} palign(8) > MCU_ATCM
    .bootCode    	: {} palign(8) 		> MCU_ATCM
    .startupCode 	: {} palign(8) 		> MCU_ATCM
    .startupData 	: {} palign(8) 		> MCU_ATCM, type = NOINIT
    .text	: {} palign(8)		> MCU2_1_DDR_SPACE
    .const   	: {} palign(8) 		> MCU2_1_DDR_SPACE
    .cinit   	: {} palign(8) 		> MCU2_1_DDR_SPACE
    .pinit   	: {} palign(8) 		> MCU2_1_DDR_SPACE
    .bss     	: {} align(4)  		> MCU2_1_DDR_SPACE
    .data    	: {} palign(128) 	> MCU2_1_DDR_SPACE
    .data_buffer: {} palign(128) 	> MCU2_1_DDR_SPACE
    .sysmem  	: {} 			> MCU2_1_DDR_SPACE
    .stack	: {} align(4)		> MCU2_1_DDR_SPACE
    ipc_data_buffer (NOINIT) : {} palign(128)	> MCU2_1_DDR_SPACE
    .resource_table : {
        __RESOURCE_TABLE = .;
    } > MCU2_1_EXT_DATA_BASE

    .tracebuf   : {}			> MCU2_1_EXT_DATA

}  /* end of SECTIONS */

/*----------------------------------------------------------------------------*/
/* Misc linker settings                                                       */


/*-------------------------------- END ---------------------------------------*/
