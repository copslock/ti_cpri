/*----------------------------------------------------------------------------*/
/* File: linker_r5f_mcu1_1_sysbios.lds                                        */
/* Description:			  					      */
/*    Link command file for J7ES MCU1_1 view				      */
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
/*    Nov 06th, 2018 Linker command file for MCU1_1............. J. Bergsagel */
/*    Feb 21st, 2019 Use R5F BTCM memory for boot vectors........J. Bergsagel */
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
#define BTCM_START 0x41010000

-e __VECS_ENTRY_POINT

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY
{
    /* MCU1_R5F_1 local view  */
    MCU_ATCM (RWX)		: origin=ATCM_START	length=0x8000
    MCU1_R5F1_TCMB1_VECS (RWIX) : origin=BTCM_START     length=0x0100
    MCU1_R5F1_TCMB1 (RWIX)      : origin=0x41010100     length=0x7F00

    /* MCU1_R5F_1 SoC view  */
    MCU1_R5F1_ATCM (RWIX)  	: origin=0x41400000 length=0x8000
    MCU1_R5F1_BTCM (RWIX)  	: origin=0x41410000 length=0x8000

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
    } palign(8) > BTCM_START
    .vecs       : {
        __VECS_ENTRY_POINT = .;
    } > MCU1_R5F1_TCMB1_VECS
    xdc.meta (COPY): { *(xdc.meta) } > MCU1_R5F1_TCMB1
    .init_text  : {
                     boot.*(.text)
                     *(.text:ti_sysbios_family_arm_MPU_*)
                     *(.text:ti_sysbios_family_arm_v7r_Cache_*)
                  }  palign(8) > MCU1_R5F1_TCMB1
    .text:xdc_runtime_Startup_reset__I     : {} palign(8) >  MCU1_R5F1_TCMB1
    .bootCode    	: {} palign(8) 		> MCU1_R5F1_TCMB1
    .startupCode 	: {} palign(8) 		> MCU1_R5F1_TCMB1
    .startupData 	: {} palign(8) 		> MCU1_R5F1_TCMB1, type = NOINIT
    .text	: {} palign(8)		> MCU1_1_DDR_SPACE
    .const   	: {} palign(8) 		> MCU1_1_DDR_SPACE
    .cinit   	: {} palign(8) 		> MCU1_1_DDR_SPACE
    .pinit   	: {} palign(8) 		> MCU1_1_DDR_SPACE
    .bss     	: {} align(4)  		> MCU1_1_DDR_SPACE
    .data    	: {} palign(128) 	> MCU1_1_DDR_SPACE
    .data_buffer: {} palign(128) 	> MCU1_1_DDR_SPACE
    .sysmem  	: {} 			> MCU1_1_DDR_SPACE
    .stack	: {} align(4)		> MCU1_1_DDR_SPACE
    ipc_data_buffer (NOINIT) : {} palign(128)	> MCU1_1_DDR_SPACE
    .resource_table : {
        __RESOURCE_TABLE = .;
    } > MCU1_1_EXT_DATA_BASE

    .tracebuf   : {}			> MCU1_1_EXT_DATA

}  /* end of SECTIONS */

/*----------------------------------------------------------------------------*/
/* Misc linker settings                                                       */


/*-------------------------------- END ---------------------------------------*/
