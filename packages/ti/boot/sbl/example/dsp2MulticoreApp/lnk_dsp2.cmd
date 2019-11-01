

-stack  0x00500000                            /* SOFTWARE STACK SIZE       */
-heap   0x00500000                         /* HEAP AREA SIZE               */
-e      _c_int00

/* SPECIFY THE SYSTEM MEMORY MAP */

MEMORY
{
    IRAM_MEM:     org = 0x00809000 len = 0x8000      /*   RAM 0x1FBFF*/
	/*CODE SECTION - 20MB*/
	/*DATA SECTION - 10MB*/
	/*STACK - 5MB*/
	/*HEAP - 5MB*/
	DDR3_MPU_CPU0_CODE : org = 0x90000000,len = 0x01400000
	DDR3_MPU_CPU0_DATA :  org = 0x91400000, len = 0x00A00000
	DDR3_MPU_CPU0_STACK : org = 0x91E00000, len = 0x00500000
	DDR3_MPU_CPU0_HEAP :  org = 0x92300000, len = 0x00500000

	DDR3_DSP1_CODE : org = 0x83200000,len = 0x01400000
	DDR3_DSP1_DATA : org = 0x84600000, len = 0x00A00000
	DDR3_DSP1_STACK :org = 0x85000000, len = 0x00500000
	DDR3_DSP1_HEAP : org = 0x85500000, len = 0x00500000

	DDR3_DSP2_CODE : org = 0x86400000,len = 0x01400000
	DDR3_DSP2_DATA :  org = 0x87800000,len = 0x00A00000
	DDR3_DSP2_STACK : org = 0x88200000,len = 0x00500000
	DDR3_DSP2_HEAP :  org = 0x88700000,len = 0x00500000
}

/* SPECIFY THE SECTIONS ALLOCATION INTO MEMORY */

SECTIONS
{
    .vects : load > IRAM_MEM
    .l2_int  : load > IRAM_MEM
	boot :
	{
		rts*.lib<boot.obj>(.text)
	}load > DDR3_DSP2_CODE

    .init    : load > DDR3_DSP2_DATA
    .cio     : load > DDR3_DSP2_DATA

    .text    : load > DDR3_DSP2_CODE /* CODE */
    .data    : load > DDR3_DSP2_DATA /* INITIALIZED GLOBAL AND STATIC VARIABLES. */
    .bss     : load > DDR3_DSP2_DATA /* UNINITIALIZED OR ZERO INITIALIZED */
                                            /* GLOBAL & STATIC VARIABLES.   */
                    RUN_START(bss_start)
                    RUN_END(bss_end)
    .const   : load > DDR3_DSP2_DATA              /* GLOBAL CONSTANTS   */
    .cinit   : load > DDR3_DSP2_DATA
    .stack   : load > DDR3_DSP2_STACK            /* SOFTWARE SYSTEM STACK */
    .plt     : load > DDR3_DSP2_DATA
	.my_sect_ddr : load > DDR3_DSP2_DATA
	.far  : load > DDR3_DSP2_DATA
    .fardata : load > DDR3_DSP2_DATA
	.neardata : load > DDR3_DSP2_DATA
	.switch : load > DDR3_DSP2_DATA
}
