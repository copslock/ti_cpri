
MEMORY
{
    L1D_PARAM   org=0x00F00000 len=0x00000040 /* Function Args */
    L2RAM		org=0x00800000 len=0x00002000 /* L2 RAM/Cache */
    EMIFB_MEM   org=0xC0000000 len=0x10000000 /* DDR EMIF3a memory */
}

SECTIONS
{
	.params		> L1D_PARAM
	.cinit      > L2RAM
	.text       > L2RAM
	.const      > L2RAM
	.bss        > L2RAM
	.far		> L2RAM
	.switch		> L2RAM
	.data       > L2RAM
	.stack      > L2RAM
    .fardata    > L2RAM
    .fardata    > L2RAM
    .neardata   > L2RAM
    .rodata     > L2RAM

  .sdram
  {
    . += 0x10000000;
  } load=EMIFB_MEM, type=DSECT, FILL=0x00000000, START(_EXTERNAL_RAM_START)  
}


