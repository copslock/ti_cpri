-m patch_dsp.map

MEMORY
{
  L2RAM       org=0x00800000 len=0x00001000 /* Use start of DSP L2 RAM */
}

SECTIONS
{
    .data:nand_ecc_patch > 0x00800000
    .text:nand_ecc_patch > L2RAM
    .text:bl1_nand_abort > L2RAM
    .text       > L2RAM
    .const      > L2RAM
    .bss        > L2RAM
    .far        > L2RAM
    .switch     > L2RAM
    .data       > L2RAM
    .stack      > L2RAM
    .fardata    > L2RAM
    .fardata    > L2RAM
    .neardata   > L2RAM
    .rodata     > L2RAM
    
    
}


