-m patch_arm.map

MEMORY
{
  PATCH_RAM       org=0xffff0b00 len=0x00000500 /* Small unused portion of ARM internal memory */
}

SECTIONS
{
    .data:nand_ecc_patch > 0xffff0b00
    .text:nand_ecc_patch > PATCH_RAM
    .text:bl1_nand_abort > PATCH_RAM
    .text:close_boot     > PATCH_RAM
    .const      		 > PATCH_RAM
    .bss        		 > PATCH_RAM
    .far        		 > PATCH_RAM
    .switch     		 > PATCH_RAM
    .data       		 > PATCH_RAM
    .stack      		 > PATCH_RAM
}


