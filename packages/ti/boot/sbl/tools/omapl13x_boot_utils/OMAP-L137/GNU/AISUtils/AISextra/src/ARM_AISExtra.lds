/*
 *  Desc: Linker command file sft application program.
 *  Auth: Daniel Allred
 */

ENTRY(start)
SECTIONS 
{
  
  EXTERNAL_RAM_SIZE   = 0x10000000;  
  EXTERNAL_RAM_START  = 0xC0000000;
  EXTERNAL_RAM_END    = EXTERNAL_RAM_START + EXTERNAL_RAM_SIZE;
  
  ASYNC_MEM_START     = 0x02000000;

  IRAM_START          = 0x80000000;
  IRAM_SIZE           = 0x00008000;

  INTERNAL_RAM_START = 0x80000000;
  INTERNAL_RAM_SIZE  = 0x00008000;
  
  STACK_START         = 0x80008800;
  STACK_SIZE          = 0x00000800;
  
  . = IRAM_START;
  .text    : 
  {
    *(.text:.start)
    *(.text)
    . = ALIGN(4);

  }
  
  /* Move into DRAM for placing const and data sections */
  .rodata    :
  {
    *(.rodata*)
    *(.rodata)
    . = ALIGN(4);
  }  
  .data    : 
  {
    *(.data)
    . = ALIGN(4);
  }
        
  .bss    :
  {
    *(.bss) *(COMMON)
    . = ALIGN(4);
  }
}

