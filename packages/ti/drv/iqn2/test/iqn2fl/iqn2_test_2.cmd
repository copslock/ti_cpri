-x
-stack 0x10000
-heap  0x6000


MEMORY
{

  VECTORS_MEM: origin = 0x00800000 length = 0x00000400
	
  GEM_TEXT_MEM: origin = 0x00802000 length = 0x0009E000

  DDR3A_0_MEM: origin = 0x80000000 length = 0x20000000
  DDR3B_0_MEM: origin = 0xA0000000 length = 0x20000000

  /* M3 Config Memory */
  M3_CONFIG_MEM: origin = 0x08000000 length = 0x00008000

  /* M3 RAM */
  M3_RAM_MEM: origin = 0x0C000000 length = 0x00200000

}
 
SECTIONS
{
  .vectors: > VECTORS_MEM

  .csl_vect:  >       GEM_TEXT_MEM
  .fasttext:  >       GEM_TEXT_MEM
  .cinit:     >       GEM_TEXT_MEM
  .stack:     >       GEM_TEXT_MEM
  .bss:       >       GEM_TEXT_MEM
  .boot       >       GEM_TEXT_MEM
  .const:     >       GEM_TEXT_MEM
  .far:       >       GEM_TEXT_MEM
  .fardata    >       GEM_TEXT_MEM
  .neardata   >       GEM_TEXT_MEM
  .rodata     >       GEM_TEXT_MEM
  .switch:    >       GEM_TEXT_MEM
  .sysmem:    >       GEM_TEXT_MEM
  .cio:       >       GEM_TEXT_MEM
  .text       >       GEM_TEXT_MEM
  .cppi		  >		  M3_RAM_MEM
  .qmss		  >		  M3_RAM_MEM
  .intData_sect   >   M3_RAM_MEM
  .iqn2descmsm    >   M3_RAM_MEM
  .extData_sect   >   DDR3A_0_MEM
  .sharedGRL  >       GEM_TEXT_MEM
  .sharedPolicy >     GEM_TEXT_MEM
  .text:dfe     >     DDR3B_0_MEM
  
}
