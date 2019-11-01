-x
-stack 0x10000
-heap  0x6000


MEMORY
{

  VECTORS_MEM: origin = 0x00800000 length = 0x00000400
	
  GEM_TEXT_MEM: origin = 0x00802000 length = 0x0009E000
  GEM1_TEXT_MEM: origin = 0x11802000 length = 0x0009E000

  DDR_MEM_CMD:    	 origin = 0xE0000000 length = 0x00001000
  DDR_MEM_RAW_TX: 	 origin = 0xE0001000 length = 0x00078C00
  DDR_MEM_RAW_RX: 	 origin = 0xE0079C00 length = 0x00086400
  DDR_MEM_CAPT:   	 origin = 0xE0100000 length = 0x00F00000
  DDR_MEM:           origin = 0xE3580000 length = 0x0CA80000
  DDR_MEM_DFE_SZE:   origin = 0xF0000000 length = 0x00000010
  DDR_MEM_DFE_CFG:   origin = 0xF0000010 length = 0x0FFFFFE0

  /* M3 Config Memory */
  M3_CONFIG_MEM: origin = 0x08000000 length = 0x00008000

  /* M3 RAM */
  M3_RAM_MEM: origin = 0x0C000000 length = 0x00200000

  OSR_RAM_MEM: origin = 0x70000000 length = 0x00100000

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
  .text       >       DDR_MEM
  .cppi		  >		  M3_RAM_MEM
  .qmss		  >		  M3_RAM_MEM
  .intData_sect   >   GEM_TEXT_MEM
  .iqn2descmsm   >    M3_RAM_MEM
  .iqn2ctldesc >      M3_RAM_MEM //GEM_TEXT_MEM //OSR_RAM_MEM //GEM_TEXT_MEM //M3_RAM_MEM
  .iqn2buffer >       DDR_MEM //M3_RAM_MEM //GEM1_TEXT_MEM// DDR_MEM
  .extData_sect   >   DDR_MEM
  .iqn2descddr >      DDR_MEM
  .dfetgtcfgsize >    DDR_MEM_DFE_SZE
  .dfetgtcfg     >    DDR_MEM_DFE_CFG
  .text:dfe   >       DDR_MEM
  p0_2_tone_axc1_in_DATA > DDR_MEM
  p0_2_tone_axc2_in_DATA > DDR_MEM
  p0_2_tone_axc3_in_DATA > DDR_MEM
  p0_2_tone_axc4_in_DATA > DDR_MEM
}
