

/****************************************************************************/
/* LNK32.CMD - v4.5.0 COMMAND FILE FOR LINKING TMS470 32BIS C/C++ PROGRAMS  */
/*                                                                          */
/*   Usage:  armlnk <obj files...>    -o <out file> -m <map file> lnk32.cmd */
/*           armcl <src files...> -z -o <out file> -m <map file> lnk32.cmd  */
/*                                                                          */
/*   Description: This file is a sample command file that can be used       */
/*                for linking programs built with the TMS470 C/C++          */
/*                Compiler.   Use it as a guideline; you may want to change */
/*                the allocation scheme according to the size of your       */
/*                program and the memory layout of your target system.      */
/*                                                                          */
/*   Notes: (1)   You must specify the directory in which run-time support  */
/*                library is located.  Either add a "-i<directory>" line to */
/*                this file, or use the system environment variable C_DIR   */
/*                to specify a search path for libraries.                   */
/*                                                                          */
/*          (2)   If the run-time support library you are using is not      */
/*                named below, be sure to use the correct name here.        */
/*                                                                          */
/****************************************************************************/

-e _reset_handler
-heap  0x1000
-stack 0x500000

MEMORY
{
    PAGE 0:
      VECMEM  :    origin      = 0x89000000, length = 0x0100
      CMDMEM  :    origin      = 0x89000100, length = 0x1000
      EXTMEM  :    origin      = 0x89001100, length = 0x10000

    PAGE 1:
      DATMEM  :    origin = 0x40020000, length = 0x8000
      WMEM    :    origin = 0x40040000, length = 0x7E00
      IMEMLA  :    origin = 0x40050000, length = 0x4000
      IMEMHA  :    origin = 0x40054000, length = 0x4000
      IMEMLB  :    origin = 0x40070000, length = 0x4000
      IMEMHB  :    origin = 0x40074000, length = 0x4000
      EXTDMEM :    origin = 0x89030000, length = 0x1000000
}


SECTIONS
{
  .intvecs      > VECMEM  PAGE 0
  .inthandler   > CMDMEM  PAGE 0
  .exitlocation > CMDMEM  PAGE 0

  .text > EXTMEM   PAGE 0
  {
    *(.text)
  }

  GROUP
  {
      .bss            /* This order facilitates a single segment for */
      .data           /* GDP-relative addressing                     */
      .rodata
  }>DATMEM PAGE 1

  .const       > DATMEM  PAGE 1
  Adata        > IMEMLA  PAGE 1
  Bdata        > IMEMHA  PAGE 1
  Cdata        > WMEM    PAGE 1
  EOutdata     > DATMEM  PAGE 1
  Sdata        > DATMEM  PAGE 1
  Udata        > WMEM    PAGE 1
  Vdata        > WMEM    PAGE 1
  Ydata        > WMEM    PAGE 1
  .cint        > EXTDMEM  PAGE 1
  .stack       > EXTDMEM  PAGE 1
  .sysmem      > EXTDMEM  PAGE 1
  .cinit       > EXTDMEM  PAGE 1
  .cio         > EXTDMEM  PAGE 1
  .far         > EXTDMEM  PAGE 1
  .fardata     > EXTDMEM  PAGE 1
  .init_array  > EXTDMEM  PAGE  1
  .vcop_parameter_block > WMEM PAGE 1

   SRC_MEM        > EXTDMEM    PAGE 1
   DDR_MEM        > EXTDMEM    PAGE 1
   DST_MEM        > EXTDMEM    PAGE 1
   IBUFLA         > IMEMLA     PAGE 1
   IBUFHA         > IMEMHA     PAGE 1
   IBUFLB         > IMEMLB     PAGE 1
   IBUFHB         > IMEMHB     PAGE 1
   WBUF           > WMEM       PAGE 1
}













