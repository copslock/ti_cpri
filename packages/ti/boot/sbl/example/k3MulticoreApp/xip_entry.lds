/*----------------------------------------------------------------------------*/
/* File: xip_entry.lds                                                           */
/* Description:								      */
/*    Link command file for Maxwell Multicore Testcase			      */
/*                                                                            */
/*    Platform: R5 Cores on AM65xx                                            */
/* (c) Texas Instruments 2019, All rights reserved.                           */
/*----------------------------------------------------------------------------*/

_xipAppEntry=0x500e0000;
--entry_point=_xipAppEntry
--retain="_xipAppEntry"
--retain="_xip_stub"

MEMORY
{
    MSMC_XIP_ENTRY	        : origin=0x7002C000 length=0x2000	/* 8KB */
}

SECTIONS
{
    .xip_entry 	        : {} palign(8) 		> MSMC_XIP_ENTRY
}
