
SECTIONS
{
	.qmss     > MSMCSRAM
	.cppi     > MSMCSRAM
	.testPkts > L2SRAM
    .pktLibLocalMemory > L2SRAM
    .QMemGlobDescRam > MSMCSRAM
    .cppiMemPaSaLinkBuf > MSMCSRAM
    .cppiMemSaPaLinkBuf > MSMCSRAM

    /* If NWAL_TEST_DESC_GLOB_MEM is  defined uncomment below placements to DDR3 and comment for L2SRAM */
     .QMemLocDescRam > DDR3
    /* .QMemLocDescRam > MSMCSRAM */

    /*.QMemLocDescRam >L2SRAM  */
    /*.pktLibLocalMemory > L2SRAM */

    /* If NWAL_TEST_DESC_BUF_GLOB_MEM is  defined uncomment below placements to DDR3 and comment for L2SRAM */
    /*.cppiMemRxCtlLinkBuf > MSMCSRAM
    .cppiMemTxCtlLinkBuf > MSMCSRAM
    .cppiMemRxPktLinkBuf > MSMCSRAM
    .cppiMemTxPktLinkBuf > MSMCSRAM */
    
    .cppiMemRxCtlLinkBuf > DDR3
    .cppiMemTxCtlLinkBuf > DDR3
    .cppiMemRxPktLinkBuf > DDR3
    .cppiMemTxPktLinkBuf > DDR3

    /*.cppiMemRxCtlLinkBuf > L2SRAM
    .cppiMemTxCtlLinkBuf > L2SRAM
    .cppiMemRxPktLinkBuf > L2SRAM
    .cppiMemTxPktLinkBuf > L2SRAM  */ 

    .nwalProcessCtx > MSMCSRAM
    .nwalInstMem > MSMCSRAM
    .nwalHandleMem > MSMCSRAM
    .paBuf0 > MSMCSRAM
    .paBuf1 > MSMCSRAM
    .paBuf2  > MSMCSRAM
    .salldHandle > MSMCSRAM
    .saContext > MSMCSRAM
    .salldChanHandle > MSMCSRAM
    .salldChanHandle > MSMCSRAM
    .testNwGlobContext > MSMCSRAM
    .pktLibSharedMemory > MSMCSRAM
}
