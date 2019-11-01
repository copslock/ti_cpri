
SECTIONS
{
	.qmss     > L2SRAM
	.cppi     > L2SRAM
	.testPkts > L2SRAM
    .QMemGlobDescRam > L2SRAM
    .cppiMemPaSaLinkBuf > L2SRAM
    .cppiMemSaPaLinkBuf > L2SRAM
    .pktLibLocalMemory > L2SRAM

    /* If NWAL_TEST_DESC_GLOB_MEM is  defined uncomment below placements to L2SRAM and comment for L2SRAM */
    /*  .QMemLocDescRam > DDR3  */
    .QMemLocDescRam > L2SRAM

    /* If NWAL_TEST_DESC_BUF_GLOB_MEM is  defined uncomment below placements to DDR3 and comment for L2SRAM */
    /*.cppiMemRxCtlLinkBuf > L2SRAM
    .cppiMemTxCtlLinkBuf > L2SRAM
    .cppiMemRxPktLinkBuf > L2SRAM
    .cppiMemTxPktLinkBuf > L2SRAM */

    .cppiMemRxCtlLinkBuf > L2SRAM
    .cppiMemTxCtlLinkBuf > L2SRAM
    .cppiMemRxPktLinkBuf > L2SRAM
    .cppiMemTxPktLinkBuf > L2SRAM   

    .nwalProcessCtx > MSMCSRAM
    .nwalInstMem > L2SRAM
    .nwalHandleMem > L2SRAM
    .paBuf0 > L2SRAM
    .paBuf1 > L2SRAM
    .paBuf2  > L2SRAM
    .salldHandle > L2SRAM
    .saContext > L2SRAM
    .salldChanHandle > L2SRAM
    .salldChanHandle > L2SRAM
    .testNwGlobContext > L2SRAM
    .pktLibSharedMemory > L2SRAM
}
