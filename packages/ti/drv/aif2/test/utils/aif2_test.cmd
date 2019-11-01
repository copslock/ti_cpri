
/*For big endian, this needs to be changed to dsplib.ae66e*/

SECTIONS
{
    .csl_vect:      > L2SRAM
    platform_lib:   > L2SRAM
    .qmss:          > MSMCSRAM
    .cppi:          > MSMCSRAM
    .intData_sect:  > MSMCSRAM
    .aifdescmsm:    > MSMCSRAM
    .aifdescddr:    > DDR3
	.aifframeddr    > DDR3
	.aif2tracedata  > MSMCSRAM
	.appSyncSharedMem > MSMCSRAM
    .aifcaptcmd >       DDR3
    .aifrawtx   >       DDR3
    .aifrawrx   >       DDR3
    .dioData    > MSMCSRAM
}
