

SECTIONS
{
    .csl_vect:      > L2SRAM
    platform_lib    > L2SRAM
    .intData_sect:  > MSMCSRAM
    .aifdescmsm:    > MSMCSRAM
    .aifdescddr:    > DDR3
	.aifframeddr    > DDR3
    .aif2			> L2SRAM
    GROUP(MC_SHARED_DATASTRUCTURES)
    {
        .cfgAifSection:			align=128  // Aif Config Section
        .cppi:                  align=128  // CPPI LLD Multicore Datastructures
        .qmss:                  align=128  // QMSS LLD Multicore Datastructures
        .appSyncSharedMem:      align=128  // Used to sync SoC Init core & other cores on Init completion
    } load=MSMCSRAM


}
