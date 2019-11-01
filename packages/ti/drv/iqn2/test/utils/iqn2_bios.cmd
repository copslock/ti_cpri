

SECTIONS
{
    .csl_vect:      > L2SRAM
    platform_lib    > L2SRAM
    .intData_sect:  > MSMCSRAM
    .iqn2descmsm:    > MSMCSRAM
    .iqn2descddr:	> DDR3
    .intData_sect   > L2SRAM
    .text:dfe       > DDR3
    GROUP(MC_SHARED_DATASTRUCTURES)
    {
        .cppi:                  align=128  // CPPI LLD Multicore Datastructures
        .qmss:                  align=128  // QMSS LLD Multicore Datastructures
    } load=MSMCSRAM


}
