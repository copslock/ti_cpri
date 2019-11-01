
SECTIONS
{
    .csl_vect:      > L2SRAM
    platform_lib:   > L2SRAM
    .qmss:          > MSMCSRAM
    .cppi:          > MSMCSRAM
    .intData_sect:  > MSMCSRAM
    .iqn2descmsm:	> MSMCSRAM
    .iqn2descddr:	> DDR3
	.text:dfe       > DDR3
}
