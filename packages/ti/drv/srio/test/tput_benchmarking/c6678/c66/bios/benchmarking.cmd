SECTIONS
{
    .init_array:    load >> L2SRAM
    .srioSharedMem: load >> MSMCSRAM
    .qmss:          load >> MSMCSRAM
    .cppi:          load >> MSMCSRAM
}
