SECTIONS
{
    .init_array:    load >> L2SRAM
    .sharedGRL:     load >> L2SRAM
    .sharedPolicy:  load >> L2SRAM
    .srioSharedMem: load >> MSMCSRAM
    .qmss:          load >> MSMCSRAM
    .rm:            load >> MSMCSRAM
    .cppi:          load >> MSMCSRAM
}

