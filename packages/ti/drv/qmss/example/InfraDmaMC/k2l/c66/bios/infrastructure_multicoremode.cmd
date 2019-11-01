SECTIONS
{
    .init_array: load >> L2SRAM
    .sharedGRL: load >> L2SRAM
    .sharedPolicy: load >> L2SRAM
    .qmss: load >> MSMCSRAM
    .rm: load >> MSMCSRAM
    .cppi: load >> L2SRAM
    .csl_vect: load >> L2SRAM
    cppiSharedHeap: load >> MSMCSRAM
}
