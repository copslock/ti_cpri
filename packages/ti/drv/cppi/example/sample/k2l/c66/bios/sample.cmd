SECTIONS
{
    .init_array: load >> L2SRAM
    .sharedGRL: load >> L2SRAM
    .sharedPolicy: load >> L2SRAM
    .qmss: load >> MSMCSRAM
    .cppi: load >> MSMCSRAM
    .rm: load >> MSMCSRAM    
}
