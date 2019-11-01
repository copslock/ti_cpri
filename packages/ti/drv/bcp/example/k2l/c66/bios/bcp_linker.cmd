SECTIONS
{
    .qmss: load >> MSMCSRAM
    .cppi: load >> MSMCSRAM
    .bcp:  load >> MSMCSRAM
    .testData: load >> L2SRAM
    .init_array: load >> L2SRAM
}
