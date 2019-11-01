SECTIONS
{
    .qmss: load >> MSMCSRAM
    .cppi: load >> MSMCSRAM
    .fftc: load >> MSMCSRAM
    .init_array: load >> L2SRAM
}
