SECTIONS
{
    .testcode > MSMCSRAM
    {
        test_main.obj(.text)
        test_osal.obj(.text)
    }

    .qmss: load >> MSMCSRAM
    .cppi: load >> MSMCSRAM
    .fftc: load >> MSMCSRAM
    .init_array: load >> L2SRAM
}
