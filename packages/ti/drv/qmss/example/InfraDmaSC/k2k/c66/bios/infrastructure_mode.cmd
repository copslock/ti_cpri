SECTIONS
{
    .init_array: load >> L2SRAM
    .qmss: load >> L2SRAM
    .cppi: load >> L2SRAM
    cppiLocalHeap: load >> L2SRAM
}
