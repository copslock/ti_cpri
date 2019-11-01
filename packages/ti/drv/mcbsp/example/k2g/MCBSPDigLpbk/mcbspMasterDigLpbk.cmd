SECTIONS
{
    .init_array:     load >> L2SRAM
    .mcbsp:          load >> L2SRAM
    .mcbspSharedMem: load >> L2SRAM
     platform_lib:   load >> L2SRAM
     systemHeap:     load >> L2SRAM
}

