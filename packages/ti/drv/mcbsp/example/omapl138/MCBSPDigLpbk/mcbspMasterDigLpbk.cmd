SECTIONS
{
    .init_array:     load >> IRAM
    .mcbsp:          load >> DDR
    .mcbspSharedMem: load >> DDR
     platform_lib:   load >> DDR
     systemHeap:     load >> IRAM
}

