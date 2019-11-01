-stack  0x4000
-heap   0x4000

MEMORY
{
    L2SRAM      (RWX)   : ORIGIN = 0x00800000, LENGTH = 0x00020000  /* 128 KB per core */
    DDR0        (RWIX)  : ORIGIN = 0xD8000000, LENGTH = 0x08000000  /* 128 MB per core */
    MSMC3       (RWIX)  : ORIGIN = 0x70480000, LENGTH = 0x00080000  /* 512 KB per core */
}

/* Set L1D, L1P and L2 Cache Sizes */
ti_sysbios_family_c66_Cache_l1dSize = 32768;
ti_sysbios_family_c66_Cache_l1pSize = 32768;
ti_sysbios_family_c66_Cache_l2Size = 32768;

SECTIONS
{
    .vecs:              load >  DDR0 ALIGN(0x10000)
    .text:_c_int00      load >  DDR0 ALIGN(0x10000)
    .text:              load >  DDR0
    .stack:             load >  DDR0
    GROUP:              load >  DDR0
    {
        .bss:
        .neardata:
        .rodata:
    }
    .cio:               load >  DDR0
    .const:             load >  DDR0
    .data:              load >  DDR0
    .switch:            load >  DDR0
    .sysmem:            load >  DDR0
    .far:               load >  DDR0
    .args:              load >  DDR0

    /* COFF sections */
    .pinit:             load >  DDR0
    .cinit:             load >  DDR0

    /* EABI sections */
    .binit:             load >  DDR0
    .init_array:        load >  DDR0
    .fardata:           load >  DDR0
    .c6xabi.exidx:      load >  DDR0
    .c6xabi.extab:      load >  DDR0

    .csl_vect:          load >  L2SRAM

    .udma_buffer_msmc        >  MSMC3
    .udma_buffer_ddr         >  DDR0
    .udma_buffer_internal    >  L2SRAM
    .udma_buffer_ospi        >  DDR0
}
