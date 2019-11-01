-c
-heap  0x20000
-stack 0x20000
--args 0x1000
--diag_suppress=10068 /* "no matching section" */
--cinit_compression=off
--retain="*(xdc.meta)"

MEMORY
{
    L2SRAM      (RWX)   : ORIGIN = 0x64800000, LENGTH = 0x00070000  /* 448 KB */
    DDR0        (RWIX)  : ORIGIN = 0xA8000000, LENGTH = 0x08000000  /* 128 MB per core */
    MSMC3       (RWIX)  : ORIGIN = 0x70400000, LENGTH = 0x00080000  /* 512 KB per core */
}

SECTIONS
{
    xdc.meta (COPY):    { } > DDR0
    boot:
    {
      boot.*<boot.oe71>(.text)
    } load > DDR0 ALIGN(0x200000)
    .vecs       >       DDR0 ALIGN(0x400000)
    .text       >       DDR0 ALIGN(0x200000)

    .bss        >       DDR0  /* Zero-initialized data */
    .data       >       DDR0  /* Initialized data */

    .cinit      >       DDR0  /* could be part of const */
    .init_array >       DDR0  /* C++ initializations */
    .stack      >       DDR0
    .args       >       DDR0
    .cio        >       DDR0
    .const      >       DDR0
    .switch     >       DDR0 /* For exception handling */
    .sysmem     >       DDR0 /* heap */

    GROUP:      >  DDR0
    {
        .data.ti_sysbios_family_c7x_Mmu_tableArray     : type=NOINIT
        .data.ti_sysbios_family_c7x_Mmu_tableArraySlot : type=NOINIT
        .data.ti_sysbios_family_c7x_Mmu_level1Table    : type=NOINIT
    }

    .udma_buffer_msmc       >       MSMC3
    .udma_buffer_ddr        >       DDR0
    .udma_buffer_internal   >       L2SRAM
    .udma_buffer_ospi       >       DDR0
}
