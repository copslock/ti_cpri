SECTIONS
{
	.init_array: load >> L2SRAM
	.csl_vect		> L2SRAM
	.main_mem		> L2SRAM
	.profile_mem	> L2SRAM
}
