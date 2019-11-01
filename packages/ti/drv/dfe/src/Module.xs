/******************************************************************************
 * FILE PURPOSE: DFE LLD Source Module specification file.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the DFE LLD source directory.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the DFE LLD Files */
var dfelldFiles = [
	"src/dfefl/dfe_fl_Close.c",
	"src/dfefl/dfe_fl_Init.c",
	"src/dfefl/dfe_fl_Open.c",
	"src/dfefl/dfe_fl_autocpClose.c",
	"src/dfefl/dfe_fl_autocppGetHwStatus.c",
	"src/dfefl/dfe_fl_autocppHwControl.c",
	"src/dfefl/dfe_fl_autocppOpen.c",
	"src/dfefl/dfe_fl_bbClose.c",
	"src/dfefl/dfe_fl_bbGetHwStatus.c",
	"src/dfefl/dfe_fl_bbHwControl.c",
	"src/dfefl/dfe_fl_bbOpen.c",
	"src/dfefl/dfe_fl_cbClose.c",
	"src/dfefl/dfe_fl_cbGetHwStatus.c",
	"src/dfefl/dfe_fl_cbHwControl.c",
	"src/dfefl/dfe_fl_cbOpen.c",
	"src/dfefl/dfe_fl_cdfrClose.c",
	"src/dfefl/dfe_fl_cdfrGetHwStatus.c",
	"src/dfefl/dfe_fl_cdfrHwControl.c",
	"src/dfefl/dfe_fl_cdfrOpen.c",
	"src/dfefl/dfe_fl_cfrClose.c",
	"src/dfefl/dfe_fl_cfrGetHwStatus.c",
	"src/dfefl/dfe_fl_cfrHwControl.c",
	"src/dfefl/dfe_fl_cfrOpen.c",
	"src/dfefl/dfe_fl_cppDescripClose.c",
	"src/dfefl/dfe_fl_cppDescripControl.c",
	"src/dfefl/dfe_fl_cppDescripOpen.c",
	"src/dfefl/dfe_fl_cppDescripStatus.c",
	"src/dfefl/dfe_fl_cppDmaClose.c",
	"src/dfefl/dfe_fl_cppDmaControl.c",
	"src/dfefl/dfe_fl_cppDmaOpen.c",
	"src/dfefl/dfe_fl_cppDmaStatus.c",
	"src/dfefl/dfe_fl_dducClose.c",
	"src/dfefl/dfe_fl_dducGetHwStatus.c",
	"src/dfefl/dfe_fl_dducHwControl.c",
	"src/dfefl/dfe_fl_dducOpen.c",
	"src/dfefl/dfe_fl_dpdClose.c",
	"src/dfefl/dfe_fl_dpdGetHwStatus.c",
	"src/dfefl/dfe_fl_dpdHwControl.c",
	"src/dfefl/dfe_fl_dpdOpen.c",
	"src/dfefl/dfe_fl_dpdaClose.c",
	"src/dfefl/dfe_fl_dpdaGetHwStatus.c",
	"src/dfefl/dfe_fl_dpdaHwControl.c",
	"src/dfefl/dfe_fl_dpdaOpen.c",
	"src/dfefl/dfe_fl_fbClose.c",
	"src/dfefl/dfe_fl_fbGetHwStatus.c",
	"src/dfefl/dfe_fl_fbHwControl.c",
	"src/dfefl/dfe_fl_fbOpen.c",
	"src/dfefl/dfe_fl_jesdClose.c",
	"src/dfefl/dfe_fl_jesdGetHwStatus.c",
	"src/dfefl/dfe_fl_jesdHwControl.c",
	"src/dfefl/dfe_fl_jesdOpen.c",
	"src/dfefl/dfe_fl_miscClose.c",
	"src/dfefl/dfe_fl_miscGetHwStatus.c",
	"src/dfefl/dfe_fl_miscHwControl.c",
	"src/dfefl/dfe_fl_miscOpen.c",
	"src/dfefl/dfe_fl_rxClose.c",
	"src/dfefl/dfe_fl_rxGetHwStatus.c",
	"src/dfefl/dfe_fl_rxHwControl.c",
	"src/dfefl/dfe_fl_rxOpen.c",
	"src/dfefl/dfe_fl_summerClose.c",
	"src/dfefl/dfe_fl_summerGetHwStatus.c",
	"src/dfefl/dfe_fl_summerHwControl.c",
	"src/dfefl/dfe_fl_summerOpen.c",
	"src/dfefl/dfe_fl_txClose.c",
	"src/dfefl/dfe_fl_txGetHwStatus.c",
	"src/dfefl/dfe_fl_txHwControl.c",
	"src/dfefl/dfe_fl_txOpen.c",
	"src/dfelld/DFE_bb.c",
	"src/dfelld/DFE_cb.c",
	"src/dfelld/DFE_cfr.c",
	"src/dfelld/DFE_dduc.c",
	"src/dfelld/DFE_dpd.c",
    "src/dfelld/DFE_dpda.c",	
	"src/dfelld/DFE_device.c",
	"src/dfelld/DFE_excep.c",
	"src/dfelld/DFE_fb.c",
	"src/dfelld/DFE_jesd.c",
	"src/dfelld/DFE_misc.c",
	"src/dfelld/DFE_open.c",
	"src/dfelld/DFE_rx.c",
	"src/dfelld/DFE_summer.c",
	"src/dfelld/DFE_sync.c",
	"src/dfelld/DFE_tx.c",
];

var devicesCCOpt = [ " -DDEVICE_K2L" ];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the DFE library
 **************************************************************************/
function modBuild() 
{
    /* Build the libraries for all the targets specified. */
    for (var targets=0; targets < Build.targets.length; targets++)
    {
            var libOptions = {
                copts: devicesCCOpt,
                incs: lldIncludePath, 
            };
            
            libUtility.buildLibrary (libOptions, Pkg.name, Build.targets[targets], dfelldFiles);
    }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "src", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

