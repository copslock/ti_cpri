/******************************************************************************
 * FILE PURPOSE: AIF2 Driver Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the AIF2 Driver
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the AIF2 Driver Files */
var aif2SockLibFiles = [
    "src/aif2fl/aif2fl_close.c",
    "src/aif2fl/aif2fl_getHwStatus.c",
    "src/aif2fl/aif2fl_hwSetup.c",
    "src/aif2fl/aif2fl_open.c",
    "src/aif2fl/aif2fl_getBaseAddress.c",
    "src/aif2fl/aif2fl_hwControl.c",
	"src/aif2fl/aif2fl_init.c",
	"src/aif2fl/aif2fl_reset.c",
    "src/aif2lld/AIF_calcParam.c",
	"src/aif2lld/AIF_init.c",
	"src/aif2lld/AIF_init_dat.c",
	"src/aif2lld/AIF_shutdown.c",
    "src/aif2lld/AIF_fsync.c",
	"src/aif2lld/AIF_debug.c",
	"src/aif2lld/AIF_cfg.c",
	"src/aif2lld/AIF_hibernation.c"
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the AIF2 LLD Driver and to add the core
 *  driver files to the package. 
 **************************************************************************/
function modBuild() 
{
    /* Build the libraries for all the targets specified. */
    for (var targets=0; targets < Build.targets.length; targets++)
    {
            var libOptions = {
            };
            
            libUtility.buildLibrary (libOptions, Pkg.name, Build.targets[targets], aif2SockLibFiles);
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

