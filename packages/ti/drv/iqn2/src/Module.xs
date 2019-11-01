/******************************************************************************
 * FILE PURPOSE: IQN2 LLD Source Module specification file.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the IQN2 LLD source directory.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the IQN2 LLD Files */
var iqn2lldFiles = [        
    "src/iqn2fl/iqn2fl_init.c",
    "src/iqn2fl/iqn2fl_open.c",
    "src/iqn2fl/iqn2fl_close.c",
    "src/iqn2fl/iqn2fl_hwControl.c",
    "src/iqn2fl/iqn2fl_getHwStatus.c",
    "src/iqn2fl/iqn2fl_hwSetup.c",
	"src/iqn2lld/IQN2_init.c",
	"src/iqn2lld/IQN2_calcParam.c",
	"src/iqn2lld/IQN2_debug.c",
	"src/iqn2lld/IQN2_shutdown.c",
    "src/iqn2lld/IQN2_runtime.c",	
/*
    "src/iqn2_ail.c", - placeholders
    "src/iqn2_at2.c" - placeholders
*/
];

var devicesCCOpt = [ " -DDEVICE_K2L" ];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the IQN2 library
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
            
            libUtility.buildLibrary (libOptions, Pkg.name, Build.targets[targets], iqn2lldFiles);
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

