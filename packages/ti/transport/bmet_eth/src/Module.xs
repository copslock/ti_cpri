/******************************************************************************
 * FILE PURPOSE: BMET Eth Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the BMET Eth source directory.
 *
 * Copyright (C) 2012, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the BMET Eth LLD Files */
var bmetethFile = [
    "src/bmet_transport.c",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the BMET Eth library
 **************************************************************************/
function modBuild() 
{
    /* Build the libraries for all the targets specified. */
    for (var targets=0; targets < Build.targets.length; targets++)
    {
        var libOptions = {
            incs: bmetethIncPath, 
        };
        
        libUtility.buildLibrary (libOptions, "ti.transport.bmet_eth", Build.targets[targets], bmetethFile);
    }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

