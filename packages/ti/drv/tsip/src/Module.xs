/******************************************************************************
 * FILE PURPOSE: TSIP Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the TSIP source directory.
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the TSIP LLD Files */
var tsiplldFile = [
    "src/tsip.c",
    "src/tsipinit.c",
    "src/tsipisr.c",
    "src/tsipcsl.c",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the TSIP library
 **************************************************************************/
function modBuild() 
{
    for (var device=0; device < devices.length; device++) 
    {
        /* Build the libraries for all the targets specified. */
        for (var targets=0; targets < Build.targets.length; targets++)
        {
            var libOptions = {
                copts: devicesCCOpt[device],
                incs: tsiplldIncPath,
            };
            libUtility.buildLibrary (devices[device], libOptions, Pkg.name, Build.targets[targets], tsiplldFile);
        }
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

