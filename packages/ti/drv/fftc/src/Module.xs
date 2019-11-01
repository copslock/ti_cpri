/******************************************************************************
 * FILE PURPOSE: FFTC Driver Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the FFTC Driver.
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the FFTC Files */
var fftcLibFiles = [
    "src/listlib.c",
    "src/fftc.c",
    "src/fftc_lld.c",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the FFTC Driver and to add the core
 *  driver files to the package. 
 **************************************************************************/
function modBuild() 
{
    for (var device=0; device < devices.length; device++) 
    {
        var targetFiles = fftcLibFiles.slice(); /* make copy */
        /* extract device without c66 suffix */
        var dev=devices[device];
        var idx = dev.indexOf("/");
        dev = dev.substr(0,idx);
        targetFiles.push (deviceConstruct[0]+dev+deviceConstruct[1]);
        /* Build the libraries for all the targets specified. */

        for (var targets=0; targets < Build.targets.length; targets++)
        {
            var libOptions = {
                copts: devicesCCOpt[device],
                incs: lldIncludePath, 
            };
       
            libUtility.buildLibrary (devices[device], libOptions, "ti.drv.fftc", Build.targets[targets], targetFiles);
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

