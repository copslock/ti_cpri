/******************************************************************************
 * FILE PURPOSE: BCP Driver Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the BCP Driver.
 *
 * Copyright (C) 2010, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the BCP Files */
var bcpLibFiles = [
    "src/bcp.c",
    "src/bcp_lld.c",
];

var devicesCCOpt = [ " -DDEVICE_K2K" ];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the BCP Driver and to add the core
 *  driver files to the package. 
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
            
            libUtility.buildLibrary (libOptions, Pkg.name, Build.targets[targets], bcpLibFiles);
        }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

