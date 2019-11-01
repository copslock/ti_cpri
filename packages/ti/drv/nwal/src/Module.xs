/******************************************************************************
 * FILE PURPOSE: NWAL Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the NWAL source directory.
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the NWAL LLD Files */
var nwalFile = [
    "src/nwal.c",
    "src/nwal_init.c",
    "src/nwal_sec.c",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the NWAL library
 **************************************************************************/
function modBuild() 
{
    /* Build the libraries for all the targets specified. */
    for (var targets=0; targets < Build.targets.length; targets++)
    {
        if(socFamily == "KeyStone2")
        {
            /* Build the libraries for all the devices defined in config.bld */
            /* for each(var device in devices) */
            for each(var k = 0; k < devicesK2.length; k++)
            {
                var libOptions = {
                    copts: devicesK2CCOpt[k],
                    incs: lldIncludePath, 
                };
                libUtility.buildLibrary (devicesK2[k],libOptions, "ti.drv.nwal.sa", Build.targets[targets], nwalFile,"yes");
                libUtility.buildLibrary (devicesK2[k],libOptions, "ti.drv.nwal", Build.targets[targets], nwalFile,"no");
            }
        }
        else
        {
            /* Build the libraries for all the devices defined in config.bld */
            /* for each(var device in devices) */
            for each(var k = 0; k < devicesK1.length; k++)
            {
                var libOptions = {
                    copts: devicesK1CCOpt[k],
                    incs: lldIncludePath, 
                };
                libUtility.buildLibrary (devicesK1[k],libOptions, "ti.drv.nwal.sa", Build.targets[targets], nwalFile,"yes");
                libUtility.buildLibrary (devicesK1[k],libOptions, "ti.drv.nwal", Build.targets[targets], nwalFile,"no");
            }
        }
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

