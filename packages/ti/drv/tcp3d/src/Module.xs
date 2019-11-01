/******************************************************************************
 * FILE PURPOSE: TCP3D Driver Source Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the TCP3D Driver
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the TCP3D Files for library building */
var tcp3dLibFiles = [
    "src/tcp3d_drv.c",
    "src/tcp3d_reg.c",
    "src/tcp3d_utils.c",
    "src/tcp3d_betaState.c",
];

/* Other files for packaging */
var tcp3dOtherFiles = [
    "src/tcp3d_drv_priv.h",
    "src/tcp3d_utils.h",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the TCP3D Driver and to add the core
 *  driver files to the package. 
 **************************************************************************/
function modBuild() 
{
    for (var device=0; device < devices.length; device++) 
    {
        /* Build the libraries for all the targets specified. */
        for (var targets=0; targets < Build.targets.length; targets++)
        {
            var libOptions = { 
                incs: tcp3dIncludePath, 
                copts: devicesCCOpt[device],
            };
                
            libUtility.buildLibrary (devices[device], libOptions, Pkg.name, Build.targets[targets], tcp3dLibFiles);
        }
    }

    /* Add all the .c files to the release package. */
    for (var k = 0 ; k < tcp3dLibFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = tcp3dLibFiles[k];

    /* Add all the .h files to the release package. */
    for (var k = 0 ; k < tcp3dOtherFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = tcp3dOtherFiles[k];
}

