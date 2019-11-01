/******************************************************************************
 * FILE PURPOSE: MCBSP LLD device files.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for MCBSP LLD device files.
 *
 * Copyright (C) 2012, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the source files in the device 
 *  directory into the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .c files to the release package. */
    var configFiles = libUtility.listAllFiles (".c", "device", true);
    for (var k = 0 ; k < configFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = configFiles[k];

    /* Add all the .h files to the release package. */
    var configFiles = libUtility.listAllFiles (".h", "device", true);
    for (var k = 0 ; k < configFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = configFiles[k];
}
