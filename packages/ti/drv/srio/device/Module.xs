/******************************************************************************
 * FILE PURPOSE: SRIO LLD device specific files.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for SRIO LLD device specific files.
 *
 * Copyright (C) 2011, Texas Instruments, Inc.
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
}
