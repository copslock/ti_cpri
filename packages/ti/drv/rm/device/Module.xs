/******************************************************************************
 * FILE PURPOSE: RM device specific files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for RM device specific files.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
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
        
    /* Add all the .dts files to the release package. */
    var configFiles = libUtility.listAllFiles (".dts", "device", true);
    for (var k = 0 ; k < configFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = configFiles[k];

    /* Add all the .dtb files to the release package. */
    var configFiles = libUtility.listAllFiles (".dtb", "device", true);
    for (var k = 0 ; k < configFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = configFiles[k];        
}
