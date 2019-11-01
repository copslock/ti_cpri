/******************************************************************************
 * FILE PURPOSE: IQN2 LLD include files.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for IQN2 LLD include directory
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the header files in the include 
 *  directory into the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .h files to the release package. */
    var includeFiles = libUtility.listAllFiles (".h", "include", true);
    for (var k = 0 ; k < includeFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = includeFiles[k];
}
