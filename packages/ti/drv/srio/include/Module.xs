/******************************************************************************
 * FILE PURPOSE: SRIO LLD Include Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the SRIO Driver Include
 *  Directory.
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the internal header files of the SRIO
 *  LLD driver to the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "include");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

