/******************************************************************************
 * FILE PURPOSE: TCP3D Example files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for TCP3D Driver Unit Example
 *  Files
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

var otherFiles = [
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the source files in the example 
 *  directory into the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .c files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".c", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .h files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".h", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .cfg files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".cfg", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .cmd files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".cmd", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .txt files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".txt", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .txt files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".ini", "example");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .txt files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".dat", "example/testvectors");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .txt files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".bin", "example/testvectors");
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

	/* Add all other files to the release package. */
    for (var k = 0 ; k < otherFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = otherFiles[k];
}
