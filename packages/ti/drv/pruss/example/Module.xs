/******************************************************************************
 * FILE PURPOSE: PRUSS LLD example files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for PRUSS LLD example files.
 *
 * Copyright (C) 2009-2016, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

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
    var exampleFiles = libUtility.listAllFiles (".c", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .h files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".h", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .cmd files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".cmd", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .cfg files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".cfg", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the make files to the release package. */
    var exampleFiles = libUtility.listAllFiles ("makefile", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];
    /* Add the .txt to the package */
    var exampleFiles = libUtility.listAllFiles (".txt", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .mk files to the release package. */
    var mkFiles = libUtility.listAllFiles (".mk", "example", true);
    for (var k = 0 ; k < mkFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = mkFiles[k];
    
    /* Add all the .asm files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".asm", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];
    
    /* Add all the .inc files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".inc", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

    /* Add all the .bin files to the release package. */
    var exampleFiles = libUtility.listAllFiles (".bin", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];
}
