/******************************************************************************
 * FILE PURPOSE: TSIP LLD example files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for TSIP LLD example files.
 *
 * Copyright (C) 2010, Texas Instruments, Inc.
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

   /* Add all txt files */
    var exampleFiles = libUtility.listAllFiles (".txt", "example", true);
    for (var k = 0 ; k < exampleFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = exampleFiles[k];

}
