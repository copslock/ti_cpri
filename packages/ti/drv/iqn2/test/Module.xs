/******************************************************************************
 * FILE PURPOSE: IQN2 LLD unit test files.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for IQN2 LLD test files.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the source files in the test 
 *  directory into the package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the make files to the release package. */
    var testFiles = libUtility.listAllFiles ("makefile_armv7", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .cmd files to the release package. */
    var testFiles = libUtility.listAllFiles (".cmd", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .cfg files to the release package. */
    var testFiles = libUtility.listAllFiles (".cfg", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add the .txt to the package */
    /*var testFiles = libUtility.listAllFiles (".txt", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];*/
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/dualmode/k2l/c66/iqn2DfeDualModeK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/dualmode/k2l/c66/iqn2DualModeK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/k2l/c66/iqn2LteCpriK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/k2l/c66/iqn2LteDfeDualCoreK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/k2l/c66/iqn2LteDfeK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/k2l/c66/iqn2WcdmaCpriK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/k2l/c66/iqn2WcdmaDfeK2LC66TestProject.txt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/k2l/c66/iqn2WcdmaObsaiK2LC66TestProject.txt";
	
	/* Add the documentation file to the package. */
	Pkg.otherFiles[Pkg.otherFiles.length++] = "test/iqn2fl/dfe_init_wcdma.lib";
}
