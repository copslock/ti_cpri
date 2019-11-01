/******************************************************************************
 * FILE PURPOSE: TCP3D Test files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for TCP3D Driver Unit Test
 *  Files
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

var otherFiles = [
    "test/gen_test_vectors/msvc/GenTestVectors.dsp",
    "test/gen_test_vectors/msvc/Debug/GenTestVectors.exe",
    "test/gen_test_vectors/simulator/debug/Test_c_model.exe",
    "test/gen_test_vectors/cleanTestVect.bat",
    "test/gen_test_vectors/genTestVect.bat",
    "test/gen_test_vectors/genTestVect_200.bat",
    "test/gen_test_vectors/GenConfig_wimax.m",
    "test/gen_test_vectors/GenConfig_wcdma.m",
    "test/gen_test_vectors/GenConfig_lte.m",
    "test/gen_test_vectors/LTE_200/GenConfig_lte.m",
    "test/gen_test_vectors/WCDMA_200/GenConfig_wcdma.m",
    "test/gen_test_vectors/WIMAX_200/GenConfig_wimax.m",
];

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
    var testFiles = libUtility.listAllFiles ("makefile", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .cfg files to the release package. */
    var testFiles = libUtility.listAllFiles (".cfg", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .cmd files to the release package. */
    var testFiles = libUtility.listAllFiles (".cmd", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .txt files to the release package. */
    var testFiles = libUtility.listAllFiles (".txt", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .ini files to the release package. */
    var testFiles = libUtility.listAllFiles (".ini", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all other files to the release package. */
    for (var k = 0 ; k < otherFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = otherFiles[k];
}

