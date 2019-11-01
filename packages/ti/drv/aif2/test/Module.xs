/******************************************************************************
 * FILE PURPOSE: SRIO LLD Test files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for AIF2 Driver Test
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the example project and source files
 *  to the LLD package.
 **************************************************************************/
function modBuild() 
{
    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "test");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "test");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
		
    /* Add all the make files to the release package. */
    var testFiles = libUtility.listAllFiles ("makefile_armv7", "test", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];		

    /* Add all the .cfg files to the release package. */
    var testFiles = libUtility.listAllFiles (".cfg", "test");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .cmd files to the release package. */
    var testFiles = libUtility.listAllFiles (".cmd", "test");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the project definition text files to the package */
	if (aif2LLDPartNumber == "K2")
    {
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteCpriK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteCpriK2HC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteObsaiK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteObsaiK2HC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteSingleToneK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteSingleToneK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaSingleToneK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaSingleToneK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteCheckRfK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteCheckRfK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteTddCheckRfK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteTddCheckRfK2HC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2WcdmaCheckRfK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2WcdmaCheckRfK2HC66TestProject.txt";	
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/generic/aif2GenericK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/generic/aif2GenericK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cprifastcm/aif2CpriFastCMK2KC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cprifastcm/aif2CpriFastCMK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/dualmode/aif2DualModeK2KC66TestProject.txt"; 
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/dualmode/aif2DualModeK2HC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/multicore/aif2LteMultiCoreK2KC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/multicore/aif2LteMultiCoreK2HC66TestProject.txt";		
    } else {
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteCheckRfC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2LteTddCheckRfC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cpricheckrf/aif2WcdmaCheckRfC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/cprifastcm/aif2CpriFastCMC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/dualmode/aif2DualModeC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/generic/aif2GenericC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteSingleToneC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteCpriC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/lte/aif2LteObsaiC66TestProject.txt";		
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaSingleToneC66TestProject.txt";
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/wcdma/aif2WcdmaC66TestProject.txt";	
		Pkg.otherFiles[Pkg.otherFiles.length++] = "test/multicore/aif2LteMultiCoreC66TestProject.txt";
	}
    
    /* Add DSP lib */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/utils/dsplib.ae66";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "test/utils/dsplib.ae66e";
	Pkg.otherFiles[Pkg.otherFiles.length++] = "test/utils/mathUtils.lib";
	Pkg.otherFiles[Pkg.otherFiles.length++] = "test/utils/mathUtilse.lib";
}

