/******************************************************************************
 * FILE PURPOSE: QMSS PDSP firmware Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the QMSS PDSP firmware files.
 *
 * Copyright (C) 2009-2012, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of PDSP firmware
 **************************************************************************/
function modBuild() 
{
    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "firmware", true);
    testFiles = testFiles.concat (libUtility.listAllFiles (".bib", "firmware", true));
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

