/******************************************************************************
 * FILE PURPOSE: PA Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the PA source directory.
 *
 * Copyright (C) 2009-2015, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/* List of all the PA LLD Files */

var pa =
{
    /* library name */
    libname: "ti.drv.pa",
    
    /* Souce File List */
    srcFile: ["src/v0/painit.c",
              "src/v0/pa.c",
			  "src/v0/paconst.c"],
              
    /* Library options */
    copts: " -DDEVICE_K2K -DSOC_K2K"
};

var pa2 =
{
    /* library name */
    libname: "ti.drv.pa2",
    
    /* Souce File List */
    srcFile: ["src/v1/painit.c",
              "src/v1/pa.c",
			  "src/v1/paconst.c"],
              
    /* Library options */
    copts: " -DDEVICE_K2L -DSOC_K2L -DNSS_GEN2"
};


var devices = [pa2, pa];


/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the PA library
 **************************************************************************/
function modBuild() 
{
    /* Build the libraries for all the targets specified. */
    for (var dev = 0; dev < devices.length; dev++)
    {
        for (var targets=0; targets < Build.targets.length; targets++)
        {
            var libOptions = {
                copts: devices[dev].copts,
                incs: palldIncPath, 
            };
        
            libUtility.buildLibrary (libOptions, devices[dev].libname, Build.targets[targets], devices[dev].srcFile);
        }
    }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "src");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

