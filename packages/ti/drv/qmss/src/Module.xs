/******************************************************************************
 * FILE PURPOSE: QMSS Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the QMSS source directory.
 *
 * Copyright (C) 2009-2014, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

var qmsslldFile = [
    "src/qmss_drv.c",
    "src/qmss_acc.c",
    "src/qmss_qos.c",
    "src/qmss_qosSched.c",
    "src/qmss_srioRtr.c"
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the QMSS library
 **************************************************************************/
function modBuild() 
{
    /* Build the device independent libraries for all the targets specified. */
    for (var targets=0; targets < socs["all"].targets.length; targets++)
    {
        var targetFiles = qmsslldFile.slice(); /* make copy */
        var libOptions = {
            copts: socs["all"].copts,
            incs:  lldIncludePath, 
        };
        libUtility.buildLibrary ("",  "false", libOptions, Pkg.name, socs["all"].targets[targets], targetFiles);
    }

    /* Build library targets for device dependent SoCs */
    for (var soc=0; soc < soc_names.length; soc++) 
    {
        var dev = socs[soc_names[soc]];
        
        /* do not proceed if this SoC is not configured to be built */
        if (dev.build == "false")
           continue;

        if (dev.socDevLib == "true")
        { 
            var targetFiles_soc = qmsslldFile.slice(); /* make copy */
            targetFiles_soc.push (deviceConstruct[0]+soc_names[soc]+deviceConstruct[1]);
            /* Build the libraries for all the targets specified. */
            for (var targets=0; targets < dev.targets.length; targets++)
            {
                var libOptions = {
                    copts: dev.copts,
                    incs:  lldIncludePath, 
                };
                libUtility.buildLibrary (soc_names[soc], "true", libOptions, Pkg.name, dev.targets[targets], targetFiles_soc);
            }
         }
    }

    /* Add all the .c files to the release package. */
    var testFiles = libUtility.listAllFiles (".c", "src", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    /* Add all the .h files to the release package. */
    var testFiles = libUtility.listAllFiles (".h", "src", true);
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];
}

