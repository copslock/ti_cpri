/******************************************************************************
 * FILE PURPOSE: RM Source module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the RM source directory.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

var rmlldFile = [
    "src/rm.c",
    "src/rm_nameserver.c",
    "src/rm_policy.c",
    "src/rm_services.c",
    "src/rm_transport.c",
    "src/rm_tree.c",
    "src/rm_dtb_util.c",
    "src/rm_allocator.c",
    "util/libfdt/fdt.c",
    "util/libfdt/fdt_ro.c",
    "util/libfdt/fdt_rw.c",
    "util/libfdt/fdt_strerror.c",
    "util/libfdt/fdt_sw.c",
    "util/libfdt/fdt_wip.c",
];

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the RM library
 **************************************************************************/
function modBuild() 
{
    /* Build the device independent libraries for all the targets specified. */
    for (var targets=0; targets < socs["all"].targets.length; targets++)
    {
        var targetFiles = rmlldFile.slice(); /* make copy */
        var libOptions = {
            copts: socs["all"].copts,
            incs:  lldIncludePath, 
        };
        libUtility.buildLibrary ("", "false", libOptions, Pkg.name, socs["all"].targets[targets], targetFiles);
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
            var targetFiles_soc = rmlldFile.slice(); /* make copy */
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
}
