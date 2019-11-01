/******************************************************************************
 * FILE PURPOSE: RM include files.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for RM util directory
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add all the header files in the include 
 *  directory into the package.
 **************************************************************************/
function modBuild() 
{
    /* Add tree algorithm files to the release package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/tree.h";

    /* Add libfdt library files to the relesae packages */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt_ro.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt_rw.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt_strerror.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt_sw.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/fdt_wip.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/libfdt.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/libfdt_env.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/libfdt_internal.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/Makefile.libfdt";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/TODO";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/libfdt/version.lds";

    /* Add the DTB to C-character array conversion utility */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "util/cify.sh";
}

