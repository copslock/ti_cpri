/******************************************************************************
 * FILE PURPOSE: TCP3D Driver DOCS Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the TCP3D Driver Documentation .
 *
 * Copyright (C) 2008, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build the TCP3D driver documentation and add it
 *  to the package.
 **************************************************************************/
function modBuild() 
{
    /* Create the actual PROLOGUE Section for the Documentation.*/
    Pkg.makePrologue += "release: tcp3d_document_generation\n";
    Pkg.makePrologue += "tcp3d_document_generation:\n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";
    Pkg.makePrologue += "\t @echo Generating TCP3D Driver Documentation\n";
    Pkg.makePrologue += "\t doxygen docs/TCP3D_DRV_doxconfig\n";
    Pkg.makePrologue += "\t @echo TCP3D Driver Documentation Generated \n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";

    /* Add the documentation file to the package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/TCP3D_DriverSDS.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tifooter.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tiheader.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tilogo.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/titagline.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxy/html";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/TCP3D_LLD_SoftwareManifest.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/ReleaseNotes_TCP3DDriver.pdf";

    if ( tcp3dDriverInstallType == "SETUP" )
    {
        Pkg.makePrologue += "all: eclipse_plugin_generation\n";
        Pkg.makePrologue += "eclipse_plugin_generation:\n";
        Pkg.makePrologue += "\t @echo ----------------------------\n";
        Pkg.makePrologue += "\t @echo TCP3D Eclipse Plugin Generation\n";
        Pkg.makePrologue += "\t xs xdc.tools.eclipsePluginGen -o . -x ./docs/eclipse/sample.xml -c ./docs/eclipse/toc_cdoc_sample.xml\n";
        Pkg.makePrologue += "\t @echo TCP3D Eclipse Plugin Generated \n";
        Pkg.makePrologue += "\t @echo ----------------------------\n";
    }
}

