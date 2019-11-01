/******************************************************************************
 * FILE PURPOSE: CPPI/QMSS LLD DOCS Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the NWAL Driver Documentation.
 *
 * Copyright (C) 2009, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the documentation
 **************************************************************************/
function modBuild() 
{
    /* Create the actual PROLOGUE Section for the Documentation.*/
    Pkg.makePrologue += "release: nwal_document_generation\n";
    Pkg.makePrologue += "nwal_document_generation:\n";
    Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
    Pkg.makePrologue += "\t @echo Generating NWAL Driver Documentation\n";
    Pkg.makePrologue += "\t doxygen docs/Doxyfile\n";
    Pkg.makePrologue += "\t @echo NWAL Driver Documentation Generated \n";
    Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";

    /* Add the documentation files to the package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/ReleaseNotes_NWAL.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/UserGuide_NWAL.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/NWAL_SoftwareManifest.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen/tifooter.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen/tiheader.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen/tilogo.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen/titagline.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen";

    /* Add all the .pdf files to the release package. */
    var testFiles = libUtility.listAllFiles (".pdf", "docs");
    for (var k = 0 ; k < testFiles.length; k++)
        Pkg.otherFiles[Pkg.otherFiles.length++] = testFiles[k];

    if (lldInstallType == "SETUP")
    {
        /* Generate the ECLIPSE Plugin Generation */
        Pkg.makePrologue += "all: eclipse_plugin_generation\n";
        Pkg.makePrologue += "eclipse_plugin_generation:\n";
        Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
        Pkg.makePrologue += "\t @echo NWAL Driver Eclipse Plugin Generation\n";
        Pkg.makePrologue += "\t xs xdc.tools.eclipsePluginGen -o . -x ./eclipseDocs/sample.xml -c ./eclipseDocs/toc_cdoc_sample.xml\n";
        Pkg.makePrologue += "\t @echo NWAL Driver Eclipse Plugin Generated \n";
        Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
    }
}

