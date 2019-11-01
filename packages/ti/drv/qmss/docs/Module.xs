/******************************************************************************
 * FILE PURPOSE: CPPI/QMSS LLD DOCS Module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the QMSS LLD Documentation.
 *
 * Copyright (C) 2011, Texas Instruments, Inc.
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
    Pkg.makePrologue += ".PHONY: qmss_lld_document_generation\n";
    Pkg.makePrologue += "release: qmss_lld_document_generation\n";
    Pkg.makePrologue += "docs/doxygen/html/index.html: qmss_lld_document_generation\n";
    Pkg.makePrologue += "qmss_lld_document_generation::\n";
    Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
    Pkg.makePrologue += "\t @echo Generating QMSS LLD Documentation\n";
    Pkg.makePrologue += "\t doxygen docs/Doxyfile\n";
    Pkg.makePrologue += "\t @echo QMSS LLD Documentation Generated \n";
    Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";

    /* Add the documentation file to the package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tifooter.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tiheader.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tilogo.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/titagline.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/CPPI_QMSS_LLD_SDS.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/QMSS_LLD_SoftwareManifest.html";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/ReleaseNotes_QMSS_LLD.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/firmware/qos.pdf";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/firmware/qos_sched,qos_sched_drop_sched,qos_sched_wide.pdf";

    if (lldInstallType == "SETUP")
    {
        /* Generate the ECLIPSE Plugin Generation */
        Pkg.makePrologue += "all: eclipse_plugin_generation\n";
        Pkg.makePrologue += "eclipse_plugin_generation:\n";
        Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
        Pkg.makePrologue += "\t @echo QMSS LLD Eclipse Plugin Generation\n";
        Pkg.makePrologue += "\t xs xdc.tools.eclipsePluginGen -o . -x ./eclipseDocs/sample.xml -c ./eclipseDocs/toc_cdoc_sample.xml\n";
        Pkg.makePrologue += "\t @echo QMSS LLD Eclipse Plugin Generated \n";
        Pkg.makePrologue += "\t @echo -------------------------------------------------------\n";
    }   
}

