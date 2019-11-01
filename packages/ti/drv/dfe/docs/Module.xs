/******************************************************************************
 * FILE PURPOSE: DFE LLD DOCS Module specification file.
 ******************************************************************************
 * FILE NAME: Module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the DFE LLD Documentation.
 *
 * Copyright (C) 2012-2013, Texas Instruments, Inc.
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
    Pkg.makePrologue += "release: dfe_lld_document_generation\n";
    Pkg.makePrologue += "dfe_lld_document_generation:\n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";
    Pkg.makePrologue += "\t @echo Generating DFE LLD Documentation\n";
    Pkg.makePrologue += "\t doxygen docs/Doxyfile\n";
    Pkg.makePrologue += "\t @echo DFE LLD Documentation Generated \n";
    Pkg.makePrologue += "\t @echo ----------------------------\n";

    /* Add the documentation file to the package. */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tifooter.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tiheader.htm";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/tilogo.gif";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/titagline.gif";

    /* Add the DFE Design Document to the package */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/DFE_LLD_SDS.pdf";

    /* Add the DFE Software Manifest to the package */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/DFE_LLD_SoftwareManifest.pdf";

    /* Add the HTML documentation to the package */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/doxygen";

    /* Add the release notes to the package */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "docs/ReleaseNotes_DFE_LLD.pdf";

    /* Generate the ECLIPSE Plugin Generation: Only for SETUP Releases. */
    if (lldInstallType == "SETUP")
    {
        Pkg.makePrologue += "all: eclipse_plugin_generation\n";
        Pkg.makePrologue += "eclipse_plugin_generation:\n";
        Pkg.makePrologue += "\t @echo ----------------------------\n";
        Pkg.makePrologue += "\t @echo DFE LLD Eclipse Plugin Generation\n";
        Pkg.makePrologue += "\t xs xdc.tools.eclipsePluginGen -o . -x ./eclipseDocs/sample.xml -c ./eclipseDocs/toc_cdoc_sample.xml\n";
        Pkg.makePrologue += "\t @echo DFE LLD Eclipse Plugin Generated \n";
        Pkg.makePrologue += "\t @echo ----------------------------\n";
    }
}

