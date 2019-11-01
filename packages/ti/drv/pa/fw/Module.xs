/******************************************************************************
 * FILE PURPOSE: PA FW module specification file.
 ******************************************************************************
 * FILE NAME: module.xs
 *
 * DESCRIPTION: 
 *  This file contains the module specification for the PA fw directory.
 *
 * Copyright (C) 2009-2013, Texas Instruments, Inc.
 *****************************************************************************/

/* Load the library utility. */
var libUtility = xdc.loadCapsule ("../build/buildlib.xs");

/**************************************************************************
 * FUNCTION NAME : modBuild
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to build all the components of the PA library
 **************************************************************************/
function modBuild() 
{

    /* Assemble the firmware */
  /*
    Pkg.makePrologue += ".PHONY: firmware\n\n";
    Pkg.makePrologue += "release: firmware\n";
    Pkg.makePrologue += "firmware: \n";
    
    Pkg.makePrologue += "\t $(ECHO) making the firmware for PASS version 0\n";
    Pkg.makePrologue += "\t cd fw/v0; pasm_dos -EBldXc -Cc1_0 classify1_0.p\n";
    Pkg.makePrologue += "\t cd fw/v0; pasm_dos -EBldXc -Cc1_1 classify1_1.p\n";
    Pkg.makePrologue += "\t cd fw/v0; pasm_dos -EBldXc -Cc1_2 classify1_2.p\n";
    Pkg.makePrologue += "\t cd fw/v0; pasm_dos -EBldXc -Cc2 classify2.p\n";
    Pkg.makePrologue += "\t cd fw/v0; pasm_dos -EBldXc -Cm pam.p\n";
  */
    /* format the firmware assembler output */
  /*
    Pkg.makePrologue += "\t cd fw/v0; cat classify1_0_desc.c > classify1_0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cat classify1_0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> classify1_0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; $(ECHO) \"const int c1_0Size = sizeof(c1_0);\" >> classify1_0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cp classify1_0_bin.c ../. \n";

    Pkg.makePrologue += "\t cd fw/v0; cat classify1_1_desc.c > classify1_1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cat classify1_1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> classify1_1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; $(ECHO) \"const int c1_1Size = sizeof(c1_1);\" >> classify1_1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cp classify1_1_bin.c ../. \n";

    Pkg.makePrologue += "\t cd fw/v0; cat classify1_2_desc.c > classify1_2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cat classify1_2_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> classify1_2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; $(ECHO) \"const int c1_2Size = sizeof(c1_2);\" >> classify1_2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cp classify1_2_bin.c ../. \n";

    Pkg.makePrologue += "\t cd fw/v0; cat classify2_desc.c > classify2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cat classify2_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> classify2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; $(ECHO) \"const int c2Size = sizeof(c2);\" >> classify2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cp classify2_bin.c ../. \n";

    Pkg.makePrologue += "\t cd fw/v0; cat pam_desc.c > pam_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cat pam_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pam_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; $(ECHO) \"const int mSize = sizeof(m);\" >> pam_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v0; cp pam_bin.c ../. \n";
  */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/pafw.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_2_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify2_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/pam_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/pafw.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify1_2.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/classify2.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v0/pam.bib";
    
    /* add for backward compatibility */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/classify1_0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/classify1_1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/classify1_2_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/classify2_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/pam_bin.c";

    /* Delete temporary files */
  /*  
    Pkg.makePrologue += "\t cd fw/v0; rm -f classify1_0_bin.h classify1_1_bin.h classify1_2_bin.h classify2_bin.h pam_bin.h\n";
    
    Pkg.makePrologue += "\t $(ECHO) making the firmware for PASS version 1\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin0_pdsp0 in0_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin0_pdsp1 in0_pdsp1.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin1_pdsp0 in1_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin1_pdsp1 in1_pdsp1.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin2_pdsp0 in2_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin3_pdsp0 in3_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin4_pdsp0 in4_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cin4_pdsp1 in4_pdsp1.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cpost_pdsp0 post_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Cpost_pdsp1 post_pdsp1.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Ceg0_pdsp0 eg0_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Ceg0_pdsp1 eg0_pdsp1.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Ceg0_pdsp2 eg0_pdsp2.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Ceg1_pdsp0 eg1_pdsp0.p\n";
    Pkg.makePrologue += "\t cd fw/v1; pasm_dos -V3EBldc -Ceg2_pdsp0 eg2_pdsp0.p\n";
  */
    /* format the firmware assembler output */
  /*
    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in0_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in0_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in0_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in0_pdsp0Size = sizeof(in0_pdsp0);\" >> pa2_in0_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in0_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in0_pdsp1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in0_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in0_pdsp1Size = sizeof(in0_pdsp1);\" >> pa2_in0_pdsp1_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in1_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in1_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in1_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in1_pdsp0Size = sizeof(in1_pdsp0);\" >> pa2_in1_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in1_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in1_pdsp1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in1_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in1_pdsp1Size = sizeof(in1_pdsp1);\" >> pa2_in1_pdsp1_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in2_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in2_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in2_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in2_pdsp0Size = sizeof(in2_pdsp0);\" >> pa2_in2_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in3_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in3_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in3_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in3_pdsp0Size = sizeof(in3_pdsp0);\" >> pa2_in3_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in4_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in4_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in4_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in4_pdsp0Size = sizeof(in4_pdsp0);\" >> pa2_in4_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_in4_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat in4_pdsp1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_in4_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int in4_pdsp1Size = sizeof(in4_pdsp1);\" >> pa2_in4_pdsp1_bin.c\n";
    
    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_post_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat post_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_post_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int post_pdsp0Size = sizeof(post_pdsp0);\" >> pa2_post_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_post_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat post_pdsp1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_post_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int post_pdsp1Size = sizeof(post_pdsp1);\" >> pa2_post_pdsp1_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_eg0_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat eg0_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_eg0_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int eg0_pdsp0Size = sizeof(eg0_pdsp0);\" >> pa2_eg0_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_eg0_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat eg0_pdsp1_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_eg0_pdsp1_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int eg0_pdsp1Size = sizeof(eg0_pdsp1);\" >> pa2_eg0_pdsp1_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_eg0_pdsp2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat eg0_pdsp2_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_eg0_pdsp2_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int eg0_pdsp2Size = sizeof(eg0_pdsp2);\" >> pa2_eg0_pdsp2_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_eg1_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat eg1_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_eg1_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int eg1_pdsp0Size = sizeof(eg1_pdsp0);\" >> pa2_eg1_pdsp0_bin.c\n";

    Pkg.makePrologue += "\t cd fw/v1; cat pa2_pdsp_desc.c > pa2_eg2_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; cat eg2_pdsp0_bin.h | sed -e s/\"unsigned int\"/\"uint32_t\"/g >> pa2_eg2_pdsp0_bin.c\n";
    Pkg.makePrologue += "\t cd fw/v1; $(ECHO) \"const int eg2_pdsp0Size = sizeof(eg2_pdsp0);\" >> pa2_eg2_pdsp0_bin.c\n";
 */
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pafw.h";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in0_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in0_pdsp1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in1_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in1_pdsp1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in2_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in3_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in4_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_in4_pdsp1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_post_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_post_pdsp1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_eg0_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_eg0_pdsp1_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_eg0_pdsp2_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_eg1_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/pa2_eg2_pdsp0_bin.c";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in0_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in0_pdsp1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in1_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in1_pdsp1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in2_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in3_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in4_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/in4_pdsp1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/post_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/post_pdsp1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/eg0_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/eg0_pdsp1.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/eg0_pdsp2.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/eg1_pdsp0.bib";
    Pkg.otherFiles[Pkg.otherFiles.length++] = "fw/v1/eg2_pdsp0.bib";

    /* Delete temporary files */
  /*  
    Pkg.makePrologue += "\t cd fw/v1; rm -f in0_pdsp0_bin.h in0_pdsp1_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f in1_pdsp0_bin.h in1_pdsp1_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f in2_pdsp0_bin.h in3_pdsp0_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f in4_pdsp0_bin.h in4_pdsp1_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f post_pdsp0_bin.h post_pdsp1_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f eg0_pdsp0_bin.h eg0_pdsp1_bin.h eg0_pdsp2_bin.h\n";
    Pkg.makePrologue += "\t cd fw/v1; rm -f eg1_pdsp0_bin.h eg2_pdsp0_bin.h\n";
  */
}

