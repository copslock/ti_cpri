#############################################################
# Makefile for TI AISExtra project.                         #
#   Generates the COFF AISExtra file which is used to       #
#   supplement the AIS ROM bootloader                       #
#############################################################
#

# CROSSCOMPILE definition for entire tree
include ../../../../../Common/build.mak
include ../../../../device.mak

PROGRAM:=nand_ecc_patch

DSP_CC=$(DSP_CROSSCOMPILE)cl6x
ARM_CC=$(TI_ARM_CROSSCOMPILE)gcc

SOURCES=nand_ecc_patch.c

DSP_OBJECTS:=$(patsubst %.c,./dsp_obj/%.obj,$(SOURCES))
DSP_EXECUTABLE:=../DSP_$(PROGRAM)_$(DEVSTRING).out
DSP_LINKERSCRIPT:=DSP.cmd

DSP_CFLAGS=-O=2 -c --obj_directory=./dsp_obj -I=$(DSP_INC_PATH) -I=../include -ms3 -mo -mv6740 --symdebug:none
DSP_LNKFLAGS=-z -a -uNAND_ECC_info -uNAND_ECC_patchApply -w -x -l$(DSP_LIB_PATH)/rts6740_elf.lib


ARM_OBJECTS:=$(patsubst %.c,./arm_obj/%.obj,$(SOURCES))
ARM_EXECUTABLE:=../ARM_$(PROGRAM)_$(DEVSTRING).out
ARM_LINKERSCRIPT:=ARM.lds

ARM_CFLAGS= -I=$(TI_ARM_INC_PATH) -I=../include -c -Os -Wall -ffreestanding -o $@
ARM_LNKFLAGS= -uNAND_ECC_info -nostdlib -uNAND_ECC_patchApply


all: $(DSP_EXECUTABLE) $(ARM_EXECUTABLE) $(DSP_OBJECTS) $(ARM_OBJECTS) 

.PHONY : clean objclean
clean:
		-rm -f -v $(DSP_OBJECTS) $(ARM_OBJECTS) $(DSP_EXECUTABLE) $(ARM_EXECUTABLE)
		
objclean:
		-rm -f -v $(DSP_OBJECTS) $(ARM_OBJECTS)

$(DSP_EXECUTABLE): $(DSP_OBJECTS) $(DSP_LINKERSCRIPT) 
		$(DSP_CC) $(DSP_LNKFLAGS) -o=$@ $(DSP_LINKERSCRIPT) $(DSP_OBJECTS)
		
./dsp_obj/%.obj : %.c
		@mkdir -p dsp_obj  
		$(DSP_CC) $(DSP_CFLAGS) $<
    
$(ARM_EXECUTABLE): $(ARM_OBJECTS) $(ARM_LINKERSCRIPT)
		$(ARM_CC) $(ARM_LNKFLAGS) -T$(ARM_LINKERSCRIPT) $(ARM_OBJECTS) -o $@ 

./arm_obj/%.obj : %.c
		@mkdir -p arm_obj
		$(ARM_CC) $(ARM_CFLAGS) $<

vpath %.c ../../../../../Common/src:../../../../Common/src:../../../Common/src:../../Common/src:../src        
vpath %.h ../../../../../Common/include:../../../../Common/include:../../../Common/include:../../Common/include:../include       

