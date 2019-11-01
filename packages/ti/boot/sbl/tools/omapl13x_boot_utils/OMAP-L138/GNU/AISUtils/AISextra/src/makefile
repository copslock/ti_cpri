#############################################################
# Makefile for TI AISExtra project.                         #
#   Generates the COFF AISExtra file which is used to       #
#   supplement the AIS ROM bootloader                       #
#############################################################
#

# CROSSCOMPILE definition for entire tree
include ../../../../../Common/build.mak
include ../../../../device.mak

PROGRAM:=AISExtra

DSP_CC=$(DSP_CROSSCOMPILE)cl6x
ARM_CC=$(CROSSCOMPILE)gcc

SOURCES=aisextra.c

DSP_OBJECTS:=$(patsubst %.c,./dsp_obj/%.obj,$(SOURCES))
DSP_EXECUTABLE:=../DSP_$(PROGRAM)_$(DEVSTRING).out
DSP_LINKERSCRIPT:=DSP_AISExtra.cmd

DSP_CFLAGS=-O=2 -c --obj_directory=./dsp_obj -I=../../../../../Common/include -I=../../../../Common/include -I=../../../Common/include -I=../include -ms3 -mo -mv6740 --symdebug:none
DSP_LNKFLAGS=-z -a -uPatchDDRConfig -w -x

ARM_OBJECTS:=$(patsubst %.c,./arm_obj/%.obj,$(SOURCES))
ARM_EXECUTABLE:=../ARM_$(PROGRAM)_$(DEVSTRING).out
ARM_LINKERSCRIPT:=ARM_AISExtra.lds

ARM_CFLAGS= -I../../../../../Common/include -I../../../../Common/include -I../../../Common/include -I../include -c -Os -Wall -ffreestanding -o $@
ARM_LNKFLAGS=-Wl,-T$(ARM_LINKERSCRIPT) -nostdlib -u PatchDDRConfig


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
		$(ARM_CC) $(ARM_LNKFLAGS) $(ARM_OBJECTS) -o $@ 

./arm_obj/%.obj : %.c
		@mkdir -p arm_obj
		$(ARM_CC) $(ARM_CFLAGS) $<

vpath %.c ../../../../../Common/src:../../../../Common/src:../../../Common/src:../../Common/src:../src        
vpath %.h ../../../../../Common/include:../../../../Common/include:../../../Common/include:../../Common/include:../include       

