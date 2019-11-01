###########################################################################
#                                                                         #
#   Copyright (C) 2012-2017 Texas Instruments Incorporated                #
#     http://www.ti.com/                                                  #
#                                                                         #
###########################################################################



#############################################################################
#                                                                           #
#  Redistribution and use in source and binary forms, with or without       #
#  modification, are permitted provided that the following conditions       #
#  are met:                                                                 #
#                                                                           #
#    Redistributions of source code must retain the above copyright         #
#    notice, this list of conditions and the following disclaimer.          #
#                                                                           #
#    Redistributions in binary form must reproduce the above copyright      #
#    notice, this list of conditions and the following disclaimer in the    #
#    documentation and/or other materials provided with the                 #
#    distribution.                                                          #
#                                                                           #
#    Neither the name of Texas Instruments Incorporated nor the names of    #
#    its contributors may be used to endorse or promote products derived    #
#    from this software without specific prior written permission.          #
#                                                                           #
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS      #
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT        #
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    #
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     #
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    #
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         #
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    #
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    #
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      #
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    #
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     #
#                                                                           #
#############################################################################
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
DSP_LNKFLAGS=-z -a -usetEmifA45Div -usetEmifB45Div -w -x

ARM_OBJECTS:=$(patsubst %.c,./arm_obj/%.obj,$(SOURCES))
ARM_EXECUTABLE:=../ARM_$(PROGRAM)_$(DEVSTRING).out
ARM_LINKERSCRIPT:=ARM_AISExtra.lds

ARM_CFLAGS= -I../../../../../Common/include -I../../../../Common/include -I../../../Common/include -I../include -c -Os -Wall -ffreestanding -o $@
ARM_LNKFLAGS=-Wl,-T$(ARM_LINKERSCRIPT) -nostdlib -u setEmifA45Div -u setEmifB45Div 


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

