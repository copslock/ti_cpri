###########################################################################
#                                                                         #
#   Copyright (C) 2012 Texas Instruments Incorporated                     #
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
# Makefile for TI Non-Secure HexAIS project.                #
#############################################################
# Auth: Daniel Allred
#	    
include ../../../device.mak

PROGRAM:=HexAIS

MONOCOMPILE=gmcs
DOTNETCOMPILE=csc
LOCALSOURCES=$(PROGRAM).cs
REMOTESOURCES=device_name.cs AISGen_$(DEVSTRING).cs Ini.cs CRC.cs Endian.cs $(PROGRAM)_version.cs \
              EmbeddedFileIO.cs Debug.cs HexConv.cs AIS.cs AISGen.cs SerialIO.cs ObjectFile.cs \
              CoffFile.cs ElfFile.cs BinaryFile.cs
OBJECTS:=$(patsubst %.cs,%.module,$(REMOTESOURCES))
EXECUTABLE:=../$(PROGRAM)_$(DEVSTRING).exe

COMMONPATH:=../../../../Common
ICON:=$(COMMONPATH)/ti.ico
UTILLIBPATH:=$(COMMONPATH)/UtilLib
AISGENLIBPATH:=$(COMMONPATH)/AIS

DEVCOMMONPATH:=../../../Common
GNUCOMMONPATH:=../../Common
DEVAISGENLIBPATH:=$(GNUCOMMONPATH)/AISGenLib

AISEXTRAPATH:=../AISextra
ARM_AISEXTRAIMAGE:=$(AISEXTRAPATH)/ARM_AISExtra_$(DEVSTRING).out
DSP_AISEXTRAIMAGE:=$(AISEXTRAPATH)/DSP_AISExtra_$(DEVSTRING).out

# Target definitions
all: $(EXECUTABLE) $(OBJECTS) $(LOCALSOURCES) $(REMOTESOURCES) 

.PHONY : clean
clean:
		-rm -f -v $(EXECUTABLE) $(OBJECTS)

$(ARM_AISEXTRAIMAGE) $(DSP_AISEXTRAIMAGE):
		make -C $(AISEXTRAPATH) all
        
$(EXECUTABLE): $(ARM_AISEXTRAIMAGE) $(DSP_AISEXTRAIMAGE) $(LOCALSOURCES) $(OBJECTS) $(REMOTESOURCES) makefile
ifeq ($(shell uname),Linux)
		@$(MONOCOMPILE) -t:exe -resource:$(ARM_AISEXTRAIMAGE) -resource:$(DSP_AISEXTRAIMAGE) -out:$(EXECUTABLE) $(LOCALSOURCES) $(OBJECTS)
else
		@$(DOTNETCOMPILE) /t:exe /resource:$(ARM_AISEXTRAIMAGE) /resource:$(DSP_AISEXTRAIMAGE) /out:$(EXECUTABLE) /win32icon:$(ICON) $(LOCALSOURCES) $(OBJECTS)
endif

%.module: %.cs
		@cp $< $@

vpath %.cs $(UTILLIBPATH)/:$(DEVAISGENLIBPATH)/:$(AISGENLIBPATH)/:$(DEVCOMMONPATH)/src/
