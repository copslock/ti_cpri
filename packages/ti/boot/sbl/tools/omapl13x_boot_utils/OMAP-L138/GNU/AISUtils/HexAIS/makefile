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

NAND_ECC_PATCH_PATH:=../NAND_ECC_Patch
ARM_NAND_ECC_PATCH:=$(NAND_ECC_PATCH_PATH)/ARM_nand_ecc_patch_$(DEVSTRING).out
DSP_NAND_ECC_PATCH:=$(NAND_ECC_PATCH_PATH)/DSP_nand_ecc_patch_$(DEVSTRING).out

BINARY_RESOURCES:=$(ARM_AISEXTRAIMAGE) $(DSP_AISEXTRAIMAGE) $(ARM_NAND_ECC_PATCH) $(DSP_NAND_ECC_PATCH)

# Target definitions
all: $(EXECUTABLE) $(OBJECTS) $(LOCALSOURCES) $(REMOTESOURCES) 

.PHONY : clean
clean:
		-rm -f -v $(EXECUTABLE) $(OBJECTS)

$(ARM_AISEXTRAIMAGE) $(DSP_AISEXTRAIMAGE):
		make -C $(AISEXTRAPATH) all
        
$(ARM_NAND_ECC_PATCH) $(DSP_NAND_ECC_PATCH):
		make -C $(NAND_ECC_PATCH_PATH) all

$(EXECUTABLE): $(BINARY_RESOURCES) $(LOCALSOURCES) $(OBJECTS) $(REMOTESOURCES) makefile
ifeq ($(shell uname),Linux)
		$(MONOCOMPILE) -t:exe $(foreach res,$(BINARY_RESOURCES),-resource:$(res) ) -out:$(EXECUTABLE) $(LOCALSOURCES) $(OBJECTS)
else
		$(DOTNETCOMPILE) /t:exe $(foreach res,$(BINARY_RESOURCES),/resource:$(res) ) /out:$(EXECUTABLE) /win32icon:$(ICON) $(LOCALSOURCES) $(OBJECTS)
endif

%.module: %.cs
		@cp $< $@

vpath %.cs $(UTILLIBPATH)/:$(DEVAISGENLIBPATH)/:$(AISGENLIBPATH)/:$(DEVCOMMONPATH)/src/
