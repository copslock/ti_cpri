# Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
#
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Macro definitions referenced below
#

#
# Make rules for EVE - This file has all the common rules and defines required
#                     for Cortex-EVE ISA
#
# This file needs to change when:
#     1. Code generation tool chain changes
#     2. Internal switches (which are normally not touched) has to change
#     3. XDC specific switches change
#     4. a rule common for EVE ISA has to be added or modified

# Endianness : Allowed values = little | big
ENDIAN = little

# Format : Allowed values = COFF | ELF
FORMAT = ELF

ISA = arp32
ARCH = arp32

#
# Derive XDC/ISA specific settings
#
ifeq ($(FORMAT),ELF)
  FORMAT_EXT = e
endif

ifeq ($(SOC), AM574x)
  SOC_FLD = am574x
  BOARD = idkAM574x
endif

ifeq ($(SOC), AM572x)
  SOC_FLD = am572x
  BOARD = idkAM572x
endif

# If ENDIAN is set to "big", set ENDIAN_EXT to "e", that would be used in
#    in the filename extension of object/library/executable files
ifeq ($(ENDIAN),big)
  ENDIAN_EXT = e
endif

SBL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl
EVE_FIRMWARE_OBJDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/mpuM4EveLoaderApp/eveFirmware/arp32/obj
EVE_FIRMWARE_BINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/mpuM4EveLoaderApp/eveFirmware/arp32/bin

OBJEXT = o$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
LIBEXT = a$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
EXEEXT = x$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
ASMEXT = s$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)

# Set compiler/archiver/linker commands and include paths
CODEGEN_INCLUDE = $(TOOLCHAIN_PATH_EVE)/include
CC = $(TOOLCHAIN_PATH_EVE)/bin/cl-arp32
AR = $(TOOLCHAIN_PATH_EVE)/bin/ar-arp32
LNK = $(TOOLCHAIN_PATH_EVE)/bin/lnk-arp32
LD = $(TOOLCHAIN_PATH_EVE)/bin/cl-arp32

# Derive a part of RTS Library name based on ENDIAN: little/big
ifeq ($(ENDIAN),little)
  RTSLIB_ENDIAN =
else
  RTSLIB_ENDIAN = e
endif

# Executable using device independent library and device object file
EVE1_EXE=eve1_firmware.$(EXEEXT)
EVE2_EXE=eve2_firmware.$(EXEEXT)
ifeq ($(SOC), AM572x)
EVE3_EXE=eve3_firmware.$(EXEEXT)
EVE4_EXE=eve4_firmware.$(EXEEXT)
endif

# EVE firmware layout when embedding into IPU firmware
ENDIANNESS:=LE
DEV_ID:=55
INPUTRPRCS:=
EVE1_ID:=10
EVE2_ID:=11
ifeq ($(SOC), AM572x)
EVE3_ID:=12
EVE4_ID:=13
endif

EVE1_RPRC_PATH:=$(EVE_FIRMWARE_BINDIR)/$(EVE1_EXE).rprc
EVE2_RPRC_PATH:=$(EVE_FIRMWARE_BINDIR)/$(EVE2_EXE).rprc
ifeq ($(SOC), AM572x)
EVE3_RPRC_PATH:=$(EVE_FIRMWARE_BINDIR)/$(EVE3_EXE).rprc
EVE4_RPRC_PATH:=$(EVE_FIRMWARE_BINDIR)/$(EVE4_EXE).rprc
endif

EVECORE ?= $(EVE1_EXE)
TARGET ?= eve1

# List of INPUT RPRCs that can be enabled based on user requirements
INPUTRPRCS += $(EVE1_ID) $(EVE1_RPRC_PATH)
#INPUTRPRCS += $(EVE2_ID) $(EVE2_RPRC_PATH)
#INPUTRPRCS += $(EVE3_ID) $(EVE3_RPRC_PATH)
#INPUTRPRCS += $(EVE4_ID) $(EVE4_RPRC_PATH)

#Libraries
CSL_LIB = "$(PDK_INSTALL_PATH)/ti/csl/lib/$(SOC_FLD)/$(ISA)/release/ti.csl.aearp32F"

ifeq ($(OS),Windows_NT)
OUT2RPRC            = $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/out2rprc/bin/out2rprc.exe
else
OUT2RPRC            = mono $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/out2rprc/bin/out2rprc.exe
endif

MULTICORE_IMAGE_GEN = $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/multicoreImageGen/bin/MulticoreImageGen

# Internal CFLAGS - normally doesn't change
EVE_CFLAGS_INTERNAL = -c --gen_func_subsections -qq -pdsw225 --mem_model:data=far_aggregates -eo.$(OBJEXT) -ea.$(ASMEXT) -DSOC_$(SOC) --embed_inline_assembly

# Path of the RTS library - normally doesn't change for a given tool-chain
#Let the linker choose the required library
 RTSLIB_PATH = $(TOOLCHAIN_PATH_EVE)/lib/libc.a
 LIB_PATHS += $(RTSLIB_PATH)
 LNK_LIBS = $(addprefix -l,$(LIB_PATHS))
 LNKCMD_FILE = $(SBL_SRC_DIR)/example/mpuM4EveLoaderApp/eve1MulticoreApp/lnk_$(TARGET).cmd

 INTERNALLINKDEFS = --warn_sections -e=_c_int00 --silicon_version=arp32 --priority -w -x $(CSL_LIB)

# INCLUDE Directories
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx

DEPDIR = $(EVE_FIRMWARE_OBJDIR)/.deps
DEPFILE = $(DEPDIR)/$(*F)

SRCDIR = $(SBL_SRC_DIR)/example/mpuM4EveLoaderApp/eve1MulticoreApp

INCLUDES = -I$(PDK_INSTALL_PATH) -I$(SBL_SOC_DIR) -I$(TOOLCHAIN_PATH_EVE)/include

VPATH=$(SRCDIR):$(SBL_SRC_DIR)/soc/am57xx

#Lists the Source Files
SRC_C = \
	sbl_multicore_eve1.c \
	mailbox.c

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(EVE_FIRMWARE_OBJDIR)/%.$(OBJEXT), $(SRC_C))

eveFirmware: $(SBL_SRC_DIR)/example/mpuM4EveLoaderApp/ipu1EveLoaderApp/eve_firmware.h

#Converts the EVE multicore application binary into eve_firmware.h.
$(SBL_SRC_DIR)/example/mpuM4EveLoaderApp/ipu1EveLoaderApp/eve_firmware.h: $(EVE1_RPRC_PATH)
	$(MULTICORE_IMAGE_GEN) $(ENDIANNESS) $(DEV_ID) $(EVE_FIRMWARE_BINDIR)/app $(INPUTRPRCS)
	hexdump -ve '1/1 "0x%02x,"' $(EVE_FIRMWARE_BINDIR)/app | \
            sed -e "s/\(0x..,\)\{90\}/&\n/g" -e "\$$a // bytes" > $@

# Recursively calls the makefile to generate the binary for all 
# EVE instances on user preference.
$(EVE1_RPRC_PATH):
	$(MAKE) -f ./eve1MulticoreApp/makefile EVECORE=$(EVE1_EXE) TARGET=eve1 $(EVE1_EXE)

$(EVE2_RPRC_PATH):
	$(MAKE) -f ./eve1MulticoreApp/makefile EVECORE=$(EVE2_EXE) TARGET=eve2 $(EVE2_EXE)

$(EVE3_RPRC_PATH):
	$(MAKE) -f ./eve1MulticoreApp/makefile EVECORE=$(EVE3_EXE) TARGET=eve3 $(EVE3_EXE)

$(EVE4_RPRC_PATH):
	$(MAKE) -f ./eve1MulticoreApp/makefile EVECORE=$(EVE4_EXE) TARGET=eve4 $(EVE4_EXE)

# Generates EVE binary and RPRC images for EVE firmware
$(EVECORE): $(EVE_FIRMWARE_BINDIR)/$(EVECORE)
	$(OUT2RPRC) $< $(EVE_FIRMWARE_BINDIR)/$(EVECORE).rprc

$(EVE_FIRMWARE_BINDIR)/$(EVECORE): $(SRC_OBJS) $(EVE_FIRMWARE_BINDIR)/.created $(EVE_FIRMWARE_OBJDIR)/.created
	@echo linking $(SRC_OBJS) into $@ ...
	$(LNK) $(SRC_OBJS) $(INTERNALLINKDEFS) $(LNKCMD_FILE) -o $@ -m $@.map $(LNK_LIBS)

$(EVE_FIRMWARE_OBJDIR)/%.$(OBJEXT): %.c $(EVE_FIRMWARE_OBJDIR)/.created
	@echo compiling $< ...
	$(CC) $(EVE_CFLAGS_INTERNAL) $(INCLUDES) -fr=$(EVE_FIRMWARE_OBJDIR) -fs=$(EVE_FIRMWARE_OBJDIR) -fc $<

$(EVE_FIRMWARE_OBJDIR)/.created:
	@mkdir -p $(EVE_FIRMWARE_OBJDIR)
	@touch $(EVE_FIRMWARE_OBJDIR)/.created

$(EVE_FIRMWARE_BINDIR)/.created:
	@mkdir -p $(EVE_FIRMWARE_BINDIR)
	@touch $(EVE_FIRMWARE_BINDIR)/.created

eveFirmware_clean:
	@rm  -rf $(EVE_FIRMWARE_BINDIR)
	@rm -f $(SRC_OBJS) $(EVE_FIRMWARE_BINDIR)/.created $(EVE_FIRMWARE_OBJDIR)/.created
