# Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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
# Make rules for c66x - This file has all the common rules and defines required
#                     for c66x ISA
#
# This file needs to change when:
#     1. Code generation tool chain changes (currently it uses CG600_7.2.0.B2)
#     2. Internal switches (which are normally not touched) has to change
#     3. XDC specific switches change
#     4. a rule common for C66 ISA has to be added or modified

# Endianness : Allowed values = little | big
ENDIAN = little

# Format : Allowed values = COFF | ELF
FORMAT = ELF

ISA = 66
ARCH = c66x

#
# Derive XDC/ISA specific settings
#
ifeq ($(FORMAT),ELF)
  FORMAT_EXT = e
endif

# If ENDIAN is set to "big", set ENDIAN_EXT to "e", that would be used in
#    in the filename extension of object/library/executable files
ifeq ($(ENDIAN),big)
  ENDIAN_EXT = e
endif

SBL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl
SBL_DSP_OBJDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/c66/dsp1/obj
SBL_DSP_BINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/c66/dsp1/bin

OBJEXT = o$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
LIBEXT = a$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
EXEEXT = x$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)
ASMEXT = s$(FORMAT_EXT)$(ISA)$(ENDIAN_EXT)

# Set compiler/archiver/linker commands and include paths
CODEGEN_INCLUDE = $(C6X_GEN_INSTALL_PATH)/include
CC = $(C6X_GEN_INSTALL_PATH)/bin/cl6x -c
AR = $(C6X_GEN_INSTALL_PATH)/bin/ar6x
LNK = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x
LD = $(C6X_GEN_INSTALL_PATH)/bin/lnk6x

# Derive a part of RTS Library name based on ENDIAN: little/big
ifeq ($(ENDIAN),little)
  RTSLIB_ENDIAN =
else
  RTSLIB_ENDIAN = e
endif

# Derive compiler switch and part of RTS Library name based on FORMAT: COFF/ELF
ifeq ($(FORMAT),COFF)
  CSWITCH_FORMAT = ti_arm9_abi
  RTSLIB_FORMAT = _tiarm9
endif
ifeq ($(FORMAT),ELF)
  CSWITCH_FORMAT = eabi
  RTSLIB_FORMAT = _elf
endif

# Internal CFLAGS - normally doesn't change
C66_CFLAGS_INTERNAL = -qq -pdsw225 --endian=$(ENDIAN) -mv6600 --abi=$(CSWITCH_FORMAT) -eo.$(OBJEXT) -ea.$(ASMEXT) -mi10 -mo -pdr -pden -pds=238 -pds=880 -pds1110 --program_level_compile -g -DSOC_$(SOC) -ppa -ppd=$@.dep

LNKFLAGS_INTERNAL_PROFILE =
C66_CFLAGS_INTERNAL += -O2 --optimize_with_debug --diag_wrap=off --preproc_with_compile
LNKFLAGS_GLOBAL_c66x = -x --zero_init=on --retain=_vectors

# Path of the RTS library - normally doesn't change for a given tool-chain
#Let the linker choose the required library
 RTSLIB_PATH = $(C6X_GEN_INSTALL_PATH)/lib/libc.a
 LIB_PATHS += $(RTSLIB_PATH)
 LNK_LIBS = $(addprefix -l,$(LIB_PATHS))
 LNKCMD_FILE = $(SBL_SRC_DIR)/example/dsp1MulticoreApp/lnk_dsp1.cmd

 INTERNALLINKDEFS = -w -q -u _c_int00 --silicon_version=6600 -c --dynamic

 INTERNALLINKDEFS += $(LNKFLAGS_GLOBAL_c66x)

# INCLUDE Directories
SBL_SOC_DIR = $(SBL_SRC_DIR)/soc/am57xx

DEPDIR = $(SBL_DSP_OBJDIR)/.deps
DEPFILE = $(DEPDIR)/$(*F)

SRCDIR = $(SBL_SRC_DIR)/example/dsp1MulticoreApp

INCLUDES = -I$(PDK_INSTALL_PATH) -I$(SBL_SOC_DIR) -I$(C6X_GEN_INSTALL_PATH)/include

# Executable using device independent library and device object file
EXE=sbl_app.$(EXEEXT)

VPATH=$(SRCDIR):$(SBL_SRC_DIR)/soc/am57xx

#List the Source Files
SRC_C = \
	sbl_multicore_dsp1.c \
	mailbox.c

# Make Rule for the SRC Files
SRC_OBJS = $(patsubst %.c, $(SBL_DSP_OBJDIR)/%.$(OBJEXT), $(SRC_C))

example:$(SBL_DSP_BINDIR)/$(EXE)

$(SBL_DSP_BINDIR)/$(EXE): $(SRC_OBJS) $(SBL_DSP_BINDIR)/.created $(SBL_DSP_OBJDIR)/.created
	@echo linking $(SRC_OBJS) into $@ ...
	$(LNK) $(SRC_OBJS) $(INTERNALLINKDEFS) $(LNKCMD_FILE) -o $@ -m $@.map $(LNK_LIBS)

$(SBL_DSP_OBJDIR)/%.$(OBJEXT): %.c $(SBL_DSP_OBJDIR)/.created
	@echo compiling $< ...
	#$(CC) -ppd=$(DEPFILE).P $(C66_CFLAGS_INTERNAL) $(INCLUDES) -fr=$(SBL_DSP_OBJDIR) -fs=$(SBL_DSP_OBJDIR) -fc $<
	$(CC) $(C66_CFLAGS_INTERNAL) $(INCLUDES) -fr=$(SBL_DSP_OBJDIR) -fs=$(SBL_DSP_OBJDIR) -fc $<

$(SBL_DSP_OBJDIR)/.created:
	@mkdir -p $(SBL_DSP_OBJDIR)
	@touch $(SBL_DSP_OBJDIR)/.created

$(SBL_DSP_BINDIR)/.created:
	@mkdir -p $(SBL_DSP_BINDIR)
	@touch $(SBL_DSP_BINDIR)/.created

example_clean:
	@rm -f $(SBL_DSP_BINDIR)/$(EXE)
	@rm -f $(SRC_OBJS) $(SBL_DSP_BINDIR)/.created $(SBL_DSP_OBJDIR)/.created
