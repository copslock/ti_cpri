# Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

empty =
space =$(empty) $(empty)

CFLAGS += -Wall -O2

ARMV7OBJDIR ?= ./obj
ARMV7BINDIR ?= ./bin
ARMV7LIBDIR ?= ./lib

ARMV7OBJDIR := $(ARMV7OBJDIR)/dfe/test
ARMV7BINDIR := $(ARMV7BINDIR)/dfe/test

#Cross tools
ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

# INCLUDE Directories
DFE_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/dfe
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl
PDK_INSTALL_DIR = $(PDK_INSTALL_PATH)
DFE_TEST_DIR = $(DFE_SRC_DIR)/test
DFE_ARM_LIN_TEST_DIR = $(DFE_TEST_DIR)/$(DEVICE)/armv7/linux
IQN2_LIB_DIR = $(PDK_INSTALL_PATH)/ti/drv/iqn2/lib

INCDIR := $(DFE_SRC_DIR);$(CSL_DIR);$(PDK_INSTALL_DIR);$(DFE_TEST_DIR)/src;$(DFE_TEST_DIR)/utils;$(DFE_ARM_LIN_TEST_DIR)

# Libraries
DFE_LIB = -ldfe
IQN2_LIB = -liqn2

ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(DFE_LIB) $(IQN2_LIB)
else
#forcing ARM executable to depend on static LLD libraries
EXE_EXTN =
LIBS     = -static $(DFE_LIB)  $(IQN2_LIB) -Wl,-Bdynamic
endif

LIBS    += -lm


# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) -D__ARMv7 -DDEVICE_K2L -DLTE_RATE=20 -DCFG -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD -DINTERNAL_LINKING_RAM

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) -L $(IQN2_LIB_DIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

EXE=dfeTest$(EXE_EXTN).out
OBJEXT = o 
SRCDIR = $(DFE_TEST_DIR)/utils:$(DFE_TEST_DIR)/src:$(DFE_ARM_LIN_TEST_DIR)

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR)

#List the Source Files
TEST_SRC = \
	dfe_osal.c  \
	dfess.c \
	serdes_cfg.c \
	cslUtils.c \
	dfe2CSLIFace.c \
	dynamic_testMain.c  \
	start_dfe.c  \
	testCase.c  \
	testCmd.c  \
	tgtData.c

# FLAGS for the SourceFiles
SRC_CFLAGS = -I. $(CFLAGS)

# Make Rule for the SRC Files
TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(TEST_SRC))

all:$(ARMV7BINDIR)/$(EXE)

$(ARMV7BINDIR)/$(EXE): $(TEST_SRC_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(SRC_CFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7BINDIR)/.created:
	@mkdir -p $(ARMV7BINDIR)
	@touch $(ARMV7BINDIR)/.created

clean:
	@rm -fr $(ARMV7OBJDIR)
	@rm -fr $(ARMV7BINDIR)
