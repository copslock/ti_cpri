# Copyright (C) 2013-2014 Texas Instruments Incorporated - http://www.ti.com/
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

ARMV7OBJDIR := $(ARMV7OBJDIR)/hyplnk/example
ARMV7BINDIR := $(ARMV7BINDIR)/hyplnk/example

#Cross tools
ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

# INCLUDE Directories
HYPLNK_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/hyplnk/
CSL_DIR = $(PDK_INSTALL_PATH)/ti/csl/
PDK_INSTALL_DIR = $(PDK_INSTALL_PATH)
HPLIB_DIR = $(HPLIB_INSTALL_PATH)/ti/runtime/hplib/
HYPLNK_SAMPLE_DIR = $(HYPLNK_SRC_DIR)/example/cicInterruptExample
HYPLNK_ARM_LIN_SAMPLE_DIR = $(HYPLNK_SAMPLE_DIR)/$(DEVICE)/armv7/linux

INCDIR := $(HYPLNK_SRC_DIR);$(CSL_DIR);$(PDK_INSTALL_DIR);$(HPLIB_DIR);$(HYPLNK_SAMPLE_DIR)/src;$(HYPLNK_SAMPLE_DIR)/../common;$(HYPLNK_ARM_LIN_SAMPLE_DIR)

# Libraries
HYPLNK_LIB = -lhyplnk
HYPLNK_DEV_LIB = -lhyplnk_$(DEVICE)
HPLIB = -lhplib

#presuming ARM executable would depend on dynamic library dependency
DYNAMIC_EXTN = _so
LIBS     = $(HYPLNK_LIB)
DEVLIBS  = $(HYPLNK_DEV_LIB)

# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) -D__ARMv7 -DDEVICE_K2E -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD -DINTERNAL_LINKING_RAM

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) -static $(LIBS) -Wl,-Bdynamic -lrt -Wl,--end-group -pthread $(LDFLAGS)
INTERNALLINKDEFSDEVLIB = -Wl,--start-group -L $(ARMV7LIBDIR) -static $(DEVLIBS) -Wl,-Bdynamic -lrt -Wl,--end-group -pthread $(LDFLAGS)
INTERNALLINKDEFSDYNAMIC = -Wl,--start-group -L $(ARMV7LIBDIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)
INTERNALLINKDEFSDEVLIBDYNAMIC = -Wl,--start-group -L $(ARMV7LIBDIR) $(DEVLIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

# Executable using device independent library and device object file
EXE=cicInterruptExample.out
EXE_DYNAMIC=cicInterruptExample$(DYNAMIC_EXTN).out

# Executable using device dependant library and no device object file
EXEDEVLIB=cicInterruptExampleDevLib.out
EXEDEVLIB_DYNAMIC=cicInterruptExample$(DYNAMIC_EXTN).out

OBJEXT = o
SRCDIR = $(HYPLNK_SAMPLE_DIR)/../common:$(HYPLNK_SAMPLE_DIR)/src:$(HYPLNK_ARM_LIN_SAMPLE_DIR):$(HYPLNK_SRC_DIR)/device/$(DEVICE)/src

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR)

#List the Source Files
TEST_SRC = \
	cicInterruptExample.c \
	hyplnkCSLIFace.c \
	hyplnkLLDIFace.c \
	hyplnkResource.c

# device dependent source files
TEST_SRC_DEV = \
	hyplnk_device.c

# FLAGS for the SourceFiles
SRC_CFLAGS = -I. $(CFLAGS)

# Make Rule for the SRC Files
TEST_SRC_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(TEST_SRC))
TEST_SRC_DEV_OBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(TEST_SRC_DEV))

all:$(ARMV7BINDIR)/$(EXE) $(ARMV7BINDIR)/$(EXEDEVLIB) $(ARMV7BINDIR)/$(EXE_DYNAMIC) $(ARMV7BINDIR)/$(EXEDEVLIB_DYNAMIC)

$(ARMV7BINDIR)/$(EXE): $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7BINDIR)/$(EXEDEVLIB): $(TEST_SRC_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(INTERNALLINKDEFSDEVLIB) -o $@

$(ARMV7BINDIR)/$(EXE_DYNAMIC): $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(TEST_SRC_DEV_OBJS) $(INTERNALLINKDEFSDEVLIBDYNAMIC) -o $@

$(ARMV7BINDIR)/$(EXEDEVLIB_DYNAMIC): $(TEST_SRC_OBJS) $(ARMV7BINDIR)/.created $(ARMV7OBJDIR)/.created
	@echo linking $(TEST_SRC_OBJS) into $@ ...
	@$(CC) $(TEST_SRC_OBJS) $(INTERNALLINKDEFSDEVLIBDYNAMIC) -o $@

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
