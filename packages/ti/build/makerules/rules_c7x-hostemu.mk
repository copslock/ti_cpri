#
# Copyright (c) 2017, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Filename: rules_c7x-hostemu.mk
#
# Make rules for c7x-hostemu - This file has all the common rules and defines required
#                     for c7x to be run on HOST emulation
#
# This file needs to change when:
#     1. All environment variables for Visual Studio compiler

# Set compiler/archiver/linker commands and include paths
CODEGEN_INCLUDE = $(C7X_GEN_INSTALL_PATH)/host_emulation/include
#CODEGEN_INCLUDE = $(C7X_GEN_INSTALL_PATH)/include


ifeq ($(OS),Windows_NT)
CC = CL
AR = LIB
LNK = LINK

#########

# Internal CFLAGS - normally doesn't change
CFLAGS_INTERNAL = /EHsc /TP /W0 /DHOST_EMULATION /c /nologo  /Gm /Zi /D_HOST_BUILD /D_HAS_ITERATOR_DEBUGGING=0 /D_ITERATOR_DEBUG_LEVEL=0
CFLAGS_DIROPTS = /Fo$(OBJDIR)

ifeq ($(BUILD_PROFILE_$(CORE)), release)
 CFLAGS_INTERNAL += /Ox /D_NDEBUG /MT
 LNKFLAGS_INTERNAL_BUILD_PROFILE +=
else
 CFLAGS_INTERNAL += /Od /MT
 LNKFLAGS_INTERNAL_BUILD_PROFILE += /DEBUG
endif

########################################

# Assemble CFLAGS from all other CFLAGS definitions
_CFLAGS = $(CFLAGS_GLOBAL_$(CORE)) $(CFLAGS_INTERNAL) $(CFLAGS_LOCAL_COMMON) $(CFLAGS_LOCAL_$(CORE)) $(CFLAGS_COMP_COMMON)
ifeq ($($(MODULE_NAME)_BOARD_DEPENDENCY),yes)
  _CFLAGS += $(CFLAGS_LOCAL_$(BOARD)) $(CFLAGS_GLOBAL_$(BOARD))
else
  ifeq ($($(APP_NAME)_BOARD_DEPENDENCY),yes)
    _CFLAGS += $(CFLAGS_LOCAL_$(BOARD)) $(CFLAGS_GLOBAL_$(BOARD))
  else
    _CFLAGS += $(CFLAGS_LOCAL_$(SOC)) $(CFLAGS_GLOBAL_$(SOC))
  endif
endif


# Object file creation
# The first $(CC) generates the dependency make files for each of the objects
# The second $(CC) compiles the source to generate object
$(OBJ_PATHS): $(OBJDIR)/%.$(OBJEXT): %.c | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<

	$(CC) $(_CFLAGS) $(INCLUDES) $< /Fo$@ /Fd$(DEPFILE).pdb
# Archive flags - normally doesn't change
ARFLAGS := /nologo /MACHINE:X64

# Archive/library file creation
$(LIBDIR)/$(LIBNAME).$(LIBEXT) : $(OBJ_PATHS) | $(LIBDIR)
	$(ECHO) \#
	$(ECHO) \# Archiving $(PRINT_MESSAGE) into $@ ...
	$(ECHO) \#
	$(AR) $(ARFLAGS) /OUT:$@ $(OBJ_PATHS)

# Linker options and rules
LNKFLAGS_INTERNAL_COMMON +=

# Assemble Linker flags from all other LNKFLAGS definitions
_LNKFLAGS = $(LNKFLAGS_INTERNAL_COMMON) $(LNKFLAGS_INTERNAL_BUILD_PROFILE) $(LNKFLAGS_GLOBAL_$(CORE)) $(LNKFLAGS_LOCAL_COMMON) $(LNKFLAGS_LOCAL_$(CORE))

LNK_LIBS = $(LIB_PATHS)
LNK_LIBS += $(EXT_LIB_PATHS)

# Linker - to create executable file
ifeq ($(LOCAL_APP_NAME),)
 EXE_NAME = $(BINDIR)/$(APP_NAME)_$(CORE)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
else
 ifeq ($(BUILD_PROFILE_$(CORE)),prod_release)
  EXE_NAME = $(BINDIR)/$(LOCAL_APP_NAME).$(EXEEXT)
 else
  EXE_NAME = $(BINDIR)/$(LOCAL_APP_NAME)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
 endif
endif

$(EXE_NAME) : $(OBJ_PATHS) $(LIB_PATHS) $(LNKCMD_FILE)
	$(ECHO) \# Linking into $(EXE_NAME)...
	$(ECHO) \#
	$(LNK) $(_LNKFLAGS) $(OBJ_PATHS) $(LNK_LIBS) /nologo /MACHINE:X64 /OUT:$@
	$(ECHO) \#
	$(ECHO) \# $@ created.
	$(ECHO) \#
    
    
else
CC=g++-5 -c
AR=gcc-ar-5
LD=g++-5
LNK = g++-5
CFLAGS_DIROPTS = 


ifeq ($(BUILD_PROFILE_$(CORE)), release)
  CFLAGS_INTERNAL += -std=c++14 -O3 -DHOST_EMULATION  -w -D_HOST_BUILD -DGCC_BUILD  -D__C7100__ -DCORE_DSP
  LNKFLAGS_INTERNAL_BUILD_PROFILE +=
else
  CFLAGS_INTERNAL += -std=c++14 -ggdb -ggdb3 -gdwarf-2 -DHOST_EMULATION -w -D_HOST_BUILD -DGCC_BUILD  -D__C7100__ -DCORE_DSP
  LNKFLAGS_INTERNAL_BUILD_PROFILE +=
endif


########################################

# Assemble CFLAGS from all other CFLAGS definitions
_CFLAGS = $(CFLAGS_GLOBAL_$(CORE)) $(CFLAGS_INTERNAL) $(CFLAGS_LOCAL_COMMON) $(CFLAGS_LOCAL_$(CORE)) $(CFLAGS_COMP_COMMON)
ifeq ($($(MODULE_NAME)_BOARD_DEPENDENCY),yes)
  _CFLAGS += $(CFLAGS_LOCAL_$(BOARD)) $(CFLAGS_GLOBAL_$(BOARD))
else
  ifeq ($($(APP_NAME)_BOARD_DEPENDENCY),yes)
    _CFLAGS += $(CFLAGS_LOCAL_$(BOARD)) $(CFLAGS_GLOBAL_$(BOARD))
  else
    _CFLAGS += $(CFLAGS_LOCAL_$(SOC)) $(CFLAGS_GLOBAL_$(SOC))
  endif
endif

# Object file creation
# The first $(CC) generates the dependency make files for each of the objects
# The second $(CC) compiles the source to generate object
$(OBJ_PATHS): $(OBJDIR)/%.$(OBJEXT): %.c | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<

	$(CC) $(_CFLAGS) $(INCLUDES) $< -o $@ 
# Archive flags - normally doesn't change
ARFLAGS :=rvs

# Archive/library file creation
$(LIBDIR)/$(LIBNAME).$(LIBEXT) : $(OBJ_PATHS) | $(LIBDIR)
	$(ECHO) \#
	$(ECHO) \# Archiving $(PRINT_MESSAGE) into $@ ...
	$(ECHO) \#
	$(AR) $(ARFLAGS) $@ $(OBJ_PATHS)

# Linker options and rules
LNKFLAGS_INTERNAL_COMMON +=

# Assemble Linker flags from all other LNKFLAGS definitions
_LNKFLAGS = $(LNKFLAGS_INTERNAL_COMMON) $(LNKFLAGS_INTERNAL_BUILD_PROFILE) $(LNKFLAGS_GLOBAL_$(CORE)) $(LNKFLAGS_LOCAL_COMMON) $(LNKFLAGS_LOCAL_$(CORE))

LNK_LIBS_DIR = $(dir $(LIB_PATHS))
LNK_LIBS_DIR := $(addprefix -L ,$(LNK_LIBS_DIR))

LNK_LIBS_NAME = $(notdir $(LIB_PATHS))
LNK_LIBS_NAME := $(addprefix -l:,$(LNK_LIBS_NAME))


LNK_LIBS = $(LIB_PATHS)
LNK_LIBS += $(EXT_LIB_PATHS)

# Linker - to create executable file
ifeq ($(LOCAL_APP_NAME),)
 EXE_NAME = $(BINDIR)/$(APP_NAME)_$(CORE)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
else
 ifeq ($(BUILD_PROFILE_$(CORE)),prod_release)
  EXE_NAME = $(BINDIR)/$(LOCAL_APP_NAME).$(EXEEXT)
 else
  EXE_NAME = $(BINDIR)/$(LOCAL_APP_NAME)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
 endif
endif

$(EXE_NAME) : $(OBJ_PATHS) $(LIB_PATHS) $(LNKCMD_FILE)
	$(ECHO) \# Linking into $(EXE_NAME)...
	$(ECHO) \#
	$(LNK) -Wl,--start-group -o $@ $(_LNKFLAGS) $(LNK_LIBS_DIR) $(LNK_LIBS_NAME) $(OBJ_PATHS)
	$(ECHO) \#
	$(ECHO) \# $@ created.
	$(ECHO) \#

endif

# Include dependency make files that were generated by $(CC)
#-include $(SRCS:%.c=$(DEPDIR)/%.pdb)

# Nothing beyond this point
