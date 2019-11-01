#
# Copyright (c) 2018, Texas Instruments Incorporated
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

# Filename: rules_a72-qnx.mk
#
# Make rules for A72-QNX - This file has all the common rules and defines required
#                     for Cortex-A72 ISA when building for QNX
#
# This file needs to change when:
#     1. Code generation tool chain changes (This file uses Code Sourcery)
#     2. Internal switches (which are normally not touched) has to change
#     3. a rule common for A72 ISA has to be added or modified

# Set compiler/archiver/linker commands and include paths
CODEGEN_INCLUDE = $(TOOLCHAIN_PATH_QNX_A72)/arm-none-eabi/include
CC = $(TOOLCHAIN_PATH_QNX_A72)/aarch64-unknown-nto-qnx7.0.0-gcc
AR = $(TOOLCHAIN_PATH_QNX_A72)/aarch64-unknown-nto-qnx7.0.0-ar
LNK = $(TOOLCHAIN_PATH_QNX_A72)/aarch64-unknown-nto-qnx7.0.0-gcc
SIZE = $(TOOLCHAIN_PATH_QNX_A72)/aarch64-unknown-nto-qnx7.0.0-size

# Derive a part of RTS Library name based on ENDIAN: little/big
ifeq ($(ENDIAN),little)
  RTSLIB_ENDIAN = le
else
  RTSLIB_ENDIAN = be
endif

# Derive compiler switch and part of RTS Library name based on FORMAT: COFF/ELF
ifeq ($(FORMAT),ELF)
  CSWITCH_FORMAT = eabi
  RTSLIB_FORMAT = eabi
endif

# Internal CFLAGS - normally doesn't change
CFLAGS_INTERNAL = -Wimplicit -Wall -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections -c -mcpu=cortex-a72+fp+simd -g -mabi=lp64 -mstrict-align -std=gnu99 -fms-extensions
CFLAGS_INTERNAL += -mfix-cortex-a53-835769 -mfix-cortex-a53-843419
ifeq ($(TREAT_WARNINGS_AS_ERROR), yes)
  CFLAGS_INTERNAL += -Werror
  LNKFLAGS_INTERNAL_COMMON += -Werror
endif
CFLAGS_DIROPTS =

# CFLAGS based on profile selected
ifeq ($(BUILD_PROFILE_$(CORE)), debug)
  CFLAGS_INTERNAL += -D_DEBUG_=1 -ggdb -ggdb3 -gdwarf-2
  LNKFLAGS_INTERNAL_BUILD_PROFILE =
endif
ifeq ($(BUILD_PROFILE_$(CORE)), release)
  CFLAGS_INTERNAL += -O2 -s -DNDEBUG
  LNKFLAGS_INTERNAL_BUILD_PROFILE = -O2
endif

LNKFLAGS_INTERNAL_BUILD_PROFILE =

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

# Decide the compile mode
COMPILEMODE =
ifeq ($(CPLUSPLUS_BUILD), yes)
  COMPILEMODE = -x c++
endif

# Object file creation
# The first $(CC) generates the dependency make files for each of the objects
# The second $(CC) compiles the source to generate object
$(OBJ_PATHS): $(OBJDIR)/%.$(OBJEXT): %.c | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<
	$(CC) -MD -MF $(DEPDIR)/$(basename $(notdir $<)).P $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) $(COMPILEMODE) -o $(OBJDIR)/$(basename $(notdir $<)).$(OBJEXT) $<

$(OBJ_PATHS_CPP): $(OBJDIR)/%.$(OBJEXT): %.cpp | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<
	$(CC) -MD -MF $(DEPDIR)/$(basename $(notdir $<)).P $(_CFLAGS) -Wno-write-strings $(INCLUDES) $(CFLAGS_DIROPTS) $(COMPILEMODE) -o $(OBJDIR)/$(basename $(notdir $<)).$(OBJEXT) $<

ASMFLAGS = $(ASMFLAGS_INTERNAL) $(ASMFLAGS_GLOBAL_$(CORE)) $(ASMFLAGS_LOCAL_COMMON) $(ASMFLAGS_LOCAL_$(CORE)) $(ASMFLAGS_LOCAL_$(BOARD)) $(ASMFLAGS_LOCAL_$(SOC)) $(ASMFLAGS_APP_DEFINES) $(ASMFLAGS_COMP_COMMON) $(ASMFLAGS_GLOBAL_$(BOARD))
# Object file creation
$(OBJ_PATHS_ASM): $(OBJDIR)/%.$(OBJEXT): %.asm | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<
	$(CC) -c -x assembler-with-cpp $(_CFLAGS) $(ASMFLAGS) $(INCLUDES) -o $(OBJDIR)/$(basename $(notdir $<)).$(OBJEXT) $<

$(PACKAGE_PATHS): $(PACKAGEDIR)/%: %
	$(ECHO) \# Copying to $(PACKAGE_RELPATH)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)/$<
	$(MKDIR) -p $(PACKAGE_ROOT)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)
	$(CP) --parents -rf $< $(PACKAGE_ROOT)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)

# Archive flags - normally doesn't change
ARFLAGS = cr

# Archive/library file creation
$(LIBDIR)/$(LIBNAME).$(LIBEXT) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(OBJ_PATHS_CPP) | $(LIBDIR)
	$(ECHO) \#
	$(ECHO) \# Archiving $(PRINT_MESSAGE) into $@...
	$(ECHO) \#
	$(AR) $(ARFLAGS) $@ $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(OBJ_PATHS_CPP)

$(LIBDIR)/$(LIBNAME).$(LIBEXT)_size:
	$(ECHO) \#
	$(ECHO) \# Computing sectti size $(PRINT_MESSAGE) info into $@.txt...
	$(ECHO) \#
	$(SIZE) $(subst _size,,$@) > $@.txt

# Linker options and rules
LNKFLAGS_INTERNAL_COMMON += -Wl,-static -Wl,--gc-sections -nostartfiles

# Assemble Linker flags from all other LNKFLAGS definitions
_LNKFLAGS = $(LNKFLAGS_INTERNAL_COMMON) $(LNKFLAGS_INTERNAL_BUILD_PROFILE) $(LNKFLAGS_GLOBAL_$(CORE)) $(LNKFLAGS_LOCAL_COMMON) $(LNKFLAGS_LOCAL_$(CORE))

# Path of the RTS library - normally doesn't change for a given tool-chain
RTSLIB_PATH =

LIB_PATHS += $(APP_LIBS_$(CORE))
LIB_PATHS += $(EXT_LIB_a72-qnx_0)
LIB_PATHS += $(EXT_LIB_PATHS)
LIB_PATHS += $(RTSLIB_PATH)

LNKCMD_PREFIX = -Wl,-T,
LINKER1 = $(addprefix $(LNKCMD_PREFIX), $(CONFIG_BLD_LNK_a72-qnx))

ifneq ($(LNKCMD_FILE),)
LINKER2 = $(addprefix $(LNKCMD_PREFIX), $(LNKCMD_FILE))
endif

ifneq ($(XDC_CFG_FILE_$(CORE)),)
  ifneq ($(EXTERNAL_LNKCMD_FILE_LOCAL),)
    LINKER1 = $(addprefix $(LNKCMD_PREFIX), $(EXTERNAL_LNKCMD_FILE_LOCAL))
  endif
else
  ifneq ($(LNKCMD_FILE),)
    LINKER1 =
  endif
  ifneq ($(EXTERNAL_LNKCMD_FILE_LOCAL),)
    LINKER1 = $(addprefix $(LNKCMD_PREFIX), $(EXTERNAL_LNKCMD_FILE_LOCAL))
  endif
endif

LNK_LIBS = $(addprefix -L,$(LIB_PATHS_DIR))
LNK_LIBS += $(addprefix -L,$(LIB_PATHS))
LNK_LIBS += $(addprefix -L,$(EXT_LIB_PATHS))

# Added -lgcc twice to support both "before" and "after" order with -lm
# This is required to satisfy some linking order to support
# some Math/Algo kernals on A72
LNK_LIBS += -lstdc++ -lgcc -lm -lgcc -lc -lrdimon

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

$(EXE_NAME) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(OBJ_PATHS_CPP) $(LIB_PATHS) $(LNKCMD_FILE)
	$(ECHO) \# Linking into $(EXE_NAME)...
	$(ECHO) \#
	$(LNK) $(_LNKFLAGS) $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(OBJ_PATHS_CPP) $(LINKER1) $(LINKER2) $(LNK_LIBS) -Wl,--build-id=none -Wl,-Map=$@.map -o $@ $(EXT_LIB_a72-qnx_0) $(EXT_LIB_PATHS)  -Wl,--start-group $(LIB_PATHS) -Wl,--end-group
	$(ECHO) \#
	$(ECHO) \# $@ created.
	$(ECHO) \#

# Include dependency make files that were generated by $(CC)
-include $(SRCS:%.c=$(DEPDIR)/%.P)
-include $(SRCS_CPP:%.cpp=$(DEPDIR)/%.P)

# Nothing beyond this point
