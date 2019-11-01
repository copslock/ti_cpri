#
# Copyright (c) 2013-2016, Texas Instruments Incorporated
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

# Filename: rules_ti_cgt_arm.mk
#
# Make rules for TI ARM CGT - This file has all the common rules and defines
# required for Cortex-M4/R5F ISA
#
# This file needs to change when:
#     1. Code generation tool chain changes (currently it uses TI ARM CGT)
#     2. Internal switches (which are normally not touched) has to change
#     3. XDC specific switches change
#     4. a rule common for M4/R5F ISA has to be added or modified

# Set compiler/archiver/linker commands and include paths
CODEGEN_INCLUDE = $(TOOLCHAIN_PATH_$(CGT_ISA))/include
CC = $(TOOLCHAIN_PATH_$(CGT_ISA))/bin/armcl
AR = $(TOOLCHAIN_PATH_$(CGT_ISA))/bin/armar
LNK = $(TOOLCHAIN_PATH_$(CGT_ISA))/bin/armlnk
STRP = $(TOOLCHAIN_PATH_$(CGT_ISA))/bin/armstrip
SIZE = $(TOOLCHAIN_PATH_$(CGT_ISA))/bin/armofd

# Derive a part of RTS Library name based on ENDIAN: little/big
ifeq ($(ENDIAN),little)
  RTSLIB_ENDIAN = le
else
  RTSLIB_ENDIAN = be
endif

# Derive compiler switch and part of RTS Library name based on FORMAT: COFF/ELF
ifeq ($(FORMAT),COFF)
  CSWITCH_FORMAT = ti_arm9_abi
  RTSLIB_FORMAT = tiarm9
endif
ifeq ($(FORMAT),ELF)
  CSWITCH_FORMAT = eabi
  RTSLIB_FORMAT = eabi
endif

# Internal CFLAGS - normally doesn't change
ifeq ($(CGT_ISA),$(filter $(CGT_ISA), M4 R5 M3))
  CFLAGS_INTERNAL = -c -qq -pdsw225 --endian=$(ENDIAN) -mv7$(CGT_ISA) --abi=$(CSWITCH_FORMAT) -eo.$(OBJEXT) -ea.$(ASMEXT) --symdebug:dwarf --embed_inline_assembly
  ifeq ($(CGT_ISA),$(filter $(CGT_ISA), R5))
    CFLAGS_INTERNAL += --float_support=vfpv3d16
  else
    CFLAGS_INTERNAL += --float_support=vfplib
  endif
else ifeq ($(CGT_ISA), Arm9)
  CFLAGS_INTERNAL = -c -qq -pdsw225 --endian=$(ENDIAN) -mv5e --float_support=vfplib --abi=$(CSWITCH_FORMAT) -eo.$(OBJEXT) -ea.$(ASMEXT) --symdebug:dwarf --embed_inline_assembly
endif
ifeq ($(TREAT_WARNINGS_AS_ERROR), yes)
  CFLAGS_INTERNAL += --emit_warnings_as_errors
  LNKFLAGS_INTERNAL_COMMON += --emit_warnings_as_errors
endif
CFLAGS_DIROPTS = -fr=$(OBJDIR) -fs=$(OBJDIR)

ifeq ($(CGT_ISA),$(filter $(CGT_ISA),R5))
 XDC_TARGET_NAME=$(CGT_ISA)F

 ifneq ($(EXTERNAL_LNKCMD_FILE_LOCAL),)
 EXTERNAL_LNKCMD_FILE = $(EXTERNAL_LNKCMD_FILE_LOCAL)
 else
 EXTERNAL_LNKCMD_FILE = $(CONFIG_BLD_LNK_r5f)
 endif

else
 XDC_TARGET_NAME=$(CGT_ISA)
endif


XDC_HFILE_NAME = $(basename $(notdir $(XDC_CFG_FILE_$(CORE))))
# CFLAGS based on profile selected
ifeq ($(BUILD_PROFILE_$(CORE)), debug)
 LNKFLAGS_INTERNAL_BUILD_PROFILE =
 CFLAGS_XDCINTERNAL = -Dxdc_target_name__=$(XDC_TARGET_NAME) -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_debug
 CFLAGS_INTERNAL += -D_DEBUG_=1
 ifndef MODULE_NAME
  CFLAGS_XDCINTERNAL += -Dxdc_cfg__header__='$(CONFIGURO_DIR)/package/cfg/$(XDC_HFILE_NAME)_pe$(CGT_EXT).h'
 endif

endif
ifeq ($(BUILD_PROFILE_$(CORE)), release)
 ifeq ($(CGT_ISA),$(filter $(CGT_ISA), M4 R5 M3))
   ifeq ($(CGT_ISA),$(filter $(CGT_ISA), R5))
     LNKFLAGS_INTERNAL_BUILD_PROFILE = --opt='--float_support=vfpv3d16 --endian=$(ENDIAN) -mv7$(CGT_ISA) --abi=$(CSWITCH_FORMAT) -qq -pdsw225 $(CFLAGS_GLOBAL_$(CORE)) -ms -op2 -O4 -s --diag_suppress=23000' --strict_compatibility=on
   else
     LNKFLAGS_INTERNAL_BUILD_PROFILE = --opt='--float_support=vfplib   --endian=$(ENDIAN) -mv7$(CGT_ISA) --abi=$(CSWITCH_FORMAT) -qq -pdsw225 $(CFLAGS_GLOBAL_$(CORE)) -oe --symdebug:dwarf -ms -op2 -O3 -os --optimize_with_debug --inline_recursion_limit=20 --diag_suppress=23000' --strict_compatibility=on
   endif
   ifeq ($(CGT_ISA),$(filter $(CGT_ISA), R5))
     CFLAGS_INTERNAL += -ms -O4 -s
   else
     CFLAGS_INTERNAL += -ms -oe -O3 -op0 -os --optimize_with_debug --inline_recursion_limit=20
   endif
   CFLAGS_XDCINTERNAL = -Dxdc_target_name__=$(XDC_TARGET_NAME) -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release
   ifndef MODULE_NAME
     CFLAGS_XDCINTERNAL += -Dxdc_cfg__header__='$(CONFIGURO_DIR)/package/cfg/$(XDC_HFILE_NAME)_pe$(CGT_EXT).h'
   endif
 endif
 ifeq ($(CGT_ISA), Arm9)
	 LNKFLAGS_INTERNAL_BUILD_PROFILE = --opt='--endian=$(ENDIAN) -mv5e --float_support=vfplib --abi=$(CSWITCH_FORMAT) -qq -pdsw225 $(CFLAGS_GLOBAL_$(CORE)) -oe --symdebug:dwarf -ms -op2 -O3 -os --optimize_with_debug --inline_recursion_limit=20 --diag_suppress=23000' --strict_compatibility=on
	 CFLAGS_INTERNAL += -ms -oe -O3 -op0 -os --optimize_with_debug --inline_recursion_limit=20
	 CFLAGS_XDCINTERNAL = -Dxdc_target_name__=$(XDC_TARGET_NAME) -Dxdc_target_types__=ti/targets/arm/elf/std.h -Dxdc_bld__profile_release
	 ifndef MODULE_NAME
	  CFLAGS_XDCINTERNAL += -Dxdc_cfg__header__='$(CONFIGURO_DIR)/package/cfg/$(XDC_HFILE_NAME)_pe$(CGT_EXT).h'
	 endif
 endif
endif

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
ifneq ($(XDC_CFG_FILE_$(CORE)),)
  _CFLAGS += $(CFLAGS_XDCINTERNAL)
else
  ifneq ($(findstring xdc, $(INCLUDE_EXTERNAL_INTERFACES)),)
      _CFLAGS += $(CFLAGS_XDCINTERNAL)
  endif
endif

# Decide the compile mode
COMPILEMODE = -fc
ifeq ($(CPLUSPLUS_BUILD), yes)
  COMPILEMODE = -fg
endif

# Object file creation
# The first $(CC) generates the dependency make files for each of the objects
# The second $(CC) compiles the source to generate object
$(OBJ_PATHS): $(OBJDIR)/%.$(OBJEXT): %.c | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<
	$(CC) -ppd=$(DEPFILE).P $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) $(COMPILEMODE) $<
	$(CC) $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) $(COMPILEMODE) $<

#TODO: Check ASMFLAGS if really required
ASMFLAGS = -me -g --code_state=32 --diag_warning=225

# Object file creation
$(OBJ_PATHS_ASM): $(OBJDIR)/%.$(OBJEXT): %.asm | $(OBJDIR) $(DEPDIR)
	$(ECHO) \# Compiling $(PRINT_MESSAGE): $<
	$(CC) -ppd=$(DEPFILE).P $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) -fa $<
	$(CC) $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) -fa $<

$(PACKAGE_PATHS): $(PACKAGEDIR)/%: %
	$(ECHO) \# Copying to $(PACKAGE_RELPATH)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)/$<
	$(MKDIR) -p $(PACKAGE_ROOT)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)
	$(CP) --parents -rf $< $(PACKAGE_ROOT)/$($(APP_NAME)$(MODULE_NAME)_RELPATH)

# Archive flags - normally doesn't change
ARFLAGS = rq

# Archive/library file creation
$(LIBDIR)/$(LIBNAME).$(LIBEXT) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) | $(LIBDIR)
	$(ECHO) \#
	$(ECHO) \# Archiving $(PRINT_MESSAGE) into $@ ...
	$(ECHO) \#
	$(AR) $(ARFLAGS) $@ $(OBJ_PATHS_ASM) $(OBJ_PATHS)

$(LIBDIR)/$(LIBNAME).$(LIBEXT)_size: $(LIBDIR)/$(LIBNAME).$(LIBEXT)
	$(ECHO) \#
	$(ECHO) \# Computing sectti size $(PRINT_MESSAGE) info into $@.txt ...
	$(ECHO) \#
	$(SIZE) -x $(subst _size,,$@) > $@temp
	$(SECTTI) $@temp > $@.txt
	$(RM)   $@temp

# Linker options and rules
LNKFLAGS_INTERNAL_COMMON += -w -q -u _c_int00

ifeq ($(BOARD),$(filter $(BOARD), qtJ7))
  LNKFLAGS_INTERNAL_COMMON += -cr --ram_model
else
  LNKFLAGS_INTERNAL_COMMON += -c
endif

ifeq ($(CGT_ISA), R5)
  LNKFLAGS_INTERNAL_COMMON += -mv7R5
  #--diag_suppress=10063 supresses 'warning: entry point other than _c_int00 specified'
  LNKFLAGS_INTERNAL_COMMON += --diag_suppress=10063
else
ifeq ($(CGT_ISA), Arm9)
  LNKFLAGS_INTERNAL_COMMON +=
else
  LNKFLAGS_INTERNAL_COMMON += --silicon_version=7$(CGT_ISA)
endif
endif

# Assemble Linker flags from all other LNKFLAGS definitions
_LNKFLAGS = $(LNKFLAGS_INTERNAL_COMMON) $(LNKFLAGS_INTERNAL_BUILD_PROFILE) $(LNKFLAGS_GLOBAL_$(CORE)) $(LNKFLAGS_LOCAL_COMMON) $(LNKFLAGS_LOCAL_$(CORE))

# Path of the RTS library - normally doesn't change for a given tool-chain
#Let the linker choose the required library
RTSLIB_PATH = $(CGT_PATH)/lib/libc.a

LNK_LIBS = $(addprefix -l,$(LIB_PATHS))
LNK_LIBS += $(addprefix -l,$(EXT_LIB_PATHS))

# Linker - to create executable file
ifeq ($(LOCAL_APP_NAME),)
 EXE_NAME = $(BINDIR)/$(APP_NAME)_$(CORE)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
 EXE_STRIP_NAME = $(BINDIR)/$(APP_NAME)_$(CORE)_$(BUILD_PROFILE_$(CORE))_strip.$(EXEEXT)
else
 EXE_NAME = $(BINDIR)/$(LOCAL_APP_NAME)_$(BUILD_PROFILE_$(CORE)).$(EXEEXT)
 EXE_STRIP_NAME = $(BINDIR)/$(LOCAL_APP_NAME)_$(BUILD_PROFILE_$(CORE))_strip.$(EXEEXT)
endif

NUM_PROCS = 1

ifeq ($(OS),Windows_NT)
  NUM_PROCS = $(NUMBER_OF_PROCESSORS)
else
  NUM_PROCS = $(shell grep -c ^processor /proc/cpuinfo)
endif

ifneq ($(findstring mcu,$(CORE)),)
BUILD_LIB_ONCE = $(CGT_PATH)/lib/rtsv7R4_A_le_v3D16_eabi.lib
$(BUILD_LIB_ONCE):
	$(ECHO) \# $@ not found, building  $@ ...
	$(CGT_PATH)/lib/mklib --pattern=rtsv7R4_A_le_v3D16_eabi.lib --parallel=$(NUM_PROCS) --compiler_bin_dir=$(CGT_PATH)/bin
endif

ifneq ($(XDC_CFG_FILE_$(CORE)),)
$(EXE_NAME) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(LIB_PATHS) $(LNKCMD_FILE) $(OBJDIR)/$(CFG_COBJ_XDC) $(BUILD_LIB_ONCE)
	$(CP) $(CONFIGURO_DIR)/package/cfg/*.rov.xs $(BINDIR)
else
$(EXE_NAME) : $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(LIB_PATHS) $(LNKCMD_FILE) $(BUILD_LIB_ONCE)
endif
	$(ECHO) \# Linking into $(EXE_NAME)...
	$(ECHO) \#
ifneq ($(XDC_CFG_FILE_$(CORE)),)
	$(CP) $(OBJDIR)/$(CFG_COBJ_XDC) $(CONFIGURO_DIR)/package/cfg
  ifeq ($(BUILD_PROFILE_$(CORE)),whole_program_debug)
	$(LNK) $(_LNKFLAGS) $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(OBJDIR)/$(CFG_COBJ_XDC) $(LNKCMD_FILE) $(EXTERNAL_LNKCMD_FILE) -o $@ -m $@.map $(LNK_LIBS) $(RTSLIB_PATH)
  else
	$(LNK) $(_LNKFLAGS) $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(LNKCMD_FILE) $(EXTERNAL_LNKCMD_FILE) -o $@ -m $@.map $(LNK_LIBS) $(RTSLIB_PATH)
  endif
else
	$(LNK) $(_LNKFLAGS) $(OBJ_PATHS_ASM) $(OBJ_PATHS) $(LNKCMD_FILE) $(EXTERNAL_LNKCMD_FILE) -o $@ -m $@.map $(LNK_LIBS) $(RTSLIB_PATH)
endif
	$(ECHO) \#
	$(ECHO) \# $@ created.
	$(ECHO) \#
ifeq ($(BUILD_PROFILE_$(CORE)), release)
	$(ECHO) \# Generating stripped image into $(EXE_STRIP_NAME)...
	$(ECHO) \#
	$(STRP) -p $(EXE_NAME) -o $(EXE_STRIP_NAME)
endif
# XDC specific - assemble XDC-Configuro command
ifeq ($(BUILD_PROFILE_$(CORE)),prod_release)
  CONFIGURO_BUILD_PROFILE = release
else
  CONFIGURO_BUILD_PROFILE = $(BUILD_PROFILE_$(CORE))
endif

_XDC_GREP_STRING = \"$(XDC_GREP_STRING)\"
EGREP_CMD = $(EGREP) -ivw $(XDC_GREP_STRING) $(XDCLNKCMD_FILE)

ifeq ($(OS),Windows_NT)
  EVERYONE = $(word 1,$(shell whoami -groups | findstr "S-1-1-0"))
endif

# Invoke configuro for the rest of the components
#  NOTE: 1. String handling is having issues with various make versions when the
#           cammand is directly tried to be given below. Hence, as a work-around,
#           the command is re-directed to a file (shell or batch file) and then
#           executed
#        2. The linker.cmd file generated, includes the libraries generated by
#           XDC. An egrep to search for these and omit in the .cmd file is added
#           after configuro is done
xdc_configuro : $(CFG_C_XDC)
	$(MAKE) $(LNKCMD_FILE)

ifeq ($(DEST_ROOT),)
  XDC_BUILD_ROOT = .
else
  XDC_BUILD_ROOT = $(DEST_ROOT)
endif

$(CFG_C_XDC) $(LNKCMD_FILE) : $(XDC_CFG_FILE_NAME) $(XDC_CFG_UPDATE)
	$(ECHO) \# Invoking configuro...
	$(MKDIR) -p $(XDC_BUILD_ROOT)
	$(xdc_PATH)/xs xdc.tools.configuro --generationOnly -o $(CONFIGURO_DIR) -t $(TARGET_XDC) -p $(PLATFORM_XDC) \
               -r $(CONFIGURO_BUILD_PROFILE) -b $(CONFIG_BLD_XDC_$(ISA)) --ol $(LNKCMD_FILE) $(XDC_CFG_FILE_NAME)
	$(ECHO) \# Configuro done!

ifneq ($(XDC_CFG_FILE_$(CORE)),)
  ifndef MODULE_NAME
$(OBJDIR)/$(CFG_COBJ_XDC) : $(CFG_C_XDC)
	$(ECHO) \# Compiling generated $(CFG_COBJ_XDC)
	$(CC) -ppd=$(DEPFILE).P $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) -fc $(CFG_C_XDC)
	$(CC) $(_CFLAGS) $(INCLUDES) $(CFLAGS_DIROPTS) -fc $(CFG_C_XDC)
  endif
endif

# Include dependency make files that were generated by $(CC)
-include $(SRCS:%.c=$(DEPDIR)/%.P)

# Nothing beyond this point
