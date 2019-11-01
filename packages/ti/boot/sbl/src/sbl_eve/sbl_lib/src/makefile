#
# This file is the makefile for building sbl library.
#
ifndef SOC
SOC = am572x
endif

ifndef BOARD
BOARD = idkAM572x
endif

CORE = ipu1_0
ENDIAN = little
ENDIAN_EXT = 
MODULE_NAME = sbl_lib

include $(PDK_INSTALL_PATH)/ti/build/Rules.make

# Include common make files
ifeq ($(MAKERULEDIR), )
#Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif

SBL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl
$(MODULE_NAME)_OBJPATH = $(MODULE_NAME)
$(MODULE_NAME)_LIBPATH = $(SBL_SRC_DIR)/lib

include $(MAKERULEDIR)/platform.mk

DEPDIR = $(OBJDIR)/.deps
DEPFILE = $(DEPDIR)/$(*F)

BUILD_PROFILE_$(CORE) = release
$(MODULE_NAME)_BOARD_DEPENDENCY = yes

ifeq ($(SOC),$(filter $(SOC), AM572x AM574x))
SRCDIR = . $(SBL_SRC_DIR)/src/sbl_eve/sbl_lib/src $(SBL_SRC_DIR)/src/sbl_eve/sbl_lib/src/am57xx
INCDIR = . $(SBL_SRC_DIR)/src/sbl_eve $(SBL_SRC_DIR)/src/sbl_eve/sbl_lib/src $(SBL_SRC_DIR)/src/sbl_eve/sbl_lib/src/am57xx
endif

VPATH := $(SRCDIR)

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk
INCLUDE_INTERNAL_INTERFACES = csl

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(SBL_CFLAGS)
PACKAGE_SRCS_COMMON = . 

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sbl_lib_common.c sbl_lib_board.c sbl_lib_weak.c
SRCS_AM572x = sbl_lib_platform.c sbl_lib_prcm_dpll.c
SRCS_AM574x = sbl_lib_platform.c sbl_lib_prcm_dpll.c

SRCS = $(SRCS_COMMON) $(SRCS_$(SOC))

LIBNAME = $(MODULE_NAME)


include $(MAKERULEDIR)/common.mk

$(MODULE_NAME):$(LIBDIR)/$(LIBNAME).$(LIBEXT)


$(MODULE_NAME)_clean :
	$(RM) -rf $(OBJDIR)/* $(DEPDIR)/* $(LIBDIR)/*
 
# Nothing beyond this point
