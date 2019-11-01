#
# This file is the makefile for building Sciclient ccs init test application.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = sciclient_ccs_init

SRCDIR = .
INCDIR = .

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk sciclient osal_nonos board uart

# List all the components required by the application
COMP_LIST_COMMON  = sciclient csl osal_nonos csl_init board uart i2c

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON = sciclient_ccs_init_main.c

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS)
PACKAGE_SRCS_COMMON = . ../common

CONFIG_BLD_LNK_r5f = linker_r5.lds

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<coqre/SoC/platform-name> =
#   CFLAGS_LOCAL_<core/SoC/platform-name> =

# Include common make files
ifeq ($(MAKERULEDIR), )
#Makerule path not defined, define this and assume relative path from ROOTDIR
  MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
  export MAKERULEDIR
endif
include $(MAKERULEDIR)/common.mk

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
