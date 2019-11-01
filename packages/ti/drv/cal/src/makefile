#
# This file is the makefile for building CAL driver library.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

MODULE_NAME = cal

SRCDIR = src src/hal src/core src/drv
INCDIR = .

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk
INCLUDE_INTERNAL_INTERFACES =

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON +=
include src/hal/src_files_hal.mk
include src/core/src_files_core.mk
include src/drv/src_files_drv.mk

PACKAGE_SRCS_COMMON = cal.h cal_component.mk makefile .gitignore include src docs
PACKAGE_SRCS_COMMON += config_mk.bld package.bld package.xdc package.xs
PACKAGE_SRCS_COMMON += Settings.xdc.xdt calver.h calver.h.xdt

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(CAL_CFLAGS)

# Core/SoC/platform specific source files and CFLAGS
# Example:
#   SRCS_<core/SoC/platform-name> =
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
