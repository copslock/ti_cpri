#
# This file is the makefile for building fvid2.
#
include $(PDK_INSTALL_PATH)/ti/build/Rules.make

MODULE_NAME = fvid2

SRCDIR = src
INCDIR = . include src

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# Common source files and CFLAGS across all platforms and cores
PACKAGE_SRCS_COMMON = .

SRCS_COMMON += fvid2_drvMgr.c fvid2_utils.c fvid2_graph.c trace.c

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(FVID2_CFLAGS)
PACKAGE_SRCS_COMMON = fvid2.h fvid2_component.mk makefile .gitignore include src docs
PACKAGE_SRCS_COMMON += config_mk.bld package.bld package.xdc package.xs
PACKAGE_SRCS_COMMON += Settings.xdc.xdt fvid2ver.h fvid2ver.h.xdt

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
