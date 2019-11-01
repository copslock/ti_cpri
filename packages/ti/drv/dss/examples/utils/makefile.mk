#
# This file is the makefile for building DSS app utils library.
#

SRCDIR = .
INCDIR = .

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES += pdk
INCLUDE_INTERNAL_INTERFACES = csl

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON += app_utils_$(SOC).c

PACKAGE_SRCS_COMMON = .
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(DSS_CFLAGS)

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

