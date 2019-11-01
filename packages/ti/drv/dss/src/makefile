#
# This file is the makefile for building DSS driver library.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

MODULE_NAME = dss

SRCDIR = .
INCDIR = .
ifeq ($(SOC),$(filter $(SOC), am65xx))
  SRCDIR += soc/V0
  INCDIR += soc/V0
  PACKAGE_SRCS_COMMON = soc/V0
endif
ifeq ($(SOC),$(filter $(SOC), j721e))
  SRCDIR += soc/V1
  INCDIR += soc/V1
  PACKAGE_SRCS_COMMON = soc/V1
  CFLAGS_LOCAL_COMMON = -DREF_CLK_19_2MHz -DHAVE_CMN_PLL1
endif

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk
INCLUDE_INTERNAL_INTERFACES = csl

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON +=
include soc/src_files_soc.mk
include src/csl/src_files_csl.mk
include src/drv/src_files_drv.mk

PACKAGE_SRCS_COMMON += dss.h dss_component.mk makefile .gitignore include docs src/makefile
PACKAGE_SRCS_COMMON += soc/dss_soc.h soc/src_files_soc.mk src/csl/src_files_csl.mk src/drv/src_files_drv.mk
PACKAGE_SRCS_COMMON += config_mk.bld package.bld package.xdc package.xs
PACKAGE_SRCS_COMMON += Settings.xdc.xdt dssver.h dssver.h.xdt
CFLAGS_LOCAL_COMMON += $(PDK_CFLAGS) $(DSS_CFLAGS)

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
