#
# This file is the makefile for building UDMA driver library.
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

MODULE_NAME = udma

SRCDIR = src
ifeq ($(SOC),$(filter $(SOC), am65xx))
  SRCDIR += soc/V0
endif
ifeq ($(SOC),$(filter $(SOC), j721e))
  SRCDIR += soc/V1
endif
INCDIR =

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk

# Common source files and CFLAGS across all platforms and cores
SRCS_COMMON += udma.c
SRCS_COMMON += udma_rm.c udma_rmcfg.c
SRCS_COMMON += udma_ch.c udma_dru.c
SRCS_COMMON += udma_ring.c udma_proxy.c udma_flow.c udma_event.c
SRCS_COMMON += udma_osal.c udma_utils.c
SRCS_COMMON += udma_soc.c

PACKAGE_SRCS_COMMON = udma.h udma_component.mk makefile .gitignore include docs $(SRCDIR)
PACKAGE_SRCS_COMMON += soc/udma_soc.h
PACKAGE_SRCS_COMMON += config_mk.bld package.bld package.xdc package.xs
PACKAGE_SRCS_COMMON += Settings.xdc.xdt udmaver.h udmaver.h.xdt
CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS) $(UDMA_CFLAGS)

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
