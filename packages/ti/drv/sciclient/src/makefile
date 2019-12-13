#
# This file is the makefile for building sciclient .
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

MODULE_NAME = sciclient

SRCDIR = src
INCDIR = soc/sysfw/include
PACKAGE_SRCS_COMMON =

# SoC Specific source files
ifeq ($(SOC),$(filter $(SOC), am65xx))
  SRCDIR += soc/V0
  INCDIR += soc/V0
  GEN_FILE = soc/V0/sciclient_firmware_V0.h
endif
ifeq ($(SOC),$(filter $(SOC), j721e j7200))
  SRCDIR += soc/V1
  INCDIR += soc/V1
  GEN_FILE = soc/V1/sciclient_firmware_V1.h
endif

# List all the external components/interfaces, whose interface header files
# need to be included for this component
INCLUDE_EXTERNAL_INTERFACES = pdk osal

# Common source files and CFLAGS across all platforms and cores
ifeq ($(SOC),$(filter $(SOC), am65xx j721e j7200))
# Keep the sciclient.c the first file in this list. This is used to trigger
# the generation of the header files from the firware binary file.
SRCS_COMMON  = sciclient.c
SRCS_COMMON += sciclient_fmwSecureProxyMap.c
SRCS_COMMON += sciclient_pm.c
SRCS_COMMON += sciclient_rm.c 
SRCS_COMMON += sciclient_boardcfg.c
SRCS_COMMON += sciclient_defaultBoardcfg.c
SRCS_COMMON += sciclient_procboot.c
SRCS_COMMON += sciclient_genericMsgs.c
endif

#Use CSL implementation to bypass DMSC
# ifeq ($(SOC),$(filter $(SOC), j721e j7200))
# SRCS_COMMON = sciclient_rm_csl.c sciclient_dummy.c
# endif

#Bypass DMSC for host emulation and Loki build
ifeq ($(BOARD),$(filter $(BOARD), j721e_hostemu j721e_loki))
 SRCS_COMMON = sciclient_rm_csl.c sciclient_dummy.c
endif

CFLAGS_LOCAL_COMMON = $(PDK_CFLAGS)
PACKAGE_SRCS_COMMON += sciclient.h sciclient_component.mk makefile .gitignore
PACKAGE_SRCS_COMMON += include docs tools $(SRCDIR) soc/sciclient_soc_priv.h
PACKAGE_SRCS_COMMON += config_mk.bld package.bld package.xdc package.xs
PACKAGE_SRCS_COMMON += Settings.xdc.xdt sciclientver.h sciclientver.h.xdt

# Selectively package sysfw components
PACKAGE_SRCS_COMMON += soc/sysfw/binaries/system-firmware-public-documentation
PACKAGE_SRCS_COMMON += soc/sysfw/include
PACKAGE_SRCS_COMMON += soc/sysfw/binaries/ti-sci-firmware-am65x-gp.bin
PACKAGE_SRCS_COMMON += soc/sysfw/binaries/ti-sci-firmware-j721e-gp.bin
ifeq ($(BUILD_HS),yes)
PACKAGE_SRCS_COMMON += soc/sysfw/binaries/ti-sci-firmware-am65x-hs.bin
endif


FIRMWARE_HEADER_GEN=tools/firmwareHeaderGen.sh
ifeq ($(BUILD_HS),yes)
	FIRMWARE_HEADER_GEN_SUFFIX=-hs
else
	FIRMWARE_HEADER_GEN_SUFFIX=
endif

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

$(SRCS): $(GEN_FILE)

$(GEN_FILE):
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(FIRMWARE_HEADER_GEN)
endif
	$(FIRMWARE_HEADER_GEN) $(SOC)$(FIRMWARE_HEADER_GEN_SUFFIX)

# OBJs and libraries are built by using rule defined in rules_<target>.mk
#     and need not be explicitly specified here

# Nothing beyond this point
