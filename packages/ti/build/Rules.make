#
# Copyright (c) 2013-2018, Texas Instruments Incorporated
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

#Default build environment, (Windows_NT / linux)
#if nothing is defined, default to linux as in windows this variable is defined
export OS ?= linux

SDK_INSTALL_PATH ?= $(abspath ../../../../)
TOOLS_INSTALL_PATH ?= $(SDK_INSTALL_PATH)

#Default BUILD_OS_TYPE (tirtos/baremetal/qnx)
export BUILD_OS_TYPE ?= tirtos

#if PDK_SOC is specified , derive LIMIT_SOCS from there (if LIMIT_SOCS not defined)
ifneq ($(PDK_SOC),)

LIMIT_CORES_am335x    = $(CORE_LIST_am335x)
LIMIT_CORES_am437x    = $(CORE_LIST_am437x)
LIMIT_CORES_am57xx    = $(sort $(CORE_LIST_am571x) $(CORE_LIST_am572x) $(CORE_LIST_am574x))
LIMIT_CORES_omapl137  = $(CORE_LIST_omapl137)
LIMIT_CORES_omapl138  = $(CORE_LIST_omapl138)
LIMIT_CORES_k2hk      = $(sort $(CORE_LIST_k2h) $(CORE_LIST_k2k))
LIMIT_CORES_k2e       = $(CORE_LIST_k2e)
LIMIT_CORES_k2l       = $(CORE_LIST_k2l)
LIMIT_CORES_k2g       = $(CORE_LIST_k2g)
LIMIT_CORES_k2g-hs    = $(CORE_LIST_k2g)
LIMIT_CORES_c665x     = $(CORE_LIST_c6657)
LIMIT_CORES_c667x     = $(CORE_LIST_c6678)
LIMIT_CORES_am65xx    = $(CORE_LIST_am65xx)
LIMIT_CORES_am65xx-hs = $(CORE_LIST_am65xx)
# Filter out c7x-hostemu as Processor SDK does not build use it
LIMIT_CORES_j7        = $(filter-out c7x-hostemu,$(sort $(CORE_LIST_j721e) $(CORE_LIST_j7200)))

export LIMIT_CORES ?= $(LIMIT_CORES_$(PDK_SOC))

LIMIT_SOCS_k2g       = k2g
LIMIT_SOCS_k2g-hs    = k2g
LIMIT_SOCS_k2hk      = k2hk
LIMIT_SOCS_k2e       = k2e
LIMIT_SOCS_k2l       = k2l
LIMIT_SOCS_am57xx    = am571x am572x am574x
LIMIT_SOCS_am437x    = am437x
LIMIT_SOCS_am335x    = am335x
LIMIT_SOCS_am65xx    = am65xx
LIMIT_SOCS_am65xx-hs = am65xx
LIMIT_SOCS_j7        = j721e j7200

export LIMIT_SOCS ?= $(LIMIT_SOCS_$(PDK_SOC))

LIMIT_BOARDS_j7        = $(BOARD_LIST_j721e) $(BOARD_LIST_j7200)
LIMIT_BOARDS_am335x    = $(BOARD_LIST_am335x)
LIMIT_BOARDS_omapl137  = $(BOARD_LIST_omapl137)
LIMIT_BOARDS_k2l       = $(BOARD_LIST_k2l)
LIMIT_BOARDS_am437x    = $(BOARD_LIST_am437x)
LIMIT_BOARDS_am437x-hs = $(BOARD_LIST_am437x)
LIMIT_BOARDS_k2hk      = $(BOARD_LIST_k2h) $(BOARD_LIST_k2k)
LIMIT_BOARDS_k2g       = $(BOARD_LIST_k2g)
LIMIT_BOARDS_k2g-hs    = $(BOARD_LIST_k2g)
LIMIT_BOARDS_k2e       = $(BOARD_LIST_k2e)
LIMIT_BOARDS_am65xx    = $(BOARD_LIST_am65xx)
LIMIT_BOARDS_am65xx-hs = $(BOARD_LIST_am65xx)
LIMIT_BOARDS_c665x     = $(BOARD_LIST_c6657)
LIMIT_BOARDS_c667x     = $(BOARD_LIST_c6678)
LIMIT_BOARDS_omapl138  = $(BOARD_LIST_omapl138)
LIMIT_BOARDS_am57xx    = $(BOARD_LIST_am571x) $(BOARD_LIST_am572x) $(BOARD_LIST_am574x)

export LIMIT_BOARDS ?= $(LIMIT_BOARDS_$(PDK_SOC))

endif
# Default board
# Supported values are printed in "make -s help" option. Below are the list for reference.
#                   evmDRA72x, evmDRA75x, evmDRA78x,
#                   evmAM572x, idkAM571x, idkAM572x idkAM574x
#                   evmK2H, evmK2K, evmK2E, evmK2L, evmK2G, evmC6678, evmC6657,
#                   evmAM335x, icev2AM335x, iceAMIC110, skAM335x, bbbAM335x,
#                   evmAM437x idkAM437x skAM437x evmOMAPL137 lcdkOMAPL138

################################################################################
# Other user configurable variables
################################################################################

#if LIMIT_BOARDS if it is defined
ifneq ($(LIMIT_BOARDS),)
BOARD ?= $(firstword $(LIMIT_BOARDS))
else
#if LIMIT_BOARDS is not defined, default BOARD and SOC to the below
export BOARD ?= j721e_evm
export SOC ?= j721e
endif
# Default to m4 build depending on BOARD selected!!
ifeq ($(BOARD),$(filter $(BOARD), evmAM572x idkAM572x idkAM571x idkAM574x))
  CORE ?= a15_0
endif
ifeq ($(BOARD),$(filter $(BOARD), am65xx_sim am65xx_evm am65xx_idk j721e_sim j721e_vhwazebu j721e_qt j721e_evm j7200_evm j7200_sim am64x_evm))
  CORE ?= mcu1_0
endif
ifeq ($(BOARD),$(filter $(BOARD), j721e_ccqt j721e_loki))
  CORE ?= c7x_1
endif
ifeq ($(BOARD),$(filter $(BOARD), j721e_hostemu j7200_hostemu))
  CORE = c7x-hostemu
endif
CORE ?= ipu1_0
export CORE

# Default Build Profile
# Supported Values: debug | release
export BUILD_PROFILE ?= release

# Treat compiler warning as error
# Supported Values: yes | no
export TREAT_WARNINGS_AS_ERROR ?= yes

#Various boards support for J7 TDA family of devices
BOARD_LIST_J7_TDA = j721e_sim j721e_hostemu j721e_ccqt j721e_loki j721e_qt j721e_vhwazebu j721e_evm
BOARD_LIST_J7_TDA += j7200_sim j7200_hostemu j7200_evm am64x_evm
export BOARD_LIST_J7_TDA


################################################################################
# Other advanced configurable variables
################################################################################

# Set Core Build Profile depending on BUILD_PROFILE value
export BUILD_PROFILE_$(CORE) ?= $(BUILD_PROFILE)

# Default PACKAGE_SELECT build flag
# Supported values: all, vps-hal-only, vps-vip-only, vps-vpe-only, vps-dss-only, vps-vip-dss, vps-vip-vpe
export PACKAGE_SELECT ?= all

# Disable recursive building of example dependencies
export DISABLE_RECURSE_DEPS ?= no

# Default C++ build flag, yes or no
export CPLUSPLUS_BUILD ?= no

include $(PDK_INSTALL_PATH)/ti/build/pdk_tools_path.mk

#use <module>_PATH variable as makefile internally expects PATH variable this way for external component path
export pdk_PATH := $(PDK_INSTALL_PATH)
export bios_PATH := $(BIOS_INSTALL_PATH)
export xdc_PATH := $(XDC_INSTALL_PATH)
export edma3_lld_PATH := $(EDMA3LLD_BIOS6_INSTALLDIR)
export ndk_PATH := $(NDK_INSTALL_PATH)
export radarLink_PATH := $(RADARLINK_INSTALL_PATH)
export ipc_PATH := $(IPC_INSTALL_PATH)
export uia_PATH := $(UIA_INSTALL_PATH)

export ROOTDIR := $(pdk_PATH)
XDCPATH = 
ifeq ($(BUILD_OS_TYPE),tirtos)
  XDCPATH = $(bios_PATH)/packages;$(xdc_PATH)/packages;$(edma3_lld_PATH)/packages;$(ndk_PATH)/packages;$(pdk_PATH);$(ipc_PATH)/packages;$(uia_PATH)/packages;
endif
export XDCPATH

#Default SECTTI SIZE INFORMATION
export SECTTI_SIZE_INFO ?= no

#Default SECTTI tool
export SECTTI ?= $(CG_XML_BIN_INSTALL_PATH)/sectti

# Build for HS devices if secdev package is found
ifneq ("$(wildcard $(TI_SECURE_DEV_PKG)/scripts/secure-binary-image.sh)","")
  export SECUREMODE ?= yes
endif

# include other dependent files
include $(PDK_INSTALL_PATH)/ti/build/comp_paths.mk
ifeq ($(MAKERULEDIR), )
  #Makerule path not defined, define this and assume relative path from ROOTDIR
  export MAKERULEDIR := $(ROOTDIR)/ti/build/makerules
endif
include $(MAKERULEDIR)/build_config.mk
include $(MAKERULEDIR)/platform.mk
include $(MAKERULEDIR)/env.mk

export PRUCORE_LIST = $(CORE_LIST_PRU)

################################################################################
# Build Tools Configuration
################################################################################

ifeq ($(OS),Windows_NT)
  PATH := $(PATH)
endif

# Compiler Tools:
# PATH := $(C6X_GEN_INSTALL_PATH)/bin;$(PATH)

# XDC Tools location:
PATH := $(XDC_INSTALL_PATH);$(XDC_INSTALL_PATH)/bin;$(XDC_INSTALL_PATH)/packages/xdc/services/io/release;$(PATH)

ifeq ($(OS),Windows_NT)
  PATH := $(subst /,\,$(PATH))
else
  PATH := $(subst ;,:,$(PATH))
endif

export PATH

LIBDIR ?= ./lib
export LIBDIR
