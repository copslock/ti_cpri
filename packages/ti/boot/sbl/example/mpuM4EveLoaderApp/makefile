# Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
#
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# Macro definitions referenced below
#

#
# Make rules for MPU-M4-EVE-Loader application
#
# Generates the Multicore application that comprises of MPU and IPU with EVE firmware
# embedded inside IPU binary.

# Device ID & CPU ID should be in sync with SBL. Refer to SBL user guide for values
DEV_ID=55
MPU_CPU0_ID=0
IPU1_CPU0_ID=2

SBL_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/boot/sbl

# Multicore App based on ENDIAN: little/big
ENDIANNESS:=LE

#List of binary path to create multicore application
MPUMULTICOREBINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/mpuM4EveLoaderApp/mpuMulticore/a15/mpuc0/bin
EVE_LOADER_M4_BINDIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/mpuM4EveLoaderApp/ipuEveLoader/ipu1/bin
MPU_M4_EVE_LOADER_DIR = $(SBL_SRC_DIR)/binary/$(BOARD)/example/mpuM4EveLoaderApp

#SBL RPRC generation tool path
ifeq ($(OS),Windows_NT)
OUT2RPRC            = $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/out2rprc/bin/out2rprc.exe
else
OUT2RPRC            = mono $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/out2rprc/bin/out2rprc.exe
endif

#Multicore image generation tool path
MULTICORE_IMAGE_GEN = $(PDK_INSTALL_PATH)/ti/boot/sbl/tools/multicoreImageGen/bin/MulticoreImageGen

#Binary names to create Multicore Application
MPU_EXE=sbl_app.out
IPU_EXE=sbl_eveLoader.xem4

INPUTRPRCS = $(MPU_CPU0_ID) $(MPU_M4_EVE_LOADER_DIR)/bin/$(MPU_EXE).rprc
INPUTRPRCS += $(IPU1_CPU0_ID) $(MPU_M4_EVE_LOADER_DIR)/bin/$(IPU_EXE).rprc

#Generates the Multicore App to load the MPU and IPU binaries.
eveLoader: create_rprc $(MPU_M4_EVE_LOADER_DIR)/.created
	$(MULTICORE_IMAGE_GEN) $(ENDIANNESS) $(DEV_ID) $(MPU_M4_EVE_LOADER_DIR)/bin/app $(INPUTRPRCS)

#Generates RPRC intermediate file to generate the APP.
create_rprc: create_image
	$(OUT2RPRC) $(MPUMULTICOREBINDIR)/$(MPU_EXE) $(MPU_M4_EVE_LOADER_DIR)/bin/$(MPU_EXE).rprc
	$(OUT2RPRC) $(EVE_LOADER_M4_BINDIR)/$(IPU_EXE) $(MPU_M4_EVE_LOADER_DIR)/bin/$(IPU_EXE).rprc

#Creates EVE IPU and MPU binaries of EVE Loader Application.
create_image:
	@$(MAKE) -f ./eve1MulticoreApp/makefile eveFirmware
	@$(MAKE) -f ./ipu1EveLoaderApp/makefile ipuEveLoader
	@$(MAKE) -f ./mpuMulticoreApp/makefile mpuMulticore

eveLoader_clean:
	@rm -rf $(MPU_M4_EVE_LOADER_DIR)/bin/*
	@$(MAKE) -f ./eve1MulticoreApp/makefile eveFirmware_clean
	@$(MAKE) -f ./ipu1EveLoaderApp/makefile ipuEveLoader_clean
	@$(MAKE) -f ./mpuMulticoreApp/makefile mpuMulticore_clean

$(MPU_M4_EVE_LOADER_DIR)/.created:
	@cd $(MPU_M4_EVE_LOADER_DIR)
	@mkdir -p $(MPU_M4_EVE_LOADER_DIR)/bin
	@touch $(MPU_M4_EVE_LOADER_DIR)/bin/.created
