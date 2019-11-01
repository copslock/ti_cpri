#
# Copyright (c) 2018, Texas Instruments Incorporated
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

# Get directory of makefile
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
mkfile_dir := $(dir $(mkfile_path))

# Setup build variables relative to makefile folder
ifeq ($(SDK_INSTALL_PATH),)
PDK_INSTALL_PATH ?= $(abspath $(mkfile_dir)../../../..)
SDK_INSTALL_PATH ?= $(abspath $(PDK_INSTALL_PATH)../../..)
else
PDK_INSTALL_PATH ?= $(mkfile_dir)../../../..
endif


# Strip drive names if present, to prevent build breaks
STRIP_DRIVE_NAME := $(subst :,: ,$(PDK_INSTALL_PATH))
DRIVE_LETTER := $(firstword $(STRIP_DRIVE_NAME))

export SBL_DRIVE_LETTER := $(subst :,,$(DRIVE_LETTER))

ifeq ($(wildcard $(PDK_INSTALL_PATH)/Rules.make),)
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(PDK_INSTALL_PATH)/Rules.make
endif

COMP = sbl

lib_$(COMP)_BUILD_DEPENDENCY = board

$(COMP)_DOXYGEN_SUPPORT = no

include $(PDK_INSTALL_PATH)/ti/build/comp_top.mk
