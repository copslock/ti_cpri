#
# Copyright (c) 2018, Texas Instruments Incorporated
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

# File: iolink_component.mk
#       This file is component include make file of IOLINK library.
# List of variables set in this file and their purpose:
# <mod>_RELPATH        - This is the relative path of the module, typically from
#                        top-level directory of the package
# <mod>_PATH           - This is the absolute path of the module. It derives from
#                        absolute path of the top-level directory (set in env.mk)
#                        and relative path set above
# <mod>_INCLUDE        - This is the path that has interface header files of the
#                        module. This can be multiple directories (space separated)
# <mod>_PKG_LIST       - Names of the modules (and sub-modules) that are a part
#                        part of this module, including itself.
# <mod>_BOARD_DEPENDENCY - "yes": means the code for this module depends on
#                             board and the compiled obj/lib has to be kept
#                             under <board> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no board dependent code and hence
#                             the obj/libs are not kept under <board> dir.
# <mod>_CORE_DEPENDENCY     - "yes": means the code for this module depends on
#                             core and the compiled obj/lib has to be kept
#                             under <core> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no core dependent code and hence
#                             the obj/libs are not kept under <core> dir.
# <mod>_APP_STAGE_FILES     - List of source files that belongs to the module
#                             <mod>, but that needs to be compiled at application
#                             build stage (in the context of the app). This is
#                             primarily for link time configurations or if the
#                             source file is dependent on options/defines that are
#                             application dependent. This can be left blank or
#                             not defined at all, in which case, it means there
#                             no source files in the module <mod> that are required
#                             to be compiled in the application build stage.
#
ifeq ($(iolink_component_make_include), )

drviolink_BOARDLIST       = idkAM437x
drviolink_SOCLIST         = am437x
drviolink_am437x_CORELIST = a9host pru_0 pru_1

############################
# iolink package
# List of components included under iolink lib
# The components included here are built and will be part of iolink lib
############################
iolink_LIB_LIST = iolink iolink_indp iolink_profile iolink_profile_indp
drviolink_LIB_LIST = $(iolink_LIB_LIST)

############################
# iolink Firmwares
# All the firmware mentioned in list are built when build target is called
# List below all firmware for allowed values
############################
iolink_FIRM_LIST = icss_iolink
drviolink_FIRM_LIST = $(iolink_FIRM_LIST)

############################
# iolink examples
# List of examples under iolink
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
ifeq ($(CORE), a9host)
ifneq ($(IOLINK_STACK_INSTALL_PATH), )
iolink_EXAMPLE_LIST = IOLINK_Stack_TestApp
endif
drviolink_EXAMPLE_LIST = $(iolink_EXAMPLE_LIST)
endif

#
# IOLINK Modules
#

# IOLINK LIB
iolink_COMP_LIST = iolink
iolink_RELPATH = ti/drv/iolink
iolink_PATH = $(PDK_IOLINK_COMP_PATH)
iolink_LIBNAME = ti.drv.iolink
export iolink_LIBNAME
iolink_LIBPATH = $(iolink_PATH)/lib
export iolink_LIBPATH
iolink_OBJPATH = $(iolink_RELPATH)/iolink
export iolink_OBJPATH
iolink_MAKEFILE = -f build/makefile.mk
export iolink_MAKEFILE
iolink_BOARD_DEPENDENCY = no
iolink_CORE_DEPENDENCY = no
iolink_SOC_DEPENDENCY = yes
export iolink_COMP_LIST
export iolink_BOARD_DEPENDENCY
export iolink_CORE_DEPENDENCY
export iolink_SOC_DEPENDENCY
iolink_PKG_LIST = iolink
export iolink_PKG_LIST
iolink_INCLUDE = $(iolink_PATH)
iolink_SOCLIST = $(drviolink_SOCLIST)
export iolink_SOCLIST
iolink_$(SOC)_CORELIST = $(drviolink_$(SOC)_CORELIST)
export iolink_$(SOC)_CORELIST

# IOLINK INDEPENDENT LIB
iolink_indp_COMP_LIST = iolink_indp
iolink_indp_RELPATH = ti/drv/iolink
iolink_indp_PATH = $(PDK_IOLINK_COMP_PATH)
iolink_indp_LIBNAME = ti.drv.iolink
export iolink_indp_LIBNAME
iolink_indp_LIBPATH = $(iolink_indp_PATH)/lib
export iolink_indp_LIBPATH
iolink_indp_OBJPATH = $(iolink_indp_RELPATH)/iolink_indp
export iolink_indp_OBJPATH
iolink_indp_MAKEFILE = -f build/makefile_indp.mk
export iolink_indp_MAKEFILE
iolink_indp_BOARD_DEPENDENCY = no
iolink_indp_CORE_DEPENDENCY = no
iolink_indp_SOC_DEPENDENCY = no
export iolink_indp_COMP_LIST
export iolink_indp_BOARD_DEPENDENCY
export iolink_indp_CORE_DEPENDENCY
export iolink_indp_SOC_DEPENDENCY
iolink_indp_PKG_LIST = iolink_indp
export iolink_indp_PKG_LIST
iolink_indp_INCLUDE = $(iolink_indp_PATH)
iolink_indp_SOCLIST = $(drviolink_SOCLIST)
export iolink_indp_SOCLIST
iolink_indp_$(SOC)_CORELIST = $(drviolink_$(SOC)_CORELIST)
export iolink_indp_$(SOC)_CORELIST

# IOLINK PROFILE LIB
iolink_profile_COMP_LIST = iolink_profile
iolink_profile_RELPATH = ti/drv/iolink
iolink_profile_PATH = $(PDK_IOLINK_COMP_PATH)
iolink_profile_LIBNAME = ti.drv.iolink.profiling
export iolink_profile_LIBNAME
iolink_profile_LIBPATH = $(iolink_profile_PATH)/lib
export iolink_profile_LIBPATH
iolink_profile_OBJPATH = $(iolink_profile_RELPATH)/iolink_profile
export iolink_profile_OBJPATH
iolink_profile_MAKEFILE = -f build/makefile_profile.mk
export iolink_profile_MAKEFILE
iolink_profile_BOARD_DEPENDENCY = no
iolink_profile_CORE_DEPENDENCY = no
iolink_profile_SOC_DEPENDENCY = yes
export iolink_profile_COMP_LIST
export iolink_profile_BOARD_DEPENDENCY
export iolink_profile_CORE_DEPENDENCY
export iolink_profile_SOC_DEPENDENCY
iolink_profile_PKG_LIST = iolink_profile
export iolink_profile_PKG_LIST
iolink_profile_INCLUDE = $(iolink_profile_PATH)
iolink_profile_SOCLIST = $(drviolink_SOCLIST)
export iolink_profile_SOCLIST
iolink_profile_$(SOC)_CORELIST = $(drviolink_$(SOC)_CORELIST)
export iolink_profile_$(SOC)_CORELIST

# IOLINK PROFILE INDEPENDENT LIB
iolink_profile_indp_COMP_LIST = iolink_profile_indp
iolink_profile_indp_RELPATH = ti/drv/iolink
iolink_profile_indp_PATH = $(PDK_IOLINK_COMP_PATH)
iolink_profile_indp_LIBNAME = ti.drv.iolink.profiling
export iolink_profile_indp_LIBNAME
iolink_profile_indp_LIBPATH = $(iolink_profile_indp_PATH)/lib
export iolink_profile_indp_LIBPATH
iolink_profile_indp_OBJPATH = $(iolink_profile_indp_RELPATH)/iolink_profile_indp
export iolink_profile_indp_OBJPATH
iolink_profile_indp_MAKEFILE = -f build/makefile_profile_indp.mk
export iolink_profile_indp_MAKEFILE
iolink_profile_indp_BOARD_DEPENDENCY = no
iolink_profile_indp_CORE_DEPENDENCY = no
iolink_profile_indp_SOC_DEPENDENCY = no
export iolink_profile_indp_COMP_LIST
export iolink_profile_indp_BOARD_DEPENDENCY
export iolink_profile_indp_CORE_DEPENDENCY
export iolink_profile_indp_SOC_DEPENDENCY
iolink_profile_indp_PKG_LIST = iolink_profile_indp
export iolink_profile_indp_PKG_LIST
iolink_profile_indp_INCLUDE = $(iolink_profile_indp_PATH)
iolink_profile_indp_SOCLIST = $(drviolink_SOCLIST)
export iolink_profile_indp_SOCLIST
iolink_profile_indp_$(SOC)_CORELIST = $(drviolink_$(SOC)_CORELIST)
export iolink_profile_indp_$(SOC)_CORELIST

#
# IOLINK Firmwares
#
icss_iolink_COMP_LIST = icss_iolink
# temporary fix for nightly build
# icss_iolink_RELPATH = ti/drv/iolink/firmware/icss_iolink
icss_iolink_RELPATH = icss_iolink
icss_iolink_PATH = $(PDK_IOLINK_COMP_PATH)/firmware/icss_iolink
icss_iolink_HEADERNAME = icss_iolink
export icss_iolink_HEADERNAME
icss_iolink_HEADERPATH = $(icss_iolink_PATH)/bin
export icss_iolink_HEADERPATH
icss_iolink_OBJPATH = $(icss_iolink_RELPATH)
export icss_iolink_OBJPATH
icss_iolink_MAKEFILE = -f ../../build/makefile_icss_iolink.mk
export icss_iolink_MAKEFILE
icss_iolink_BOARD_DEPENDENCY = no
icss_iolink_CORE_DEPENDENCY = yes
icss_iolink_SOC_DEPENDENCY = yes
export icss_iolink_COMP_LIST
export icss_iolink_BOARD_DEPENDENCY
export icss_iolink_CORE_DEPENDENCY
export icss_iolink_SOC_DEPENDENCY
icss_iolink_PKG_LIST = icss_iolink
export icss_iolink_PKG_LIST
icss_iolink_INCLUDE = $(icss_iolink_PATH)
icss_iolink_SOCLIST = am437x
export icss_iolink_SOCLIST
#icss_iolink_$(SOC)_CORELIST = $(drviolink_$(SOC)_CORELIST)
icss_iolink_$(SOC)_CORELIST = pru_0
export icss_iolink_$(SOC)_CORELIST

#
# IOLINK Examples
#

# IOLINK rtos test
ifeq ($(CORE), a9host)
IOLINK_Stack_TestApp_COMP_LIST = IOLINK_Stack_TestApp
IOLINK_Stack_TestApp_RELPATH = $(PDK_IOLINK_COMP_PATH)/test/stack_test
IOLINK_Stack_TestApp_PATH = $(PDK_IOLINK_COMP_PATH)/test/stack_test
IOLINK_Stack_TestApp_BOARD_DEPENDENCY = yes
IOLINK_Stack_TestApp_CORE_DEPENDENCY = no
IOLINK_Stack_TestApp_MAKEFILE = -f makefile
IOLINK_Stack_TestApp_XDC_CONFIGURO = yes
export IOLINK_Stack_TestApp_COMP_LIST
export IOLINK_Stack_TestApp_BOARD_DEPENDENCY
export IOLINK_Stack_TestApp_CORE_DEPENDENCY
export IOLINK_Stack_TestApp_XDC_CONFIGURO
export IOLINK_Stack_TestApp_MAKEFILE
IOLINK_Stack_TestApp_PKG_LIST = IOLINK_Stack_TestApp
IOLINK_Stack_TestApp_INCLUDE = $(IOLINK_Stack_TestApp_PATH)
IOLINK_Stack_TestApp_BOARDLIST = $(drviolink_BOARDLIST)
export IOLINK_Stack_TestApp_BOARDLIST
IOLINK_Stack_TestApp_$(SOC)_CORELIST = $(iolink_$(SOC)_CORELIST)
export IOLINK_Stack_TestApp_$(SOC)_CORELIST
endif

export drviolink_LIB_LIST
export drviolink_EXAMPLE_LIST
export drviolink_FIRM_LIST
export iolink_FIRM_LIST
export iolink_LIB_LIST
export iolink_EXAMPLE_LIST

iolink_component_make_include := 1
endif
