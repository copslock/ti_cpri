#
# Copyright (c) 2016-2019, Texas Instruments Incorporated
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

# File: pcie_component.mk
#       This file is component include make file of PCIE library.
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
ifeq ($(pcie_component_make_include), )

# under other list
drvpcie_BOARDLIST       = am65xx_evm am65xx_idk
drvpcie_SOCLIST         = am574x am572x am571x k2h k2k k2l k2e k2g c6678 c6657 am65xx
drvpcie_am574x_CORELIST = c66x a15_0 ipu1_0
drvpcie_am572x_CORELIST = c66x a15_0 ipu1_0
drvpcie_am571x_CORELIST = c66x a15_0 ipu1_0
drvpcie_k2h_CORELIST    = c66x a15_0
drvpcie_k2k_CORELIST    = c66x a15_0
drvpcie_k2l_CORELIST    = c66x a15_0
drvpcie_k2e_CORELIST    = c66x a15_0
drvpcie_k2g_CORELIST    = c66x a15_0
drvpcie_c6678_CORELIST  = c66x
drvpcie_c6657_CORELIST  = c66x

drvpcie_am65xx_CORELIST = mpu1_0 mcu1_0

############################
# pcie package
# List of components included under pcie lib
# The components included here are built and will be part of pcie lib
############################
pcie_LIB_LIST = pcie pcie_profile pcie_indp pcie_profile_indp
drvpcie_LIB_LIST = $(pcie_LIB_LIST)

############################
# pcie examples
# List of examples under pcie
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
pcie_EXAMPLE_LIST = PCIE_sample_ExampleProject PCIE_Qos_ExampleProject PCIE_sample_SMP_ExampleProject
drvpcie_EXAMPLE_LIST = $(pcie_EXAMPLE_LIST)

#
# PCIE Modules
#

# PCIE LIB
pcie_COMP_LIST = pcie
pcie_RELPATH = ti/drv/pcie
pcie_PATH = $(PDK_PCIE_COMP_PATH)
pcie_LIBNAME = ti.drv.pcie
export pcie_LIBNAME
pcie_LIBPATH = $(pcie_PATH)/lib
export pcie_LIBPATH
pcie_OBJPATH = $(pcie_RELPATH)/pcie
export pcie_OBJPATH
pcie_MAKEFILE = -f build/makefile.mk
export pcie_MAKEFILE
pcie_BOARD_DEPENDENCY = no
pcie_CORE_DEPENDENCY = no
pcie_SOC_DEPENDENCY = yes
export pcie_COMP_LIST
export pcie_BOARD_DEPENDENCY
export pcie_CORE_DEPENDENCY
export pcie_SOC_DEPENDENCY
pcie_PKG_LIST = pcie
export pcie_PKG_LIST
pcie_INCLUDE = $(pcie_PATH)
pcie_SOCLIST = $(drvpcie_SOCLIST)
export pcie_SOCLIST
pcie_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export pcie_$(SOC)_CORELIST

# PCIE LIB DEVICE INDEPENDENT
pcie_indp_COMP_LIST = pcie_indp
pcie_indp_RELPATH = ti/drv/pcie
pcie_indp_PATH = $(PDK_PCIE_COMP_PATH)
pcie_indp_LIBNAME = ti.drv.pcie
export pcie_indp_LIBNAME
pcie_indp_LIBPATH = $(pcie_indp_PATH)/lib
export pcie_indp_LIBPATH
pcie_indp_OBJPATH = $(pcie_indp_RELPATH)/pcie_indp
export pcie_indp_OBJPATH
pcie_indp_MAKEFILE = -f build/makefile_indp.mk
export pcie_indp_MAKEFILE
pcie_indp_BOARD_DEPENDENCY = no
pcie_indp_CORE_DEPENDENCY = no
pcie_indp_SOC_DEPENDENCY = no
export pcie_indp_COMP_LIST
export pcie_indp_BOARD_DEPENDENCY
export pcie_indp_CORE_DEPENDENCY
export pcie_indp_SOC_DEPENDENCY
pcie_indp_PKG_LIST = pcie_indp
pcie_indp_INCLUDE = $(pcie_indp_PATH)
pcie_indp_SOCLIST = $(drvpcie_SOCLIST)
export pcie_indp_SOCLIST
pcie_indp_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export pcie_indp_$(SOC)_CORELIST

# PCIE PROFILING SOC LIB
pcie_profile_COMP_LIST = pcie_profile
pcie_profile_RELPATH = ti/drv/pcie
pcie_profile_PATH = $(PDK_PCIE_COMP_PATH)
pcie_profile_LIBNAME = ti.drv.pcie.profiling
export pcie_profile_LIBNAME
pcie_profile_LIBPATH = $(pcie_profile_PATH)/lib
export pcie_profile_LIBPATH
pcie_profile_OBJPATH = $(pcie_profile_RELPATH)/pcie_profile
export pcie_profile_OBJPATH
pcie_profile_MAKEFILE = -f build/makefile_profile.mk
export pcie_profile_MAKEFILE
pcie_profile_BOARD_DEPENDENCY = no
pcie_profile_CORE_DEPENDENCY = no
pcie_profile_SOC_DEPENDENCY = yes
export pcie_profile_COMP_LIST
export pcie_profile_BOARD_DEPENDENCY
export pcie_profile_CORE_DEPENDENCY
export pcie_profile_SOC_DEPENDENCY
pcie_profile_PKG_LIST = pcie_profile
pcie_profile_INCLUDE = $(pcie_profile_PATH)
pcie_profile_SOCLIST = $(drvpcie_SOCLIST)
export pcie_profile_SOCLIST
pcie_profile_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export pcie_profile_$(SOC)_CORELIST

# PCIE PROFILING SOC INDEPENDENT LIB
pcie_profile_indp_COMP_LIST = pcie_profile_indp
pcie_profile_indp_RELPATH = ti/drv/pcie
pcie_profile_indp_PATH = $(PDK_PCIE_COMP_PATH)
pcie_profile_indp_LIBNAME = ti.drv.pcie.profiling
export pcie_profile_indp_LIBNAME
pcie_profile_indp_LIBPATH = $(pcie_profile_indp_PATH)/lib
export pcie_profile_indp_LIBPATH
pcie_profile_indp_OBJPATH = $(pcie_profile_indp_RELPATH)/pcie_profile_indp
export pcie_profile_indp_OBJPATH
pcie_profile_indp_MAKEFILE = -f build/makefile_profile_indp.mk
export pcie_profile_indp_MAKEFILE
pcie_profile_indp_BOARD_DEPENDENCY = no
pcie_profile_indp_CORE_DEPENDENCY = no
pcie_profile_indp_SOC_DEPENDENCY = no
export pcie_profile_indp_COMP_LIST
export pcie_profile_indp_BOARD_DEPENDENCY
export pcie_profile_indp_CORE_DEPENDENCY
export pcie_profile_indp_SOC_DEPENDENCY
pcie_profile_indp_PKG_LIST = pcie_profile_indp
pcie_profile_indp_INCLUDE = $(pcie_profile_indp_PATH)
pcie_profile_indp_SOCLIST = $(drvpcie_SOCLIST)
export pcie_profile_indp_SOCLIST
pcie_profile_indp_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export pcie_profile_indp_$(SOC)_CORELIST

#
# PCIE Examples
#

# PCIE basic example app
PCIE_sample_ExampleProject_COMP_LIST = PCIE_sample_ExampleProject
PCIE_sample_ExampleProject_RELPATH = ti/drv/pcie/example/sample
PCIE_sample_ExampleProject_PATH = $(PDK_PCIE_COMP_PATH)/example/sample
PCIE_sample_ExampleProject_BOARD_DEPENDENCY = yes
PCIE_sample_ExampleProject_CORE_DEPENDENCY = no
PCIE_sample_ExampleProject_XDC_CONFIGURO = yes
export PCIE_sample_ExampleProject_COMP_LIST
export PCIE_sample_ExampleProject_BOARD_DEPENDENCY
export PCIE_sample_ExampleProject_CORE_DEPENDENCY
export PCIE_sample_ExampleProject_XDC_CONFIGURO
PCIE_sample_ExampleProject_PKG_LIST = PCIE_sample_ExampleProject
PCIE_sample_ExampleProject_INCLUDE = $(PCIE_sample_ExampleProject_PATH)
PCIE_sample_ExampleProject_BOARDLIST = $(drvpcie_BOARDLIST)
export PCIE_sample_ExampleProject_BOARDLIST
PCIE_sample_ExampleProject_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export PCIE_sample_ExampleProject_$(SOC)_CORELIST

# PCIE Qos basic example app
PCIE_Qos_ExampleProject_COMP_LIST = PCIE_Qos_ExampleProject
PCIE_Qos_ExampleProject_RELPATH = ti/drv/pcie/example/Qos
PCIE_Qos_ExampleProject_PATH = $(PDK_PCIE_COMP_PATH)/example/Qos
PCIE_Qos_ExampleProject_BOARD_DEPENDENCY = yes
PCIE_Qos_ExampleProject_CORE_DEPENDENCY = no
PCIE_Qos_ExampleProject_XDC_CONFIGURO = yes
export PCIE_Qos_ExampleProject_COMP_LIST
export PCIE_Qos_ExampleProject_BOARD_DEPENDENCY
export PCIE_Qos_ExampleProject_CORE_DEPENDENCY
export PCIE_Qos_ExampleProject_XDC_CONFIGURO
PCIE_Qos_ExampleProject_PKG_LIST = PCIE_Qos_ExampleProject
PCIE_Qos_ExampleProject_INCLUDE = $(PCIE_Qos_ExampleProject_PATH)
PCIE_Qos_ExampleProject_BOARDLIST = $(drvpcie_BOARDLIST)
export PCIE_Qos_ExampleProject_BOARDLIST
PCIE_Qos_ExampleProject_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export PCIE_Qos_ExampleProject_$(SOC)_CORELIST

# PCIE basic example app with SMP enabled
PCIE_sample_SMP_ExampleProject_COMP_LIST = PCIE_sample_SMP_ExampleProject
PCIE_sample_SMP_ExampleProject_RELPATH = ti/drv/pcie/example/sample
PCIE_sample_SMP_ExampleProject_PATH = $(PDK_PCIE_COMP_PATH)/example/sample
PCIE_sample_SMP_ExampleProject_MAKEFILE = -f makefile SMP=enable
PCIE_sample_SMP_ExampleProject_BOARD_DEPENDENCY = yes
PCIE_sample_SMP_ExampleProject_CORE_DEPENDENCY = no
PCIE_sample_SMP_ExampleProject_XDC_CONFIGURO = yes
export PCIE_sample_SMP_ExampleProject_COMP_LIST
export PCIE_sample_SMP_ExampleProject_BOARD_DEPENDENCY
export PCIE_sample_SMP_ExampleProject_CORE_DEPENDENCY
export PCIE_sample_SMP_ExampleProject_XDC_CONFIGURO
PCIE_sample_SMP_ExampleProject_PKG_LIST = PCIE_sample_SMP_ExampleProject
PCIE_sample_SMP_ExampleProject_INCLUDE = $(PCIE_sample_SMP_ExampleProject_PATH)
PCIE_sample_SMP_ExampleProject_BOARDLIST = $(drvpcie_BOARDLIST)
export PCIE_sample_SMP_ExampleProject_BOARDLIST
PCIE_sample_SMP_ExampleProject_$(SOC)_CORELIST = $(drvpcie_$(SOC)_CORELIST)
export PCIE_sample_SMP_ExampleProject_$(SOC)_CORELIST

export drvpcie_LIB_LIST
export pcie_LIB_LIST
export pcie_EXAMPLE_LIST
export drvpcie_EXAMPLE_LIST

pcie_component_make_include := 1
endif
