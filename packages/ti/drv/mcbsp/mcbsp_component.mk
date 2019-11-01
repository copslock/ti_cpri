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

# File: mcbsp_component.mk
#       This file is component include make file of mcbsp library.
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

ifeq ($(mcbsp_component_make_include), )

# under other list
drvmcbsp_BOARDLIST         = 
drvmcbsp_SOCLIST           = k2g omapl138 c6657
drvmcbsp_k2g_CORELIST      = c66x
drvmcbsp_omapl138_CORELIST = c674x
drvmcbsp_c6657_CORELIST    = c66x

############################
# mcbsp package
# List of components included under mcbsp lib
# The components included here are built and will be part of mcbsp lib
############################

mcbsp_LIB_LIST = mcbsp
drvmcbsp_LIB_LIST = $(mcbsp_LIB_LIST)

############################
# mcbsp examples
# List of examples under mcbsp
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
mcbsp_EXAMPLE_LIST =

#
# mcbsp Modules
#

# mcbsp LIB
mcbsp_COMP_LIST = mcbsp
mcbsp_RELPATH = ti/drv/mcbsp
mcbsp_PATH = $(PDK_MCBSP_COMP_PATH)
mcbsp_LIBNAME = ti.drv.mcbsp
export mcbsp_LIBNAME
mcbsp_LIBPATH = $(mcbsp_PATH)/lib
export mcbsp_LIBPATH
mcbsp_OBJPATH = $(mcbsp_RELPATH)/mcbsp
export mcbsp_OBJPATH
mcbsp_MAKEFILE = -f build/makefile.mk
export mcbsp_MAKEFILE
mcbsp_BOARD_DEPENDENCY = no
mcbsp_CORE_DEPENDENCY = no
mcbsp_SOC_DEPENDENCY = yes
export mcbsp_COMP_LIST
export mcbsp_BOARD_DEPENDENCY
export mcbsp_CORE_DEPENDENCY
export mcbsp_SOC_DEPENDENCY
mcbsp_PKG_LIST = mcbsp
export mcbsp_PKG_LIST
mcbsp_INCLUDE = $(mcbsp_PATH)
mcbsp_SOCLIST = $(drvmcbsp_SOCLIST)
export mcbsp_SOCLIST
mcbsp_$(SOC)_CORELIST = $(drvmcbsp_$(SOC)_CORELIST)
export mcbsp_$(SOC)_CORELIST

export drvmcbsp_LIB_LIST
export mcbsp_LIB_LIST

mcbsp_component_make_include := 1
endif
