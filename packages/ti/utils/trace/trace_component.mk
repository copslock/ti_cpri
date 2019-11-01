#
# Copyright (c) 2019, Texas Instruments Incorporated
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

# File: trace_component.mk
#       This file is component include make file of TRACE Utils library.
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
ifeq ($(trace_component_make_include), )

# under other list
drvtrace_SOCLIST         = tda2xx tda2ex tda3xx dra75x dra78x am572x am574x am571x k2h k2k k2l k2e k2g c6678 c6657 am437x am335x am65xx
drvtrace_tda2xx_CORELIST = c66x a15_0 ipu1_0
drvtrace_tda2ex_CORELIST = c66x a15_0 ipu1_0
drvtrace_tda3xx_CORELIST = c66x ipu1_0
drvtrace_dra75x_CORELIST = c66x a15_0 ipu1_0
drvtrace_dra78x_CORELIST = c66x ipu1_0
drvtrace_am572x_CORELIST = c66x a15_0 ipu1_0
drvtrace_am574x_CORELIST = c66x a15_0 ipu1_0
drvtrace_am571x_CORELIST = c66x a15_0 ipu1_0
drvtrace_k2h_CORELIST    = c66x a15_0
drvtrace_k2k_CORELIST    = c66x a15_0
drvtrace_k2l_CORELIST    = c66x a15_0
drvtrace_k2e_CORELIST    = c66x a15_0
drvtrace_k2g_CORELIST    = c66x a15_0
drvtrace_c6678_CORELIST  = c66x
drvtrace_c6657_CORELIST  = c66x
drvtrace_am437x_CORELIST = a9host
drvtrace_am335x_CORELIST = a8host
drvtrace_am65xx_CORELIST = mpu1_0 mcu1_0

############################
# trace package
# List of components included under trace lib
# The components included here are built and will be part of trace lib
############################
trace_LIB_LIST =
drvtrace_LIB_LIST = $(trace_LIB_LIST)


trace_component_make_include := 1
endif
