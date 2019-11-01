# File: fvid2_component.mk
#       This file is component include make file of FVID2 driver library.
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
#                             platform and the compiled obj/lib has to be kept
#                             under <platform> directory
#                             "no" or "" or if this variable is not defined: means
#                             this module has no platform dependent code and hence
#                             the obj/libs are not kept under <platform> dir.
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
ifeq ($(fvid2_component_make_include), )

drvfvid2_SOCLIST           = j721e am65xx
drvfvid2_j721e_CORELIST = $(DEFAULT_j721e_CORELIST)
drvfvid2_am65xx_CORELIST   = mcu1_0 mpu1_0

############################
# fvid2 package
# List of components included under fvid2 lib
# The components included here are built and will be part of fvid2 lib
############################
fvid2_LIB_LIST = fvid2

############################
# fvid2 examples
# List of examples under fvid2 (+= is used at each example definition)
# All the tests mentioned in list are built when test target is called
# List below all examples for allowed values
############################
fvid2_EXAMPLE_LIST =

#
# FVID2 Modules
#

# FVID2 library
fvid2_COMP_LIST = fvid2
fvid2_RELPATH = ti/drv/fvid2
fvid2_PATH = $(PDK_FVID2_COMP_PATH)
fvid2_LIBNAME = fvid2
fvid2_LIBPATH = $(PDK_FVID2_COMP_PATH)/lib
fvid2_MAKEFILE = -fsrc/makefile
export fvid2_MAKEFILE
export fvid2_LIBNAME
export fvid2_LIBPATH
fvid2_BOARD_DEPENDENCY = no
fvid2_CORE_DEPENDENCY = no
export fvid2_COMP_LIST
export fvid2_BOARD_DEPENDENCY
export fvid2_CORE_DEPENDENCY
fvid2_PKG_LIST = fvid2
fvid2_INCLUDE = $(fvid2_PATH)
fvid2_SOCLIST = $(drvfvid2_SOCLIST)
export fvid2_SOCLIST
fvid2_$(SOC)_CORELIST = $(drvfvid2_$(SOC)_CORELIST)
export fvid2_$(SOC)_CORELIST

#
# FVID2 Examples
#

export fvid2_LIB_LIST
export fvid2_EXAMPLE_LIST
export drvfvid2_LIB_LIST = $(fvid2_LIB_LIST)
export drvfvid2_EXAMPLE_LIST = $(fvid2_EXAMPLE_LIST)

FVID2_CFLAGS =

# Enable asserts and prints
FVID2_CFLAGS += -DFVID2_CFG_TRACE_ENABLE
FVID2_CFLAGS += -DFVID2_CFG_ASSERT_ENABLE

export FVID2_CFLAGS

fvid2_component_make_include := 1
endif
