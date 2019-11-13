#*******************************************************************************
#* FILE PURPOSE: Top level makefile for Creating Component Libraries and example 
#*               binaries of PDK
#*******************************************************************************
#* FILE NAME: makefile
#*
#* DESCRIPTION: Builds the PDK Drivers & Examples 
#*
#*
#*******************************************************************************
#*
# (Mandatory) Specify where various tools are installed.
.NOTPARALLEL:
# This make file is not parallel in its highest level. This is done to serialize the multiple SOCS,BOARDS,CORES.
# However the subsequent makefiles invoked by this makefile have parallelism supported. i.e -j option given to this
# makefile will be passed on to the subsequent makefiles resulting in parallel builds. 

ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

ifndef MAKE
export MAKE = make
endif

ifndef ECHO
export ECHO = echo
endif

ifndef RM
export RM = rm -f
endif

ifndef CP
export CP = cp
endif

ifneq ($(LIMIT_SOCS),)
SOC_LIST = $(LIMIT_SOCS)
else
SOC_LIST = $(SOC)
endif

ifneq ($(LIMIT_BOARDS),)
BOARD_LIST = $(LIMIT_BOARDS)
else
BOARD_LIST = $(BOARD)
endif


ifneq ($(LIMIT_CORES),)
# PRU cores are not built directly
CORE_LIST = $(filter-out $(PRUCORE_LIST),$(LIMIT_CORES))
else
CORE_LIST = $(CORE)
endif
 
.PHONY: all all_libs all_examples clean env_check help xdc_meta \
   buildinfra_all buildinfra_clean buildinfra_pdk_examples \
   non-buildinfra_all non-buildinfra_clean buildinfra_pdk_libs  non-buildinfra_pdk_examples \
   cplusplus_build

# Default BUILD profile is release
BUILD_PROFILE ?= release
#The top level makefile two kinds of sub modules. Ones which are supported by the 'build infrastructure' and which are not.
# i.e The modules which supports 'build infrastructure' are included in ti/build/component.mk and follow a standard set of conventions and targets. 
# They can be built using ti/build/makefile. Many newer targets such as AM3/4/5/6/7 have their modules in this category.
# Those targets in this makefile has the 'buildinfra_' prefix in them.

# The non-build infrastructure modules are the ones which do not adhere to the ti/build/component.mk . They have custom makefiles which are either RTSC generated or custom written makefiles with custom targets & build options.
# Many legacy components for K1/K2 follow this methodology.
# Those non-build infrastructure targets in this makefile has 'non-buildinfra_' prefix in them.


### Common targets for build infra and non-build infra
all: env_check buildinfra_all non-buildinfra_all
all_libs: env_check buildinfra_pdk_libs non-buildinfra_pdk_libs
all_examples: env_check buildinfra_pdk_examples non-buildinfra_pdk_examples
clean: env_check buildinfra_clean non-buildinfra_clean

### Targets which are built by build infrastructure (ti/build/makefile
# The below builds for each SOC, its applicable BOARDS and CORES, filtered by the LIMIT_SOCS/BOARDS/CORES

buildinfra_all buildinfra_pdk_libs buildinfra_pdk_examples buildinfra_clean :
	$(foreach current_soc, $(SOC_LIST),\
	$(foreach current_board, $(filter $(BOARD_LIST),$(BOARD_LIST_$(current_soc)) ), \
	$(foreach current_core, $(filter $(CORE_LIST),$(CORE_LIST_$(current_soc)) ), \
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/build -f makefile $(subst buildinfra_,,$@) CORE=$(current_core) BOARD=$(current_board) SOC=$(current_soc) BUILD_PROFILE=$(BUILD_PROFILE);\
	)))

#	echo "$@ called with CORE=$(current_core) BOARD=$(current_board) SOC=$(current_soc) BUILD_PROFILE=$(BUILD_PROFILE)";\

### Targets which are not built by build infrastructure, (eg legacy rtsc generated makefiles,custom makefiles
non-buildinfra_all non-buildinfra_pdk_libs non-buildinfra_pdk_examples:
	$(MAKE) -C $(PDK_INSTALL_PATH)/ti/build -f makefile_non-buildinfra $(subst non-buildinfra_,,$@) PDK_SOC=$(PDK_SOC)

### Non library/example targets

env_check:
ifeq ($(PDK_SOC),)
	$(ECHO) "Warning !! PDK_SOC is not set!!! Please run 'help' and refer to 'Optional Arguments' in the help section"
endif

help: env_check
	$(ECHO) "------------------------------------------------------"
	$(ECHO) "\# Top level make help"
	$(ECHO) "------------------------------------------------------"
	$(ECHO) "make <target> [OPTIONAL ARGUMENTS]"
	$(ECHO) ""
	$(ECHO) "------------------"
	$(ECHO) "TARGETS <target>"
	$(ECHO) "------------------"
	$(ECHO) " all: Builds all modules (includes library & applications). Available modules are "
	$(ECHO) "     Example:  \"make all \""
	$(ECHO) ""
	$(ECHO) " clean: Cleans libraries & applications of all modules. Available modules are "
	$(ECHO) "     Example:  \"make clean \""
	$(ECHO) ""
	$(ECHO) " all_libs: Builds all pdk driver libraries"
	$(ECHO) "     Example:  \"make all_libs \""
	$(ECHO) ""
	$(ECHO) " clean: Cleans libraries & applications of all modules."
	$(ECHO) "     Example:  \"make clean \""
	$(ECHO) ""
	$(ECHO) " xdc_meta/xdc_meta: Generates xdc meta file (used if pdk is cloned from pdk.git)."
	$(ECHO) "     Example:  \"make xdc_meta \""
	$(ECHO) ""
	$(ECHO) "------------------"
	$(ECHO) "OPTIONAL ARGUMENTS "
	$(ECHO) "------------------"
	$(ECHO) "LIMIT_SOCS=<socs>  This will restrict the building of libraries/apps to the particular SOC(s) specified"
	$(ECHO) "Valid values of LIMIT_SOCS are $(LIMIT_SOCS)"
	$(ECHO) ""
	$(ECHO) "LIMIT_BOARDS=<boards>  This will restrict the building of libraries/apps to the particular BOARD(s) specified"
	$(ECHO) "Valid values of LIMIT_BOARDS are $(LIMIT_BOARDS)"
	$(ECHO) ""
	$(ECHO) "LIMIT_CORES=<cores>  This will restrict the building of libraries/apps to the particular CORE(s) specified"
	$(ECHO) "Valid values of LIMIT_CORES are $(LIMIT_CORES)"
	$(ECHO) ""
	$(ECHO) "PDK_SOC=<soc>  This will specify the SOC version if the pdk directory does not specify it (if git clone pdk.git is used)"
	$(ECHO) "If pdk is git cloned, PDK_SOC should be set to the <soc> where the original PDK directory from the installer was pdk_<soc>_<version>)"
	$(ECHO) "For example if the original PDK installed directory (which got replaced by pdk.git) was <procsdk_install_dir>/pdk_am65xx_1_0_5, PDK_SOC=am65xx"
	$(ECHO) ""
	$(ECHO) "BUILD_PROFILE=<release/debug>  This will determine the type of libraries/examples generated (release/debug friendly). Default is 'release "
	$(ECHO) "Valid values of LIMIT_CORES are $(LIMIT_CORES)"
	$(ECHO) ""
	$(ECHO) "SOC=<soc> CORE=<core> BOARD=<board> This will build libraries/examples for the specified combination of <soc,core,board>"
	$(ECHO) "Valid values of SOC/BOARD/CORE are found from LIMIT_SOC/BOARD/CORES above"
	
	


# if PDK.git is initiated from a fresh clone, the XDC meta headers will not have been generated. (xdc meta headers are used for CCS based projects)
# These headers are usually generated during the procsdk installer building process ('make release' from within a component's directory)
# This xdc_meta target will help enable the user to generate the same meta files after a fresh git clone of pdk

# Find all the directories with package.xdc and run xdc_meta on them to generate the rtsc meta files
PACKAGE_XDC_LOC := $(wildcard */package.xdc)
PACKAGE_XDC_LOC += $(wildcard */*/package.xdc)
PACKAGE_XDC_LOC += $(wildcard */*/*/package.xdc)
PACKAGE_XDC_LOC += $(wildcard */*/*/*/package.xdc)
SUBDIRS =  $(filter-out ./,$(dir $(PACKAGE_XDC_LOC)))
xdc_meta:
	$(foreach xdcdir, $(SUBDIRS),\
	$(MAKE) -C $(xdcdir) $@; \
	)

cplusplus_build:
	$(MAKE) all BUILD_PROFILE=debug CPLUSPLUS_BUILD=yes

