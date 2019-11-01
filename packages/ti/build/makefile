
include Rules.make

BOARD_LIST_ALL = evmDRA72x evmDRA75x evmDRA78x evmAM572x idkAM572x idkAM571x idkAM574x
BOARD_LIST_ALL += $(BOARD_LIST_J6_TDA)
BOARD_LIST_ALL += am65xx_sim
BOARD_LIST_ALL += $(BOARD_LIST_J7_TDA)

#Limit core list based on board
ifeq ($(BOARD),$(filter $(BOARD), $(BOARD_LIST_J7_TDA)))
  CORE_LIST_ALL = mpu1_0
  CORE_LIST_ALL += mcu1_0 mcu1_1
  CORE_LIST_ALL += mcu2_0 mcu2_1 mcu3_0 mcu3_1
  CORE_LIST_ALL += c7x_1
  CORE_LIST_ALL += c66xdsp_1 c66xdsp_2
else
  CORE_LIST_ALL = a15_0 ipu1_0 ipu1_1 ipu2_0 ipu2_1 c66x c66xdsp_1 c66xdsp_2 arp32_1 arp32_2 arp32_3 arp32_4 arm9_0 c674x
  CORE_LIST_ALL += mcu1_0 mcu1_1 mpu1_0 mpu1_1
endif

BUILD_PROFILE_LIST_ALL ?= release debug

pdk_LIB_LIST_CLEAN = $(addsuffix _clean, $(pdk_LIB_LIST))
pdk_APP_LIB_LIST_CLEAN = $(addsuffix _clean, $(pdk_APP_LIB_LIST))
pdk_EXAMPLE_LIST_CLEAN = $(addsuffix _clean, $(pdk_EXAMPLE_LIST))
pdk_DUP_EXAMPLE_LIST_CLEAN = $(addsuffix _clean, $(pdk_DUP_EXAMPLE_LIST))
pdk_PKG_LIST_ALL_CLEAN = $(addsuffix _clean, $(pdk_PKG_LIST_ALL))
pdk_PKG_LIST_PACKAGE = $(addsuffix _package, $(pdk_PKG_LIST_ALL))

.PHONY : all pdk_libs pdk_libs_clean pdk_app_libs pdk_app_libs_clean examples examples_clean clean package $(pdk_PKG_LIST_ALL) $(pdk_DUP_EXAMPLE_LIST)

all: version pdk_libs pdk_app_libs
	$(MAKE) examples
	$(foreach dup_app, $(pdk_DUP_EXAMPLE_LIST),\
	$(MAKE) $(dup_app);\
	)

version:
	$(ECHO) ------------------------------------------------------
	$(ECHO) \# PDK MM.NN.PP.BB
	$(ECHO) ------------------------------------------------------

pdk_libs: $(pdk_LIB_LIST)

pdk_libs_clean: $(pdk_LIB_LIST_CLEAN)

pdk_app_libs: $(pdk_APP_LIB_LIST)

pdk_app_libs_clean: $(pdk_APP_LIB_LIST_CLEAN)

examples: $(pdk_EXAMPLE_LIST)

examples_clean: $(pdk_EXAMPLE_LIST_CLEAN)

clean: pdk_libs_clean pdk_app_libs_clean examples_clean
	$(foreach dup_app, $(pdk_DUP_EXAMPLE_LIST_CLEAN),\
	$(MAKE) $(dup_app);\
	)

allclean: destroot_clean all_libs_clean

destroot_clean:
ifneq ($(DEST_ROOT),)
	$(RM) -rf $(DEST_ROOT)
endif

all_libs_clean:
	$(RM) -rf ../*/lib
	$(RM) -rf ../*/*/lib
	$(RM) -rf ../*/*/*/lib
	$(RM) -rf ../*/*/*/*/lib

package: $(pdk_PKG_LIST_PACKAGE)

help:
	$(ECHO) ------------------------------------------------------
	$(ECHO) \# PDK make help
	$(ECHO) ------------------------------------------------------
	$(ECHO) make -s [OPTIONAL MAKE VARIABLES] Note: use gmake for windows
	$(ECHO)
	$(ECHO) "Supported targets:"
	$(ECHO) "------------------"
	$(ECHO) "all            : Builds all libraries and examples for the provided CORE and BOARD"
	$(ECHO) "allcores       : Builds all libraries and examples for all applicable CORES for a BOARD"
	$(ECHO) "clean          : Cleans all libraries and examples for the provided CORE and BOARD"
	$(ECHO) "allcores_clean : Cleans all libraries and examples for all applicable CORES for a BOARD"
	$(ECHO) "allclean       : Removes the binary directory using rm -rf"
	$(ECHO) "examples       : Builds all examples"
	$(ECHO) "pdk_libs       : Builds all libraries"
	$(ECHO) "pdk_app_libs   : Builds all application utility libaries"
	$(ECHO) "custom_target  : Builds the target list provided by BUILD_TARGET_LIST_ALL= for all cores and profiles"
	$(ECHO) "<Module>       : Builds a module. Possible values:"
	$(ECHO) "[$(pdk_LIB_LIST)]"
	$(ECHO) ""
	$(ECHO) "<Module_App>   : Builds an application. Possible values:"
	$(ECHO) "[$(pdk_EXAMPLE_LIST) $(pdk_DUP_EXAMPLE_LIST)]"
	$(ECHO) ""
	$(ECHO) "Optional make variables:"
	$(ECHO) "------------------------"
	$(ECHO) "BOARD=[$(BOARD_LIST_ALL)]"
	$(ECHO) "    Default: $(BOARD)"
	$(ECHO) "CORE=[$(CORE_LIST_ALL)]"
	$(ECHO) "    Default: Depends on the platform"
	$(ECHO) "BUILD_PROFILE=[$(BUILD_PROFILE_LIST_ALL)]"
	$(ECHO) "    Default: $(BUILD_PROFILE)"
	$(ECHO) "OS=[Windows_NT linux]"
	$(ECHO) "    Default: Windows_NT"

allcores:
	$(foreach current_core, $(CORE_LIST_ALL),\
	$(MAKE) all CORE=$(current_core) BOARD=$(BOARD) BUILD_PROFILE_$(current_core)=$(BUILD_PROFILE);\
	)

allcores_clean:
	$(foreach current_core, $(CORE_LIST_ALL),\
	$(MAKE) clean CORE=$(current_core) BOARD=$(BOARD) BUILD_PROFILE_$(current_core)=$(BUILD_PROFILE);\
	)

allboards:
	$(foreach current_board, $(BOARD_LIST_ALL),\
	$(MAKE) allcores BOARD=$(current_board) BUILD_PROFILE=$(BUILD_PROFILE);\
	)

allboards_clean:
	$(foreach current_board, $(BOARD_LIST_ALL),\
	$(MAKE) allcores_clean BOARD=$(current_board) BUILD_PROFILE=$(BUILD_PROFILE);\
	)

profiles:
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
	$(MAKE) allcores BOARD=$(BOARD) BUILD_PROFILE=$(current_profile);\
	)

profiles_clean:
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
	$(MAKE) allcores_clean BOARD=$(BOARD) BUILD_PROFILE=$(current_profile);\
	)

pdk_libs_allcores:
	$(foreach current_core, $(CORE_LIST_ALL),\
	$(MAKE) pdk_libs CORE=$(current_core) BOARD=$(BOARD) BUILD_PROFILE_$(current_core)=$(BUILD_PROFILE);\
	)

pdk_libs_allcores_clean:
	$(foreach current_core, $(CORE_LIST_ALL),\
	$(MAKE) pdk_libs_clean CORE=$(current_core) BOARD=$(BOARD) BUILD_PROFILE_$(current_core)=$(BUILD_PROFILE);\
	)

pdk_libs_profiles:
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
	$(MAKE) pdk_libs_allcores BOARD=$(BOARD) BUILD_PROFILE=$(current_profile);\
	)

pdk_libs_profiles_clean:
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
	$(MAKE) pdk_libs_allcores_clean BOARD=$(BOARD) BUILD_PROFILE=$(current_profile);\
	)

custom_target:
	$(foreach current_profile, $(BUILD_PROFILE_LIST_ALL),\
	   	$(foreach current_core, $(CORE_LIST_ALL),\
			$(foreach current_build_target, $(BUILD_TARGET_LIST_ALL),\
				$(MAKE) $(current_build_target) CORE=$(current_core) BOARD=$(BOARD) BUILD_PROFILE_$(current_core)=$(current_profile);\
	  )))


allall:
	$(foreach current_board, $(BOARD_LIST_ALL),\
	$(MAKE) profiles BOARD=$(current_board);\
	)

allall_clean:
	$(foreach current_board, $(BOARD_LIST_ALL),\
	$(MAKE) profiles_clean BOARD=$(current_board);\
	)

#=================================================================
#PDKs libs and tests
$(pdk_PKG_LIST_ALL) $(pdk_DUP_EXAMPLE_LIST):
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($@_SOCLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_XDC_CONFIGURO))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) xdc_configuro,),),\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($@_BOARDLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_XDC_CONFIGURO))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) xdc_configuro,),),))
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($@_SOCLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(MAKE) -C $($@_PATH) $($@_MAKEFILE),$(ECHO) Nothing to be done for $(SOC) $(CORE) $@),\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($@_BOARDLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(MAKE) -C $($@_PATH) $($@_MAKEFILE),$(ECHO) Nothing to be done for $(BOARD) $(SOC) $(CORE) $@),$(ECHO) Nothing to be done for $(SOC) $@))
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($@_SOCLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_SBL_IMAGEGEN))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) sbl_imagegen,),),\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($@_BOARDLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_SBL_IMAGEGEN))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) sbl_imagegen,),),))
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($@_SOCLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_SBL_APPIMAGEGEN))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) sbl_appimagegen,),),\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($@_BOARDLIST))),\
	    $(if $(filter $(CORE), $(subst emptyreplacement,,$($@_$(SOC)_CORELIST))),\
	        $(if $(filter yes, $(subst emptyreplacement,,$($@_SBL_APPIMAGEGEN))),\
	            $(MAKE) -C $($@_PATH) $($@_MAKEFILE) sbl_appimagegen,),),))

$(pdk_PKG_LIST_ALL_CLEAN) $(pdk_DUP_EXAMPLE_LIST_CLEAN):
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($(subst _clean,,$@)_SOCLIST))),\
	        $(MAKE) -C $($(subst _clean,,$@)_PATH) $($(subst _clean,,$@)_MAKEFILE) clean,\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($(subst _clean,,$@)_BOARDLIST))),\
	        $(MAKE) -C $($(subst _clean,,$@)_PATH) $($(subst _clean,,$@)_MAKEFILE) clean,))

$(pdk_PKG_LIST_PACKAGE):
	$(if $(filter $(SOC), $(subst emptyreplacement,,$($(subst _package,,$@)_SOCLIST))),\
	        $(MAKE) -C $($(subst _package,,$@)_PATH) $($(subst _package,,$@)_MAKEFILE) package,\
	$(if $(filter $(BOARD), $(subst emptyreplacement,,$($(subst _package,,$@)_BOARDLIST))),\
	        $(MAKE) -C $($(subst _package,,$@)_PATH) $($(subst _package,,$@)_MAKEFILE) package,$(ECHO) Nothing to be done for $(SOC) $(subst _package,,$@)))

#Below is used only for checking c++ build errors during development, not to be used for any other purpose
cplusplus_build:
	$(MAKE) all BUILD_PROFILE=debug CPLUSPLUS_BUILD=yes

# Nothing beyond this point
