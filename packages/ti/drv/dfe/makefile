#*******************************************************************************
#* FILE PURPOSE: Top level makefile for Creating Component Libraries
#*******************************************************************************
#* FILE NAME: makefile
#*
#* DESCRIPTION: Defines Compiler tools paths, libraries , Build Options 
#*
#*
#*******************************************************************************
#*
# (Mandatory) Specify where various tools are installed.

# Output for prebuilt generated libraries
export LIBDIR ?= ./lib

# ROOT Directory
export ROOTDIR := ../../..

# INCLUDE Directory
export INCDIR := $(EDMA3LLD_BIOS6_INSTALLDIR)/packages;$(C6X_GEN_INSTALL_PATH)/include;$(IPC_INSTALL_PATH)/packages;$(BIOS_INSTALL_PATH)/packages;$(XDC_INSTALL_PATH)/packages;$(UIA_INSTALLDIR)/packages;$(ROOTDIR)

# Common Macros used in make

ifndef RM
export RM = rm -f
endif

ifndef CP
export CP = cp -p
endif

export MKDIR = mkdir -p

ifndef RMDIR
export RMDIR = rm -rf
endif

ifndef SED
export SED = sed
endif

ifndef MAKE
export MAKE = make
endif

# PHONY Targets
.PHONY: all clean cleanall 

# FORCE Targets
FORCE: 

# all rule
all: .executables
.executables: .libraries
.libraries:

# Clean Rule
clean:: clean_package
# Clean Top Level Object Directory 
clean_package :
	$(RMDIR) $(LIBDIR)/*/
	$(RMDIR) package/cfg


# Make rule to create $(LIBDIR)/c66/ti.drv.dfe.ae66 library
.libraries: $(LIBDIR)/c66/ti.drv.dfe.ae66
$(LIBDIR)/c66/ti.drv.dfe.ae66: FORCE
	$(MAKE) -f ./build/c66/ti.drv.dfe.ae66.mk $@
# Rule to clean $(LIBDIR)/c66/ti.drv.dfe.ae66 library
clean ::
	$(RM) $(LIBDIR)/c66/ti.drv.dfe.ae66


# Make rule to create $(LIBDIR)/c66/ti.drv.dfe.ae66e library
.libraries: $(LIBDIR)/c66/ti.drv.dfe.ae66e
$(LIBDIR)/c66/ti.drv.dfe.ae66e: FORCE
	$(MAKE) -f ./build/c66/ti.drv.dfe.ae66e.mk $@
# Rule to clean $(LIBDIR)/c66/ti.drv.dfe.ae66e library
clean ::
	$(RM) $(LIBDIR)/c66/ti.drv.dfe.ae66e
