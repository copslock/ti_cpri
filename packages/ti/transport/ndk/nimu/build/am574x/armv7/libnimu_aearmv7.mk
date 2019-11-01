#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libnimu_aearmv7.mk
#*
#* DESCRIPTION: Defines Source Files, Compilers flags and build rules
#*
#*******************************************************************************
#

#
# Macro definitions referenced below
#
empty =
space =$(empty) $(empty)

# Output for prebuilt generated libraries
ARMV7LIBDIR ?= ./lib
ARMV7OBJDIR ?= ./obj

# Default optimization is on
DEBUG_FLAG    ?= -g

#CROSS_TOOL_INSTALL_PATH = "C:/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_7-2013q3/bin"
#CROSS_TOOL_PRFX = arm-none-eabi-

ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

CFLAGS+=-mno-unaligned-access -c -mcpu=cortex-a15 -mtune=cortex-a15 -marm -mfloat-abi=hard -DDRA7xx -DDEVICE_AM574x -g -gstrict-dwarf -Wall -D__ARMv7 -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD
#INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD
#CFLAGS += $(INTERNALDEFS)

#Device specific definition 

OBJEXT = o 
INTERNALLINKDEFS =

SRCDIR = ./src


VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    nimu_Eth.c
	

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(DEBUG_FLAG) -I$(SRCDIR) -I.
CFLAGS +=$(COMMONSRCCFLAGS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ...
	-@echo compiling INCS = $(INCS) ...
	@$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(ARMV7LIBDIR)/$(LIB_NAME): $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

clean:
	@$(RMDIR) $(ARMV7OBJDIR)

