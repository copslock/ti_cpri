#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libqmss_aearmv7.mk
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
ARMV7OBJDIR := $(ARMV7OBJDIR)/iqn2/lib
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/iqn2/lib_so
ARMV7BINDIR ?= ./bin
DEBUG_FLAG  ?= -O2

ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -DDEVICE_K2L -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
CFLAGS += $(INTERNALDEFS) $(LDFLAGS) -Wall -Wextra -Werror -O2 -marm -march=armv7-a -mtune=cortex-a15 -mfpu=neon -ffast-math -mfloat-abi=hard

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    iqn2fl/iqn2fl_init.c \
    iqn2fl/iqn2fl_open.c \
    iqn2fl/iqn2fl_close.c \
    iqn2fl/iqn2fl_hwControl.c \
    iqn2fl/iqn2fl_getHwStatus.c \
    iqn2fl/iqn2fl_hwSetup.c \
	iqn2lld/IQN2_init.c \
	iqn2lld/IQN2_calcParam.c \
	iqn2lld/IQN2_debug.c \
	iqn2lld/IQN2_shutdown.c \
	iqn2lld/IQN2_runtime.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(DEBUG_FLAG) -I$(SRCDIR) -I.
CFLAGS += $(COMMONSRCCFLAGS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))
COMMONSRCCOBJS_SO = $(patsubst %.c, $(ARMV7OBJDIR_SO)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/%.$(OBJEXT): %.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@

$(ARMV7LIBDIR)/libiqn2.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

libiqn2.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@
	
$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@mkdir -p $(ARMV7OBJDIR)/iqn2fl
	@mkdir -p $(ARMV7OBJDIR)/iqn2lld
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7OBJDIR_SO)/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)
	@mkdir -p $(ARMV7OBJDIR_SO)/iqn2fl
	@mkdir -p $(ARMV7OBJDIR_SO)/iqn2lld
	@touch $(ARMV7OBJDIR_SO)/.created
	
$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

clean:
	@$(RMDIR) $(ARMV7OBJDIR)

