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
ARMV7OBJDIR := $(ARMV7OBJDIR)/aif2/lib
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/aif2/lib_so
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

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -DDEVICE_K2K -DK2 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
#CFLAGS += $(INTERNALDEFS) $(LDFLAGS) -Wall -Wextra -Werror -O2 -marm -march=armv7-a -mtune=cortex-a15 -mfpu=neon -ffast-math -mfloat-abi=hard
CFLAGS += $(INTERNALDEFS) $(LDFLAGS) -O2 -marm -march=armv7-a -mtune=cortex-a15 -mfpu=neon -ffast-math -mfloat-abi=hard

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    aif2fl/aif2fl_close.c \
    aif2fl/aif2fl_getHwStatus.c \
    aif2fl/aif2fl_hwSetup.c \
    aif2fl/aif2fl_open.c \
    aif2fl/aif2fl_getBaseAddress.c \
    aif2fl/aif2fl_hwControl.c \
	aif2fl/aif2fl_init.c \
	aif2fl/aif2fl_reset.c \
	aif2lld/AIF_calcParam.c \
	aif2lld/AIF_cfg.c \
	aif2lld/AIF_debug.c \
	aif2lld/AIF_fsync.c \
	aif2lld/AIF_hibernation.c \
	aif2lld/AIF_init.c \
	aif2lld/AIF_init_dat.c \
	aif2lld/AIF_shutdown.c           

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

$(ARMV7LIBDIR)/libaif2.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

libaif2.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@
	
$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@mkdir -p $(ARMV7OBJDIR)/aif2fl
	@mkdir -p $(ARMV7OBJDIR)/aif2lld
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7OBJDIR_SO)/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)
	@mkdir -p $(ARMV7OBJDIR_SO)/aif2fl
	@mkdir -p $(ARMV7OBJDIR_SO)/aif2lld
	@touch $(ARMV7OBJDIR_SO)/.created
	
$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

clean:
	@$(RMDIR) $(ARMV7OBJDIR)

