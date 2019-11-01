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
ARMV7OBJDIR := $(ARMV7OBJDIR)/hyplnk/lib
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/hyplnk/lib_so
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

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
CFLAGS += $(INTERNALDEFS) $(LDFLAGS)

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    hyplnk.c \
    hyplnkinit.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(DEBUG_FLAG) -I$(SRCDIR) -I.
CFLAGS += $(COMMONSRCCFLAGS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))
COMMONSRCCOBJS_SO = $(patsubst %.c, $(ARMV7OBJDIR_SO)/%.$(OBJEXT), $(COMMONSRCC))

# convert $(DEVICE) to a define without depending on tr to upcase
ifeq ($(DEVICE),k2h)
  DEVICE_DEFINE=-DDEVICE_K2H
else 
  ifeq ($(DEVICE),k2k)
    DEVICE_DEFINE=-DDEVICE_K2K
  else
    ifeq ($(DEVICE),k2e)
      DEVICE_DEFINE=-DDEVICE_K2E
    else  
      $(error unknown device)
    endif  
  endif
endif

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/%.$(OBJEXT): %.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@

$(ARMV7OBJDIR)/hyplnk_device_$(DEVICE).$(OBJEXT): device/$(DEVICE)/src/hyplnk_device.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ... 
	@$(CC) -c $(CFLAGS) $(DEVICE_DEFINE) $(INCS)  $< -o $@

$(ARMV7OBJDIR_SO)/hyplnk_device_$(DEVICE).$(OBJEXT): device/$(DEVICE)/src/hyplnk_device.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ... 
	@$(CC) -c $(CFLAGS) $(DEVICE_DEFINE) -fPIC $(INCS)  $< -o $@

$(ARMV7LIBDIR)/libhyplnk.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $(patsubst %.created,,$?)

$(ARMV7LIBDIR)/libhyplnk_$(DEVICE).a: $(COMMONSRCCOBJS) $(ARMV7OBJDIR)/hyplnk_device_$(DEVICE).$(OBJEXT) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $(subst $(ARMV7LIBDIR)/.created,,$?)

libhyplnk.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@

libhyplnk_$(DEVICE).so: $(COMMONSRCCOBJS_SO) $(ARMV7OBJDIR_SO)/hyplnk_device_$(DEVICE).$(OBJEXT)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=libhyplnk_device.so.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@
	
$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7OBJDIR_SO)/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)
	@touch $(ARMV7OBJDIR_SO)/.created
	
$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

clean:
	@$(RMDIR) $(ARMV7OBJDIR)

