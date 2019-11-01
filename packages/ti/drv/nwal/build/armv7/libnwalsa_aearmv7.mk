#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libnwalsa_aearmv7.mk
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
ARMV7OBJDIR ?= $(ARMV7OBJDIR)/$(LLD_NAME)
ARMV7OBJDIR_SO = $(ARMV7OBJDIR)/$(LLD_NAME)
ARMV7BINDIR ?= ./bin/$(DEVICE)

export ENABLE_SA ?=1 
ifdef CROSS_TOOL_INSTALL_PATH
## Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(NWAL_INSTALL_PATH) -I$(PDK_INSTALL_PATH) -I$(SA_INSTALL_PATH) -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

ifeq ($(ENABLE_SA),1)
EXTERNALDEFS=-DNWAL_ENABLE_SA
else
EXTERNALDEFS=
endif

INTERNALDEFS = $(DEBUG_FLAG) $(CSL_DEVICE) -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    nwal.c \
    nwal_init.c \
    nwal_sec.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = -I$(SRCDIR) -I.
CFLAGS += $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(EXTERNALDEFS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/$(LLD_NAME)/sa/%.$(OBJEXT), $(COMMONSRCC))
COMMONSRCCOBJS_SO = $(patsubst %.c, $(ARMV7OBJDIR_SO)/libso/sa/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/$(LLD_NAME)/sa/%.$(OBJEXT): %.c
	-@echo compiling $< ...
	mkdir -p $(ARMV7OBJDIR)/$(LLD_NAME)/sa/
	$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/libso/sa/%.$(OBJEXT): %.c
	-@echo compiling $< ...
	mkdir -p $(ARMV7OBJDIR_SO)/libso/sa/
	$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@

$(ARMV7LIBDIR)/libnwalsa_$(DEVICE).a: $(COMMONSRCCOBJS)
	mkdir -p $(ARMV7LIBDIR)
	@echo archiving $? into $@ ...
	$(AR) -r $@ $?



libnwalsa_$(DEVICE).so: $(COMMONSRCCOBJS_SO)
	-@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	mkdir -p $(ARMV7LIBDIR)
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@

clean:
	$(RM) $(ARMV7OBJDIR)/$(LLD_NAME)/sa/*.$(OBJEXT)
	$(RM) $(ARMV7OBJDIR_SO)/libso/sa/*.$(OBJEXT)

