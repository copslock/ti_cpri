#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libnwal_aearmv7.mk
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

ifdef CROSS_TOOL_INSTALL_PATH
## Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(NWAL_INSTALL_PATH) -I$(PDK_INSTALL_PATH) -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

INTERNALDEFS = $(DEBUG_FLAG) $(CSL_DEVICE) -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
#INTERNALDEFS = $(DEBUG_FLAG) -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
CFLAGS += $(INTERNALDEFS)
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
CFLAGS+=$(COMMONSRCCFLAGS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/$(LLD_NAME)/%.$(OBJEXT), $(COMMONSRCC))
COMMONSRCCOBJS_SO = $(patsubst %.c, $(ARMV7OBJDIR_SO)/libso/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/$(LLD_NAME)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/$(LLD_NAME)/.created
	-@echo compiling $< ...
	$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/libso/%.$(OBJEXT): %.c $(ARMV7OBJDIR_SO)/libso/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@
$(ARMV7LIBDIR)/libnwal_$(DEVICE).a: $(COMMONSRCCOBJS)
	mkdir -p $(ARMV7LIBDIR)
	echo archiving $? into $@ ...
	$(AR) -r $@ $?

libnwal_$(DEVICE).so: $(COMMONSRCCOBJS_SO)
	-@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	mkdir -p $(ARMV7LIBDIR)
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@

$(ARMV7OBJDIR)/$(LLD_NAME)/.created:
	@mkdir -p $(ARMV7OBJDIR)/$(LLD_NAME)
	@touch $(ARMV7OBJDIR)/$(LLD_NAME)/.created

$(ARMV7OBJDIR_SO)/libso/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)/libso
	@touch $(ARMV7OBJDIR_SO)/libso/.created
clean:
	rm -rf $(ARMV7OBJDIR)/$(LLD_NAME)
	rm -rf $(ARMV7OBJDIR_SO)/libso

