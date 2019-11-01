#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/librm_aearmv7.mk
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
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/rm/lib_so
ARMV7OBJDIR := $(ARMV7OBJDIR)/rm/lib
ARMV7BINDIR ?= ./bin

# Default optimization is on
DEBUG_FLAG    ?= -O2

ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD
CFLAGS += $(INTERNALDEFS)

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src
UTLSRCDIR = ./util/libfdt

VPATH=$(SRCDIR):$(UTLSRCDIR)

#List the COMMONSRC Files
COMMONSRCC =        \
    rm.c            \
    rm_nameserver.c \
    rm_policy.c     \
    rm_services.c   \
    rm_transport.c  \
    rm_tree.c       \
    rm_dtb_util.c   \
    rm_allocator.c  \
    fdt.c           \
    fdt_ro.c        \
    fdt_rw.c        \
    fdt_strerror.c  \
    fdt_sw.c        \
    fdt_wip.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(DEBUG_FLAG) -I$(SRCDIR) -I$(UTLSRCDIR) -I.
CFLAGS +=$(COMMONSRCCFLAGS)

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))
COMMONSRCCOBJS_SO = $(patsubst %.c, $(ARMV7OBJDIR_SO)/%.$(OBJEXT), $(COMMONSRCC))

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/%.$(OBJEXT): %.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@
	
$(ARMV7LIBDIR)/librm.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

librm.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@
	
$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

$(ARMV7OBJDIR_SO)/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)
	@touch $(ARMV7OBJDIR_SO)/.created
		
clean:
	@$(RMDIR) $(ARMV7OBJDIR)

