#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libdfe_aearmv7.mk
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
ARMV7OBJDIR := $(ARMV7OBJDIR)/dfe/lib
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/dfe/lib_so
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

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD -D__LINUX_USER_SPACE__ 

CFLAGS += $(INTERNALDEFS) $(DEBUG_FLAG)

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
        dfefl/dfe_fl_Close.c \
	dfefl/dfe_fl_Init.c \
	dfefl/dfe_fl_Open.c \
	dfefl/dfe_fl_autocpClose.c \
	dfefl/dfe_fl_autocppGetHwStatus.c \
	dfefl/dfe_fl_autocppHwControl.c \
	dfefl/dfe_fl_autocppOpen.c \
	dfefl/dfe_fl_bbClose.c \
	dfefl/dfe_fl_bbGetHwStatus.c \
	dfefl/dfe_fl_bbHwControl.c \
	dfefl/dfe_fl_bbOpen.c \
	dfefl/dfe_fl_cbClose.c \
	dfefl/dfe_fl_cbGetHwStatus.c \
	dfefl/dfe_fl_cbHwControl.c \
	dfefl/dfe_fl_cbOpen.c \
	dfefl/dfe_fl_cdfrClose.c \
	dfefl/dfe_fl_cdfrGetHwStatus.c \
	dfefl/dfe_fl_cdfrHwControl.c \
	dfefl/dfe_fl_cdfrOpen.c \
	dfefl/dfe_fl_cfrClose.c \
	dfefl/dfe_fl_cfrGetHwStatus.c \
	dfefl/dfe_fl_cfrHwControl.c \
	dfefl/dfe_fl_cfrOpen.c \
	dfefl/dfe_fl_cppDescripClose.c \
	dfefl/dfe_fl_cppDescripControl.c \
	dfefl/dfe_fl_cppDescripOpen.c \
	dfefl/dfe_fl_cppDescripStatus.c \
	dfefl/dfe_fl_cppDmaClose.c \
	dfefl/dfe_fl_cppDmaControl.c \
	dfefl/dfe_fl_cppDmaOpen.c \
	dfefl/dfe_fl_cppDmaStatus.c \
	dfefl/dfe_fl_dducClose.c \
	dfefl/dfe_fl_dducGetHwStatus.c \
	dfefl/dfe_fl_dducHwControl.c \
	dfefl/dfe_fl_dducOpen.c \
	dfefl/dfe_fl_dpdClose.c \
	dfefl/dfe_fl_dpdGetHwStatus.c \
	dfefl/dfe_fl_dpdHwControl.c \
	dfefl/dfe_fl_dpdOpen.c \
	dfefl/dfe_fl_dpdaClose.c \
	dfefl/dfe_fl_dpdaGetHwStatus.c \
	dfefl/dfe_fl_dpdaHwControl.c \
	dfefl/dfe_fl_dpdaOpen.c \
	dfefl/dfe_fl_fbClose.c \
	dfefl/dfe_fl_fbGetHwStatus.c \
	dfefl/dfe_fl_fbHwControl.c \
	dfefl/dfe_fl_fbOpen.c \
	dfefl/dfe_fl_jesdClose.c \
	dfefl/dfe_fl_jesdGetHwStatus.c \
	dfefl/dfe_fl_jesdHwControl.c \
	dfefl/dfe_fl_jesdOpen.c \
	dfefl/dfe_fl_miscClose.c \
	dfefl/dfe_fl_miscGetHwStatus.c \
	dfefl/dfe_fl_miscHwControl.c \
	dfefl/dfe_fl_miscOpen.c \
	dfefl/dfe_fl_rxClose.c \
	dfefl/dfe_fl_rxGetHwStatus.c \
	dfefl/dfe_fl_rxHwControl.c \
	dfefl/dfe_fl_rxOpen.c \
	dfefl/dfe_fl_summerClose.c \
	dfefl/dfe_fl_summerGetHwStatus.c \
	dfefl/dfe_fl_summerHwControl.c \
	dfefl/dfe_fl_summerOpen.c \
	dfefl/dfe_fl_txClose.c \
	dfefl/dfe_fl_txGetHwStatus.c \
	dfefl/dfe_fl_txHwControl.c \
	dfefl/dfe_fl_txOpen.c \
	dfelld/DFE_bb.c \
	dfelld/DFE_cb.c \
	dfelld/DFE_cfr.c \
	dfelld/DFE_dduc.c \
	dfelld/DFE_device.c \
	dfelld/DFE_dpd.c \
	dfelld/DFE_dpda.c \
	dfelld/DFE_excep.c \
	dfelld/DFE_fb.c \
	dfelld/DFE_jesd.c \
	dfelld/DFE_misc.c \
	dfelld/DFE_open.c \
	dfelld/DFE_rx.c \
	dfelld/DFE_summer.c \
	dfelld/DFE_sync.c \
	dfelld/DFE_tx.c	



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
	
$(ARMV7LIBDIR)/libdfe.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

libdfe.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@
	
$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@mkdir -p $(ARMV7OBJDIR)/dfefl
	@mkdir -p $(ARMV7OBJDIR)/dfelld
	@touch $(ARMV7OBJDIR)/.created
	@touch $(ARMV7OBJDIR)/dfefl/.created
	@touch $(ARMV7OBJDIR)/dfelld/.created

$(ARMV7OBJDIR_SO)/.created:
	@mkdir -p $(ARMV7OBJDIR_SO)
	@mkdir -p $(ARMV7OBJDIR_SO)/dfefl
	@mkdir -p $(ARMV7OBJDIR_SO)/dfelld
	@touch $(ARMV7OBJDIR_SO)/.created
	@touch $(ARMV7OBJDIR_SO)/dfefl/.created
	@touch $(ARMV7OBJDIR_SO)/dfelld/.created
		
$(ARMV7LIBDIR)/.created:
	@mkdir -p $(ARMV7LIBDIR)
	@touch $(ARMV7LIBDIR)/.created

clean:
	@$(RMDIR) $(ARMV7OBJDIR)

