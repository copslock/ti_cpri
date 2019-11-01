#*******************************************************************************
#* FILE PURPOSE: Lower level makefile for Creating Component Libraries for ARMv7
#*******************************************************************************
#* FILE NAME: ./lib/libcppi_aearmv7.mk
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

# Default optimization is on
DEBUG_FLAG    ?= -O2

# Output for prebuilt generated libraries
ARMV7LIBDIR ?= ./lib
ARMV7OBJDIR ?= ./obj
ARMV7OBJDIR_SO := $(ARMV7OBJDIR)/cppi/lib_so
ARMV7OBJDIR := $(ARMV7OBJDIR)/cppi/lib
ARMV7BINDIR ?= ./bin

ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

INCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(INCDIR))))

INTERNALDEFS = -D__ARMv7 -D_LITTLE_ENDIAN=1 -D_VIRTUAL_ADDR_SUPPORT -DMAKEFILE_BUILD  $(LDFLAGS)
CFLAGS += $(INTERNALDEFS) $(DEBUG_FLAG)

OBJEXT = o 
INTERNALLINKDEFS =
SRCDIR = ./src

VPATH=$(SRCDIR) 

#List the COMMONSRC Files
COMMONSRCC = \
    cppi_drv.c \
    cppi_desc.c \
    cppi_listlib.c \
    cppi_heap.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = $(DEBUG_FLAG) -I$(SRCDIR) -I.
CFLAGS +=$(COMMONSRCCFLAGS)

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
    ifeq ($(DEVICE),k2l)
      DEVICE_DEFINE=-DDEVICE_K2L
    else  
      ifeq ($(DEVICE),k2e)
        DEVICE_DEFINE=-DDEVICE_K2E
      else  
        $(error unknown device)
      endif  
    endif  
  endif
endif

$(COMMONSRCCOBJS): $(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) $(INCS)  $< -o $@

$(COMMONSRCCOBJS_SO): $(ARMV7OBJDIR_SO)/%.$(OBJEXT): %.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ...
	@$(CC) -c $(CFLAGS) -fPIC $(INCS)  $< -o $@

$(ARMV7OBJDIR)/cppi_device_$(DEVICE).$(OBJEXT): device/$(DEVICE)/src/cppi_device.c $(ARMV7OBJDIR)/.created
	-@echo compiling $< ... 
	@$(CC) -c $(CFLAGS) $(DEVICE_DEFINE) $(INCS)  $< -o $@

$(ARMV7OBJDIR_SO)/cppi_device_$(DEVICE).$(OBJEXT): device/$(DEVICE)/src/cppi_device.c $(ARMV7OBJDIR_SO)/.created
	-@echo compiling $< ... 
	@$(CC) -c $(CFLAGS) $(DEVICE_DEFINE) -fPIC $(INCS)  $< -o $@

$(ARMV7LIBDIR)/libcppi.a: $(COMMONSRCCOBJS) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?
    
$(ARMV7LIBDIR)/libcppi_$(DEVICE).a: $(COMMONSRCCOBJS) $(ARMV7OBJDIR)/cppi_device_$(DEVICE).$(OBJEXT) $(ARMV7LIBDIR)/.created
	@echo archiving $? into $@ ...
	@$(AR) -r $@ $?

libcppi.so: $(COMMONSRCCOBJS_SO)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=$@.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
	@ln -s $@.1.0.0 $@.1
	@ln -s $@.1     $@
	@mv -f $@.1.0.0 $(ARMV7LIBDIR)/$@.1.0.0
	@mv -f $@.1 $(ARMV7LIBDIR)/$@.1
	@mv -f $@   $(ARMV7LIBDIR)/$@

libcppi_$(DEVICE).so: $(COMMONSRCCOBJS_SO) $(ARMV7OBJDIR_SO)/cppi_device_$(DEVICE).$(OBJEXT)
	@echo archiving $? into $(ARMV7LIBDIR)/$@.1 ...
	@$(CC) $(DEBUG_FLAG) -ggdb2 -Wl,-soname=libcppi_device.so.1 -shared -fPIC ${LDFLAGS} -o $@.1.0.0 $^
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

