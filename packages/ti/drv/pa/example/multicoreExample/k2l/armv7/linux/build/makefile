#
# Macro definitions referenced below
#

empty =
space =$(empty) $(empty)

ARMV7OBJDIR ?= ./obj/$(DEVICE)
ARMV7BINDIR ?= ./bin/$(DEVICE)
ARMV7LIBDIR ?= ./lib

ARMV7OBJDIR := $(ARMV7OBJDIR)/pa/example/multicoreExample
ARMV7BINDIR := $(ARMV7BINDIR)/pa/example

#Cross tools
ifdef CROSS_TOOL_INSTALL_PATH
# Support backwards compatibility with KeyStone1 approach
 CC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
 AC = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)as
 AR = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)ar
 LD = $(CROSS_TOOL_INSTALL_PATH)/$(CROSS_TOOL_PRFX)gcc
endif

# INCLUDE Directories
QMSS_INC_DIR = $(PDK_INSTALL_PATH)/ti/drv/qmss
CPPI_INC_DIR = $(PDK_INSTALL_PATH)/ti/drv/cppi
PA_SRC_DIR ?= $(PDK_INSTALL_PATH)/ti/drv/pa
PA_TEST_INC_DIR = $(PA_SRC_DIR)/example/multicoreExample/src

INCDIR := $(PDK_INSTALL_PATH);$(PA_TEST_INC_DIR);$(CPPI_INC_DIR);$(PA_TEST_INC_DIR)

EXAMPLE_DIR = $(PA_TEST_INC_DIR)

# Libraries
CPPI_LIB = -lcppi
QMSS_LIB = -lqmss
PA_LIB   = -lpa2
RM_LIB   = -lrm

ifeq ($(USEDYNAMIC_LIB), yes)
#presuming ARM executable would depend on dynamic library dependency
EXE_EXTN = _so
LIBS     = $(QMSS_LIB) $(PA_LIB) $(CPPI_LIB) $(RM_LIB)
else
#forcing ARM executable to depend on static LLD libraries
LIBS = -Wl,-Bstatic $(QMSS_LIB) $(PA_LIB) $(CPPI_LIB) $(RM_LIB) -Wl,-Bdynamic
EXE_EXTN =
endif

# Compiler options
INTERNALDEFS = $(DEBUG_FLAG) -D__ARMv7 -DSOC_K2L -DNSS_GEN2 -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

EXE=paMCExample$(EXE_EXTN).out
OBJEXT = o 
SRCDIR = $(EXAMPLE_DIR):$(EXAMPLE_DIR)/armv7/linux

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR)

#List the COMMONSRC Files
COMMONSRCC = \
    cppi_qmss_mgmt.c \
    pa_mgmt.c \
    multicore_example.c \
    fw_main.c \
    fw_mem_allocator.c \
    fw_osal.c \
    framework.c \
    sockutils.c \
	local_variables.c

# FLAGS for the COMMONSRC Files
COMMONSRCCFLAGS = -I.

# Make Rule for the COMMONSRC Files
COMMONSRCCOBJS = $(patsubst %.c, $(ARMV7OBJDIR)/%.$(OBJEXT), $(COMMONSRCC))

all:$(ARMV7BINDIR)/$(EXE)

$(ARMV7BINDIR)/$(EXE): $(COMMONSRCCOBJS) $(ARMV7BINDIR)/.created
	@echo linking $(COMMONSRCCOBJS) into $@ ...
	@$(CC) $(COMMONSRCCOBJS) $(INTERNALLINKDEFS) -o $@

$(ARMV7OBJDIR)/%.$(OBJEXT): %.c $(ARMV7OBJDIR)/.created
	@echo compiling $< ...
	@$(CC) -c $(COMMONSRCCFLAGS) $(INTERNALDEFS) $(INCS)  $< -o $@

$(ARMV7OBJDIR)/.created:
	@mkdir -p $(ARMV7OBJDIR)
	@touch $(ARMV7OBJDIR)/.created

$(ARMV7BINDIR)/.created:
	@mkdir -p $(ARMV7BINDIR)
	@touch $(ARMV7BINDIR)/.created

clean:
	@rm -fr $(ARMV7OBJDIR)
	@rm -fr $(ARMV7BINDIR)

