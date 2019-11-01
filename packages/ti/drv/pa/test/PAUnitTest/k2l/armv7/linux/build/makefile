#
# Macro definitions referenced below
#

empty =
space =$(empty) $(empty)

ARMV7OBJDIR ?= ./obj/$(DEVICE)
ARMV7BINDIR ?= ./bin/$(DEVICE)
ARMV7LIBDIR ?= ./lib

ARMV7OBJDIR := $(ARMV7OBJDIR)/pa/test
ARMV7BINDIR := $(ARMV7BINDIR)/pa/test

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
PA_TEST_DIR = $(PA_SRC_DIR)/test/PAUnitTest
PA_TEST_COM_DIR = $(PA_TEST_DIR)/src/common
PA_TEST_UT_DIR = $(PA_TEST_DIR)/src/tests
PA_TEST_ARM_LIN_DIR = $(PA_TEST_DIR)/src/armv7/linux

PA_TEST_REASSEMBLY_LIB_DIR = $(PA_SRC_DIR)/example/reassemLib

INCDIR := $(PDK_INSTALL_PATH);$(CPPI_INC_DIR);$(QMSS_INC_DIR);$(PA_TEST_DIR);$(PA_TEST_COM_DIR);$(PA_TEST_UT_DIR);$(PA_TEST_ARM_LIN_DIR);$(PA_TEST_REASSEMBLY_LIB_DIR)

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
INTERNALDEFS = -g -D__ARMv7 -DSOC_K2L -DNSS_GEN2 -D_VIRTUAL_ADDR_SUPPORT -D__LINUX_USER_SPACE -D_LITTLE_ENDIAN=1 -DMAKEFILE_BUILD

# Linker options
INTERNALLINKDEFS = -Wl,--start-group -L $(ARMV7LIBDIR) $(LIBS) -lrt -Wl,--end-group -pthread $(LDFLAGS)

EXE=paUnitTest$(EXE_EXTN).out
OBJEXT = o 

SRCDIR = $(PA_TEST_DIR):$(PA_TEST_COM_DIR):$(PA_TEST_UT_DIR):$(PA_TEST_ARM_LIN_DIR):$(PA_TEST_REASSEMBLY_LIB_DIR)

INCS = -I. -I$(strip $(subst ;, -I,$(INCDIR)))

VPATH=$(SRCDIR)

#List the COMMONSRC Files
COMMONSRCC = \
    fw_main.c \
    fw_init.c \
    fw_osal.c \
    fw_mem_allocator.c \
    sockutils.c \
    common.c \
    setuprm.c \
    testutil.c \
    test1.c \
    test2.c \
    test3.c \
    test4.c \
    test5.c \
    test6.c \
    test7.c \
    test8.c \
    test9.c \
    test10.c \
    test11.c \
    test12.c \
    test13.c \
    test15.c \
    reassemLib.c

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

