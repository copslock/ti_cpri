###############################################################################
#
# Instructions:
#
# This makefile is for GNU make 3.79 or later.
#
# This is an example of building the initialization code and the driver.
#
# Inputs:
# - TGTDIR: points to build target (defaults to .)
# - CROSS_COMPILE: GCC compiler prefix
# - CFLAGS: flags to C compiler
#
# End products (for lib target):
#
#	$(TGTDIR)/lib/libusb_ss.a (driver library)
#
# Targets:
#	lib: as above
#	clean: remove objects
#
###############################################################################

COMPONENT_NAME := usb_ss

###############################################################################
# Configurable variables
###############################################################################

#CFLAGS := -g -O0 -Wall

CC     := $(CROSS_COMPILE)gcc
AR     := $(CROSS_COMPILE)ar

###############################################################################
# Calculated variables
###############################################################################

DIR     := .

TGTDIR  ?= $(DIR)

OBJDIR  := $(abspath $(TGTDIR)/obj)
LIBDIR  := $(abspath $(TGTDIR)/lib)

CINCDIR := $(DIR)/include
CCMNDIR := $(DIR)/common
CSRCDIR := $(DIR)/src

CFLAGS  += -I$(CINCDIR) -I$(CSRCDIR) -I$(CCMNDIR)
CFLAGS  += $(EXTRA_CFLAGS)

ARFLAGS := -rcs
ARFLAGS += $(EXTRA_ARFLAGS)

###############################################################################
# File targets
###############################################################################

CINCFILES := $(wildcard $(CINCDIR)/*.h $(CCMNDIR)/*.h)

DRVSRC    := $(notdir $(wildcard $(CSRCDIR)/*.c))

DRVOBJ    := $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(DRVSRC))))

COBJ      := $(DRVOBJ)

LIBNAME   := $(COMPONENT_NAME)
LIB       := $(LIBDIR)/lib$(LIBNAME).a

###############################################################################
# Targets
###############################################################################

.PHONY: all lib

all: lib

lib: $(LIB)

###############################################################################

# clean
.PHONY: clean

clean:
	-rm -f $(COBJ)
	-rmdir $(OBJDIR)
	-rm -rf $(LIBDIR)

###############################################################################
# Pattern Rules
###############################################################################

$(LIB): $(COBJ) | $(LIBDIR)
	@echo COBJ:$(COBJ)
	@echo CINCFILES:$(CINCFILES)
	@echo OBJDIR:$(OBJDIR)
	@echo AR:$(AR)
	$(AR) $(ARFLAGS) $(@) $(^)

$(OBJDIR)/%.o: $(CSRCDIR)/%.c | $(OBJDIR)
	@echo CFLAGS: $(CFLAGS)
	$(CC) -c -o $(@) $(CFLAGS) $(<)

$(OBJDIR) $(LIBDIR):
	@mkdir -p $(@)


###############################################################################
# header deps (lazy)
###############################################################################

$(COBJ) : $(CINCFILES)
