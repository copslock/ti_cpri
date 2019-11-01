#
# This file is the makefile for building IPC two-core echo test app for TI RTOS
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = ex01_bios_2core_echo_test

SRCS_COMMON = ex01_bios_2core_echo_test.c

include ../common/makefile.mk
