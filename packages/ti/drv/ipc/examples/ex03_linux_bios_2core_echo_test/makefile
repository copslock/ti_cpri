#
# This file is the makefile for building IPC 2-core echo test app for Linux & TI RTOS
#
ifeq ($(RULES_MAKE), )
include $(PDK_INSTALL_PATH)/ti/build/Rules.make
else
include $(RULES_MAKE)
endif

APP_NAME = ex03_linux_bios_2core_echo_test

SRCS_COMMON = ex03_linux_bios_2core_echo_test.c
CFLAGS_LOCAL_COMMON = -DA72_LINUX_OS

include ../common/makefile.mk
