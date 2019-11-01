#############################################################
# Common build definitions for Makefile use                 #
#############################################################
#

ARM_TOOLS_PATH?=C:\\ti\\gcc-arm-none-eabi-4_9-2015q3\\
ARM_TOOLS_PREFIX?=arm-none-eabi-

CROSSCOMPILE?=$(ARM_TOOLS_PATH)bin\\$(ARM_TOOLS_PREFIX)
ARM_CROSSCOMPILE=$(CROSSCOMPILE)

DSP_TOOLS_PATH?=C:\\ti\\ti-cgt-c6000_8.1.3\\

DSP_LIB_PATH=$(DSP_TOOLS_PATH)lib\\
DSP_CROSSCOMPILE=$(DSP_TOOLS_PATH)bin\\
DSP_INC_PATH=$(DSP_TOOLS_PATH)include\\
