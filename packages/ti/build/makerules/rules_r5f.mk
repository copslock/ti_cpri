# Filename: rules_r5f.mk
#
# Make rules for R5F - This file has all the common rules and defines required
#                     for R5F ISA
#
# This file needs to change when:
#     1. Code generation tool chain changes (currently it uses TMS470)
#     2. Internal switches (which are normally not touched) has to change
#     3. XDC specific switches change
#     4. a rule common for R5F ISA has to be added or modified

CGT_ISA = R5
CGT_EXT = r5f
CGT_PATH = $(TOOLCHAIN_PATH_R5)
include $(MAKERULEDIR)/rules_ti_cgt_arm.mk

# Nothing beyond this point
