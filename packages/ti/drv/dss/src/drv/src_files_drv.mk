SRCDIR += src/drv/common src/drv/dctrl src/drv/disp
INCDIR += .

PACKAGE_SRCS_COMMON += src/drv/common src/drv/dctrl src/drv/disp
# Common driver source files across all platforms and cores
SRCS_COMMON +=
include src/drv/common/src_files_common.mk
include src/drv/dctrl/src_files_dctrl.mk
include src/drv/disp/src_files_disp.mk