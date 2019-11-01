ifneq ($(SOC),$(filter $(SOC), am65xx))
SRCDIR += src/csl src/csl/dp src/csl/dp_sd0801
INCDIR += . src/csl/common

PACKAGE_SRCS_COMMON += src/csl/dp src/csl/dp_sd0801 src/csl/common src/csl/csl_priv.c
SRCS_COMMON += csl_priv.c
include src/csl/dp/src/src_files_dp.mk
include src/csl/dp_sd0801/src/src_files_sd0801.mk
endif
