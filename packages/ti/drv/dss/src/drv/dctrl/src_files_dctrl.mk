SRCS_COMMON += dss_dctrlGraph.c dss_dctrlApi.c

ifeq ($(SOC),$(filter $(SOC), j721e))
SRCS_COMMON += dss_dctrlExtended.c
endif