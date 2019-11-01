SRCS_COMMON += dss_soc.c dss_soc_graph.c

ifeq ($(SOC),$(filter $(SOC), j721e))
SRCS_COMMON += dss_soc_fw.c
endif