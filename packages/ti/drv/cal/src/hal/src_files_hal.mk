SRCS_COMMON += cal_hal.c
ifeq ($(SOC),$(filter $(SOC), am65xx))
  SRCS_COMMON += cal_halCsi2Am65xx.c
endif
