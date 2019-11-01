#############################################################
# Device and Flash types for Makefile use                   #
#############################################################
#

# Generic string for device family
DEVSTRING=OMAP-L138

# Particular device types of the family
ARM_DEVICETYPES:=OMAPL138 AM1808 AM1810 OMAPL138_LCDK
DSP_DEVICETYPES:=C6748 C6748_LCDK
DEVICETYPES:=$(ARM_DEVICETYPES) $(DSP_DEVICETYPES)

# Supported flash memory types for booting
FLASHTYPES:=SPI_MEM NAND NOR

