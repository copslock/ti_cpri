#############################################################
# Device and Flash types for Makefile use                   #
#############################################################
#

# Generic string for device family
DEVSTRING=OMAP-L137

# Particular device types of the family
ARM_DEVICETYPES:=AM1707
DSP_DEVICETYPES:=OMAPL137_v2 OMAPL137_v1 C6747

DEVICETYPES:= $(ARM_DEVICETYPES) $(DSP_DEVICETYPES)

# Supported flash memory types for booting
FLASHTYPES:=SPI_MEM NAND 
