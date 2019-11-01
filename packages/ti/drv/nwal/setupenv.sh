#!/bin/sh

# Output directory for generated libraries
export LIBDIR=./lib

# PDK install path
export PDK_INSTALL_PATH=/data/tftpboot/rnambiath/PDK/pdk_TCI6614_1_0_0_1003/packages
# NWAL install path
export NWAL_INSTALL_PATH=/data/tftpboot/rnambiath/PDK/nwal-lld

# ARM cross tool executable path
export CROSS_TOOL_INSTALL_PATH=/apps/codesourcery/arm-2009q1/bin

# ARM cross tool prefix 
export CROSS_TOOL_PRFX=arm-none-linux-gnueabi-

