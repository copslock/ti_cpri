#!/bin/bash
#
# Copyright (c) 2018, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Usage : For AM65XX    : ./firmwareHeaderGen.sh am65xx
#         For AM65XX-HS : ./firmwareHeaderGen.sh am65xx-hs
#         For J721E     : ./firmwareHeaderGen.sh j721e
#         For J721E-HS  : ./firmwareHeaderGen.sh j721e-hs
export RM=rm
export MV=mv
export MAKE=gcc
export ECHO=echo
export CHMOD=chmod
export COPY=cp
export CAT=cat
#Default SOC is am65xx .This can be changed by using first parameter
# as ,for example, "j721e". Assumes device type is GP by default.
export SOC=am65xx
export SOC_TYPE=gp

if [[ $OS == 'Windows_NT' ]]; then
export BIN2C_EXE=bin2c.exe
else
export BIN2C_EXE=bin2c.out
fi

# Specify paths relative to script
export SCRIPT_DIR=$(cd "$( dirname "${BASH_SOURCE[0]}")" && pwd )
echo $SCRIPT_DIR
export SCI_CLIENT_DIR=$(cd "$SCRIPT_DIR/.." && pwd )

# Keeping everything within SCI client folder
export BIN2C_GEN=$SCI_CLIENT_DIR/tools/bin2c/$BIN2C_EXE

# ROOTDIR is used by x509CertificateGen.sh
# If not defined, try to get it relative to
# script location. This flexibility is useful
# when running this script from a git repo, outside
# PRSDK.
if [ "$#" -gt 1 ]; then
export ROOTDIR=$2
export SOC=$1
elif [ "$#" -gt 0 ]; then
export ROOTDIR=$(cd "$SCI_CLIENT_DIR/../../.." && pwd )
export SOC=$1
else
export ROOTDIR=$(cd "$SCI_CLIENT_DIR/../../.." && pwd )
fi

# Pickup correct sysfw binary
if [[ $SOC == *"hs"* ]]; then
  SOC_TYPE=hs-enc
  SOC=${SOC%-hs}
fi

export SCI_CLIENT_IN_SOC_DIR=$SCI_CLIENT_DIR/soc/sysfw/binaries

if [ "$SOC" = "am65xx" ]; then
export SCI_CLIENT_OUT_SOC_DIR=$SCI_CLIENT_DIR/soc/V0
export FIRMWARE_SILICON=$SCI_CLIENT_IN_SOC_DIR/ti-sci-firmware-am65x-$SOC_TYPE.bin
export SYSFW_SE_INNER_CERT=$SCI_CLIENT_OUT_SOC_DIR/ti-sci-cert-am65x-$SOC_TYPE.bin
export SCICLIENT_FIRMWARE_HEADER=sciclient_firmware_V0.h
fi

if [ "$SOC" = "j721e" ]; then
export SCI_CLIENT_OUT_SOC_DIR=$SCI_CLIENT_DIR/soc/V1
export FIRMWARE_SILICON=$SCI_CLIENT_IN_SOC_DIR/ti-sci-firmware-j721e-$SOC_TYPE.bin
export SYSFW_SE_INNER_CERT=$SCI_CLIENT_OUT_SOC_DIR/ti-sci-cert-j721e-$SOC_TYPE.bin
export SCICLIENT_FIRMWARE_HEADER=sciclient_firmware_V1.h
fi

export SYSFW_SE_CUST_CERT=$SCI_CLIENT_OUT_SOC_DIR/sysfw_cert.bin
export SYSFW_SE_SIGNED=$SCI_CLIENT_OUT_SOC_DIR/sysfw.bin

# SBL_CERT_GEN may already be depending on how this is called
export SBL_CERT_GEN="${SBL_CERT_GEN:-$ROOTDIR/ti/build/makerules/x509CertificateGen.sh}"

# Confirm ROOTDIR is correct, if not error out.
if [ ! -f $SBL_CERT_GEN ]; then
    echo "Error: $SBL_CERT_GEN not found!"
    echo "       Usage $0 <pdk-install-path>"
    exit 1
fi

$ECHO "Building the bin2c generation c tool"
cd $SCI_CLIENT_DIR/tools/bin2c/
$RM $BIN2C_EXE
$MAKE bin2c.c -o $BIN2C_EXE
cd -

$CHMOD a+x $SBL_CERT_GEN
$CHMOD a+x $BIN2C_GEN

if [ "$SOC_TYPE" == "gp" ]; then
$ECHO "Generating the Header file for " $FIRMWARE_SILICON
export SBL_CERT_KEY=$ROOTDIR/ti/build/makerules/rom_degenerateKey.pem
$SBL_CERT_GEN -b $FIRMWARE_SILICON -o $SYSFW_SE_SIGNED -c DMSC_I -l 0x40000 -k $SBL_CERT_KEY
else
$ECHO "Generating outer certificate for " $SYSFW_SE_INNER_CERT
export SBL_CERT_KEY=$ROOTDIR/ti/build/makerules/k3_dev_mpk.pem
$SBL_CERT_GEN -b $SYSFW_SE_INNER_CERT -o $SYSFW_SE_CUST_CERT -c DMSC_O -l 0x40000 -k $SBL_CERT_KEY

$ECHO "Generating the Header file for " $FIRMWARE_SILICON
$CAT $SYSFW_SE_CUST_CERT $FIRMWARE_SILICON > $SYSFW_SE_SIGNED
$RM $SYSFW_SE_CUST_CERT
fi

$BIN2C_GEN $SYSFW_SE_SIGNED $SCICLIENT_FIRMWARE_HEADER SCICLIENT_FIRMWARE > $SCI_CLIENT_OUT_SOC_DIR/$SCICLIENT_FIRMWARE_HEADER

$ECHO "Done."
