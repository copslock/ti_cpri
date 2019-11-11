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
# Usage : sysfw_migrate.sh <release tag>
export RM=rm
export MV=mv
export MAKE=gcc
export ECHO=echo
export CHMOD=chmod
export COPY=cp
export CAT=cat
#Default SOC is am65xx .This can be changed by using first parameter
# as ,for example, "j721e". Assumes device type is GP by default.

# Specify paths relative to script
export SCRIPT_DIR=$(cd "$( dirname "${BASH_SOURCE[0]}")" && pwd )
echo $SCRIPT_DIR
export SCI_CLIENT_DIR=$(cd "$SCRIPT_DIR/.." && pwd )

if [ "$#" -gt 0 ]; then
export ROOTDIR=$(cd "$SCI_CLIENT_DIR/../../.." && pwd )
export RELEASE_TAG=$1
else
$ECHO "Usage : sysfw_migrate.sh <release tag>"
fi

git reset --hard HEAD
git fetch origin; git rebase origin/master
$ECHO "Cloning the system-firmware-releases"
cd $SCI_CLIENT_DIR/soc/
$RM -fr sysfw
git clone ssh://git@bitbucket.itg.ti.com/sysfw/system-firmware-releases.git sysfw
$ECHO "Checking out the release version to migrate to: " $RELEASE_TAG
cd $SCI_CLIENT_DIR/soc/sysfw/
git checkout -b $RELEASE_TAG $RELEASE_TAG
$ECHO "Removing files not required..."
$RM -fr .git
$RM -fr src
$RM -fr log
$RM -fr binaries/system-firmware-design-documentation
$RM -fr binaries/system-firmware-full-documentation
$RM binaries/*.elf
$RM binaries/*.png
$RM binaries/*.svg
$RM binaries/*.cmm
$RM binaries/t32-qt-lsf
$RM -fr binaries/am6
$RM -fr binaries/j721e
$RM -fr binaries/memory
# TODO: Support for HS binaries would be part of a separate package
$RM binaries/*hs*
$RM -fr scripts
$RM .gitignore
$RM .gitmodules
$RM Makefile
$RM README.md
cd $SCRIPT_DIR
git add $SCI_CLIENT_DIR/soc/sysfw
git commit -m "Migratiing to SYSFW version $RELEASE_TAG"
$ECHO "Done."
