#!/bin/bash
# ******************************************************************************
# * FILE PURPOSE: Create Bootable image from binaries
# ******************************************************************************
# * FILE NAME: K2GImageGen.sh
# *
# * DESCRIPTION:
# *  Creates Bootable image from binaries pointed by env variables
# *
# * USAGE:
# *  ./K2GImageGen.sh
# *
# * Copyright (C) 2017, Texas Instruments, Inc.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# *   Redistributions of source code must retain the above copyright
# *   notice, this list of conditions and the following disclaimer.
# *
# *   Redistributions in binary form must reproduce the above copyright
# *   notice, this list of conditions and the following disclaimer in the
# *   documentation and/or other materials provided with the
# *   distribution.
# *
# *   Neither the name of Texas Instruments Incorporated nor the names of
# *   its contributors may be used to endorse or promote products derived
# *   from this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *****************************************************************************

# Device Id Hard coded to  55
# TODO: Device id need to be reviewed.
# Device ID & CPU ID should be in sync with SBL. Refer to SBL user guide for values
export Dev_ID=55
export MPU_CPU0_ID=0
export DSP0_ID=5

if [ -z $TOOLS_PATH ]; then
    export TOOLS_PATH=$PDK_INSTALL_PATH/ti/boot/sbl/tools
fi

if [ -d $BIN_PATH ]
then
    echo "$BIN_PATH exists"
else
    mkdir $BIN_PATH
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_MPU_CPU0 ]; then
    export APP_MPU_CPU0=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_MPU_CPU0  ]
then
    export MPU_CPU0=$MPU_CPU0_ID
    export image_gen=1
    export APP_MPU_CPU0_RPRC=$APP_MPU_CPU0.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_MPU_CPU0 $APP_MPU_CPU0_RPRC
else
    echo "$APP_MPU_CPU0 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_DSP0 ]; then
    export APP_DSP0=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi

if [ -f $APP_DSP0  ]
then
    export DSP0_CPU=$DSP0_ID
    export image_gen=1
    export APP_DSP0_RPRC=$APP_DSP0.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_DSP0 $APP_DSP0_RPRC
else
    echo "$APP_DSP0 does not exists"
fi

# ImageGen
if [ ! -z $image_gen  ]
then
    # Generating MulticoreImage Gen
    "$TOOLS_PATH/multicoreImageGen/bin/MulticoreImageGen" LE $Dev_ID $BIN_PATH/app $MPU_CPU0 $APP_MPU_CPU0_RPRC $DSP0_CPU $APP_DSP0_RPRC
fi

#Undefine the image_gen variable to avoid re generation of the app Image.
unset image_gen
