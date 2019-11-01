#!/bin/bash

# Device Id for Vayu - 55
# Device ID & CPU ID should be in sync with SBL. Refer to SBL user guide for values
export Dev_ID=55
export MPU_CPU0_ID=0
export MPU_CPU1_ID=1
export IPU1_CPU0_ID=2
export IPU1_CPU1_ID=3
export IPU1_CPU_SMP_ID=4
export IPU2_CPU0_ID=5
export IPU2_CPU1_ID=6
export IPU2_CPU_SMP_ID=7
export DSP1_ID=8
export DSP2_ID=9

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
if [ -z $APP_MPU_CPU1 ]; then
    export APP_MPU_CPU1=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_MPU_CPU1  ]
then
    export MPU_CPU1=$MPU_CPU1_ID
    export image_gen=1
    export APP_MPU_CPU1_RPRC=$APP_MPU_CPU1.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_MPU_CPU1 $APP_MPU_CPU1_RPRC
else
    echo "$APP_MPU_CPU1 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_IPU1_CPU0 ]; then
    export APP_IPU1_CPU0=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_IPU1_CPU0  ]
then
    export IPU1_CPU0=$IPU1_CPU0_ID
    export image_gen=1
    export APP_IPU1_CPU0_RPRC=$APP_IPU1_CPU0.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_IPU1_CPU0 $APP_IPU1_CPU0_RPRC
else
    echo "$APP_IPU1_CPU0 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_IPU1_CPU1 ]; then
    export APP_IPU1_CPU1=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_IPU1_CPU1  ]
then
    export IPU1_CPU1=$IPU1_CPU1_ID
    export image_gen=1
    export APP_IPU1_CPU1_RPRC=$APP_IPU1_CPU1.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_IPU1_CPU1 $APP_IPU1_CPU1_RPRC
else
    echo "$APP_IPU1_CPU1 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_IPU2_CPU0 ]; then
    export APP_IPU2_CPU0=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_IPU2_CPU0  ]
then
    export IPU2_CPU0=$IPU2_CPU0_ID
    export image_gen=1
    export APP_IPU2_CPU0_RPRC=$APP_IPU2_CPU0.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_IPU2_CPU0 $APP_IPU2_CPU0_RPRC
else
    echo "$APP_IPU2_CPU0 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_IPU2_CPU1 ]; then
    export APP_IPU2_CPU1=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_IPU2_CPU1  ]
then
    export IPU2_CPU1=$IPU2_CPU1_ID
    export image_gen=1
    export APP_IPU2_CPU1_RPRC=$APP_IPU2_CPU1.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_IPU2_CPU1 $APP_IPU2_CPU1_RPRC
else
    echo "$APP_IPU2_CPU1 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_DSP1 ]; then
    export APP_DSP1=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_DSP1  ]
then
    export DSP1_CPU=$DSP1_ID
    export image_gen=1
    export APP_DSP1_RPRC=$APP_DSP1.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_DSP1 $APP_DSP1_RPRC
else
    echo "$APP_DSP1 does not exists"
fi

#Set the variable to point to a dummy Test Image to avoid generation of a dummy app image.
if [ -z $APP_DSP2 ]; then
    export APP_DSP2=$PDK_INSTALL_PATH/ti/boot/sbl/binary/testApp
fi
if [ -f $APP_DSP2  ]
then
    export DSP2_CPU=$DSP2_ID
    export image_gen=1
    export APP_DSP2_RPRC=$APP_DSP2.rprc
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_DSP2 $APP_DSP2_RPRC
else
    echo "$APP_DSP2 does not exists"
fi

# ImageGen
if [ ! -z $image_gen  ]
then
    # Generating MulticoreImage Gen
    "$TOOLS_PATH/multicoreImageGen/bin/MulticoreImageGen" LE $Dev_ID $BIN_PATH/app $MPU_CPU0 $APP_MPU_CPU0_RPRC $MPU_CPU1 $APP_MPU_CPU1_RPRC $IPU1_CPU0 $APP_IPU1_CPU0_RPRC $IPU1_CPU1 $APP_IPU1_CPU1_RPRC $IPU2_CPU0 $APP_IPU2_CPU0_RPRC $IPU2_CPU1 $APP_IPU2_CPU1_RPRC $DSP1_CPU $APP_DSP1_RPRC $DSP2_CPU $APP_DSP2_RPRC
fi

#Undefine the image_gen variable to avoid re generation of the app Image.
unset image_gen
