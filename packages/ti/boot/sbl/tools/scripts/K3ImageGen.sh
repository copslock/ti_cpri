#!/bin/bash

# K3ImageGen <CoreID> <.out> [CoreID2] [.out2] ....
# Core ID definition:
#       MPU1_CPU0_ID : 0
#       MPU1_CPU1_ID : 1
#       MPU2_CPU0_ID : 2
#       MPU2_CPU1_ID : 3
#       MCU1_CPU0_ID : 4
#       MCU1_CPU1_ID : 5
#       MCU2_CPU0_ID : 6
#       MCU2_CPU1_ID : 7
#       MCU3_CPU0_ID : 8
#       MCU3_CPU1_ID : 9
#       DSP1_C66X_ID : 10
#       DSP2_C66X_ID : 11
#       DSP1_C7X_ID : 12
#       DSP2_C7X_ID : 13
#       MPU1_SMP_ID : 14
#       MPU2_SMP_ID : 15
#       MPU_SMP_ID : 16
#       MCU1_SMP_ID : 17
#       MCU2_SMP_ID : 18
#       MCU3_SMP_ID : 19
#       ONLY_LOAD_ID : 20
#
# Device Id for K3 is 55
# Device ID & CPU ID should be in sync with SBL. Refer to SBL user guide for values
#  
# CoreID is provided by the user of the script
#
#

export Dev_ID=55


if [ -z $TOOLS_PATH ]; then
    export TOOLS_PATH=$PDK_INSTALL_PATH/ti/boot/sbl/tools
fi



# validating number of parametters passed in
if [ "$#" -lt 2 ]; then
    echo "Invalid number of parameters."
    echo "Syntax:"
    echo "$0 <CoreID> <.out> [<CoreID2> <.out2>] .... "
    echo ""
    exit
else
    export COREID=$1
    export APP_OUT_FILE=$2
    export APP_OUT_NAME=$(basename ${2%.*})
    export APP_OUT_DIR=$(dirname $2)
    echo "Converting $APP_OUT_FILE to SBL loadable format for core #$COREID ..."
fi

if [ ! -z $BIN_PATH ]
then
    echo "BIN_PATH defined"
else
    echo "BIN_PATH is not defined"
    export BIN_PATH=$(cd $APP_OUT_DIR; pwd)
fi

mkdir -p $BIN_PATH
echo "Using $BIN_PATH directory"

if [ -f $APP_OUT_FILE  ]
then
    export image_gen=1
    export APP_RPRC="$BIN_PATH/$APP_OUT_NAME.rprc"
    mono "$TOOLS_PATH/out2rprc/bin/out2rprc.exe" $APP_OUT_FILE $APP_RPRC
    echo "Generated $APP_RPRC"
else
    echo "$APP_OUT_FILE does not exist"
fi


# ImageGen
if [ ! -z $image_gen  ]
then
    # Generating MulticoreImage Gen
    # doing single core binary now. 
    # TODO: extend to multi-core image gen by adding more COREID and core's RPRC image.
    rm -f $BIN_PATH/$APP_OUT_NAME.appimage $BIN_PATH/$APP_OUT_NAME.appimage.signed
    "$TOOLS_PATH/multicoreImageGen/bin/MulticoreImageGen" LE $Dev_ID $BIN_PATH/$APP_OUT_NAME.appimage $COREID $APP_RPRC
    "$PDK_INSTALL_PATH/ti/build/makerules/x509CertificateGen.sh" -b $BIN_PATH/$APP_OUT_NAME.appimage -o $BIN_PATH/$APP_OUT_NAME.appimage.signed -c R5 -l 0x0 -k $PDK_INSTALL_PATH/ti/build/makerules/k3_dev_mpk.pem
fi

#Undefine the image_gen variable to avoid re generation of the app Image.
unset image_gen
