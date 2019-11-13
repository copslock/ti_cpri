#!/bin/bash
# ******************************************************************************
# * FILE PURPOSE: Environment Setup for building PDK
# ******************************************************************************
# * FILE NAME: pdksetupenv.sh
# *
# * DESCRIPTION: 
# *  Configures and sets up the Build Environment for PDK. 
# *
# *  The batch file expects an optional argument:PDK_INSTALL_PATH: Location
# *  of the PDK package.  If the argument is not specified the batch file
# *  assumes that the PDK is installed in the same location where the batch
# *  file is located and is being executed.
# *
# * USAGE:
# *  source ./pdksetupenv.sh ~/ti/pdk_<device>_<version>/packages
# *     --- OR ---
# *  source ./pdksetupenv.sh
# *
# * Copyright (C) 2012-2018, Texas Instruments, Inc.
# *****************************************************************************

# *******************************************************************************
# ******************* CHECK IF SCRIPT WAS SOURCED OR SIMPLY RUN   ***************
# *******************************************************************************
# pdksetupenv.sh must always be sourced. Sometimes, peole forget this and can run
# it. We display and error and a prompt whe we detect this.
if [[ "$(basename -- "$0")" == "pdksetupenv.sh" ]]; then
    echo "Error!! Don't run $0, source it" >&2
    echo "USAGE:" >&2
    echo "    source $0" >&2
    exit 1
fi

# *******************************************************************************
# ********************** GET PARAMETERS PASSED THROUGH ARGUMENT   ***************
# *******************************************************************************
# Parameter Validation: Check if the argument was passed to the batch file and
# if so we use that else we default to the working directory where the batch 
# file was invoked from

tempVar=$1
if [ ! -z $tempVar ];then
    export PDK_INSTALL_PATH=$tempVar
else
    export PDK_INSTALL_PATH=${PWD}
fi

# Derive PDK_SOC and PDK_VERSION from the directory name PWD
cd ..
CURDIR_LAST=${PWD##*/}
IFS='_' read -ra ADDR <<< "$CURDIR_LAST"


if [ -z $PDK_SOC ]; then
   export PDK_SOC=${ADDR[1]}
fi

if [ -z "${ADDR[2]}" ]; then
    export PDK_VERSION=
else    
    export PDK_VERSION=${ADDR[2]}_${ADDR[3]}_${ADDR[4]}
fi
cd -

# TI SDK root directory. Derive from PDK INSTALL PATH
if [ -z $SDK_INSTALL_PATH ]; then
cd ../../
export SDK_INSTALL_PATH=${PWD}
cd -
fi

# *******************************************************************************
# ********************** CHECK REQUIRED ENVIRONMENT DEFINES BEGIN ***************
# *******************************************************************************


# Rules.make location. 
export RULES_MAKE="${PDK_INSTALL_PATH}/ti/build/Rules.make"


echo "**************************************************************************"
echo "Environment Configuration:"
echo "**************************************************************************"
echo "    SDK_INSTALL_PATH        : $SDK_INSTALL_PATH"
echo "    PDK_INSTALL_PATH        : $PDK_INSTALL_PATH"
echo "    PDK_SOC                 : $PDK_SOC"
echo "    PDK_VERSION             : $PDK_VERSION"
echo "    RULES_MAKE              : $RULES_MAKE"
echo "**************************************************************************"
