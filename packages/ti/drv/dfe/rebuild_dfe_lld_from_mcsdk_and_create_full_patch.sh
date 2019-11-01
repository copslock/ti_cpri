#!/bin/bash

set -e # Stop on the first error
#set -x # Echo on

# Environment variables required to rebuild DFE LLD ARM libraries

export CROSS_TOOL_INSTALL_PATH=$LINARO_ROOT/bin
export CROSS_TOOL_PRFX=arm-linux-gnueabihf-

# Environment variables required to rebuild DFE LLD DSP libraries

# path to C6000 compiler (i.e. $HOME/ti/cgt/c6000/7_4_12/c6000_7.4.12)
export C6X_GEN_INSTALL_PATH=$CG_TOOL_ROOT


# Use input arg as LLD version to put in patch filename
LLD_VERSION=$1
OUTPUT_FILENAME=dfe_pdklinuxdevkit_full_patch_3_01_04_07_ver_${LLD_VERSION}.tar.gz

# Print usage instructions
if [ "$LLD_VERSION" == "" ]
then
    echo "Script to rebuild DFE LLD ARM and DSP libraries from MCSDK and create a full patch which"
    echo "can be un-tarred in an MCSDK install directory to patch the source and libraries without"
    echo "having to recompile the libraries."
    echo ""
    echo "Usage:  $0 <LLD_version_number_to_use_in_patch_filename>"
    echo "  i.e.  $0 01_00_00_08"
    exit
else
    echo "Creating full patch file with name $OUTPUT_FILENAME"
fi

# Rebuild ARM libraries
echo Rebuilding ARM libraries
cd pdk_keystone2_3_01_04_07/packages/ti/drv/dfe
make -f makefile_armv7 clean all

# Rebuild DSP libraries
echo Rebuilding DSP libraries
make -f makefile clean all
cd ../../../../..

# Remove previous temp_patch_dir and create new one
rm -rf temp_patch_dir
mkdir -p temp_patch_dir

# Copy PDK directory patched source files
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/build/armv7/*.mk
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/build/c66/*.mk
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/docs/ReleaseNotes_DFE_LLD.pdf
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/src/*/*.c
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/*.h
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/package.xdc

# Copy mcsdk_linux directory patched source files
cp --parents -t temp_patch_dir mcsdk_linux_3_01_04_07/linux-devkit/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi/usr/include/ti/drv/dfe/*.h

# Copy rebuilt ARM libraries
mkdir -p temp_patch_dir/mcsdk_linux_3_01_04_07/linux-devkit/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi/usr/lib
cp -t temp_patch_dir/mcsdk_linux_3_01_04_07/linux-devkit/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi/usr/lib pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/lib/libdfe.*

# Copy rebuilt DSP libraries
cp --parents -t temp_patch_dir pdk_keystone2_3_01_04_07/packages/ti/drv/dfe/lib/c66/ti.drv.dfe.ae66*

# Create tar.gz file with all patched source files
cd temp_patch_dir
tar czf $OUTPUT_FILENAME *
cd ..
mv temp_patch_dir/$OUTPUT_FILENAME .
rm -rf temp_patch_dir

echo "Done.  Full patch file was generated with name $OUTPUT_FILENAME"

