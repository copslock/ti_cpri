#!/bin/bash

set -e # Stop on the first error
#set -x # Echo on

# Use input arg as LLD version to put in patch filename
LLD_VERSION=$1
OUTPUT_FILENAME=dfe_pdklinuxdevkit_src_patch_3_01_04_07_ver_${LLD_VERSION}.tar.gz

# Print usage instructions
if [ "$LLD_VERSION" == "" ]
then
    echo "Script to create DFE LLD source code patch from DFE LLD GIT repo which can be un-tarred"
    echo "in the MCSDK install directory before rebuilding the DFE LLD libraries for ARM and DSP."
    echo ""
    echo "Usage:  $0 <LLD_version_number_to_use_in_patch_filename>"
    echo "  i.e.  $0 01_00_00_08"
    exit
else
    echo "Creating source code patch file with name $OUTPUT_FILENAME"
fi

# Remove previous temp_patch_dir and create new one
rm -rf temp_patch_dir
mkdir -p temp_patch_dir

# Copy PDK directory patched source files
mkdir -p temp_patch_dir/pdk_keystone2_3_01_04_07/packages/ti/drv/dfe
cp --parents -t temp_patch_dir/pdk_keystone2_3_01_04_07/packages/ti/drv/dfe  build/armv7/*.mk build/c66/*.mk docs/ReleaseNotes_DFE_LLD.pdf src/*/*.c *.h package.xdc

# Copy mcsdk_linux directory patched source files
mkdir -p temp_patch_dir/mcsdk_linux_3_01_04_07/linux-devkit/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi/usr/include/ti/drv/dfe
cp --parents -t temp_patch_dir/mcsdk_linux_3_01_04_07/linux-devkit/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi/usr/include/ti/drv/dfe  *.h

# Copy script that will be used later to rebuild the LLD from the MCSDK and create the full patch file including libraries
cp --parents -t temp_patch_dir rebuild_dfe_lld_from_mcsdk_and_create_full_patch.sh

# Create tar.gz file with all patched source files
cd temp_patch_dir
tar czf $OUTPUT_FILENAME *
cd ..
mv temp_patch_dir/$OUTPUT_FILENAME .
rm -rf temp_patch_dir

