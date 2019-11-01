# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=evmK2G ${1}
make flashwriter_clean BOARD=evmK2G
make clean BOARD=iceK2G
make flashwriter_clean BOARD=iceK2G

make all BOARD=evmK2G SOC=K2G BOOTMODE=mmcsd ${1}
make all BOARD=evmK2G SOC=K2G BOOTMODE=qspi ${1}
make flashwriter BOARD=evmK2G SOC=K2G ${1}
make all BOARD=iceK2G SOC=K2G BOOTMODE=mmcsd
make all BOARD=iceK2G SOC=K2G BOOTMODE=qspi
make flashwriter BOARD=iceK2G SOC=K2G

# removes the object files
rm -rf binary/evmK2G/mmcsd/obj/
rm -rf binary/evmK2G/qspi/obj/
rm -rf binary/iceK2G/mmcsd/obj/
rm -rf binary/iceK2G/qspi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
