# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=evmK2H
make clean BOARD=evmK2K
make spi_flashwriter_clean BOARD=evmK2H
make spi_flashwriter_clean BOARD=evmK2K

make all BOARD=evmK2H SOC=K2H BOOTMODE=spi
make all BOARD=evmK2K SOC=K2K BOOTMODE=spi
make spi_flashwriter BOARD=evmK2H SOC=K2H
make spi_flashwriter BOARD=evmK2K SOC=K2K

# removes the object files
rm -rf binary/evmK2H/spi/obj/
rm -rf binary/evmK2K/spi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
