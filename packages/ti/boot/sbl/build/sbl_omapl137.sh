# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=evmOMAPL137
make spi_flashwriter_clean BOARD=evmOMAPL137


make all BOARD=evmOMAPL137 SOC=OMAPL137 BOOTMODE=spi
make spi_flashwriter BOARD=evmOMAPL137 SOC=OMAPL137

# removes the object files
rm -rf binary/evmOMAPL137/spi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
