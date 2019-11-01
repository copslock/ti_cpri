# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=evmK2L
make spi_flashwriter_clean BOARD=evmK2L

make all BOARD=evmK2L SOC=K2L BOOTMODE=spi
make spi_flashwriter BOARD=evmK2L SOC=K2L

# removes the object files
rm -rf binary/evmK2L/spi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
