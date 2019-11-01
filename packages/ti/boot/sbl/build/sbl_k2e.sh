# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=evmK2E
make spi_flashwriter_clean BOARD=evmK2E

make all BOARD=evmK2E SOC=K2E BOOTMODE=spi
make spi_flashwriter BOARD=evmK2E SOC=K2E

# removes the object files
rm -rf binary/evmK2E/spi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
