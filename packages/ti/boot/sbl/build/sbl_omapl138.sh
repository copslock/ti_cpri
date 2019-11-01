# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=lcdkOMAPL138
make mmcsd_flashwriter_clean BOARD=lcdkOMAPL138


make all BOARD=lcdkOMAPL138 SOC=OMAPL138 BOOTMODE=mmcsd
make mmcsd_flashwriter BOARD=lcdkOMAPL138 SOC=OMAPL138

# removes the object files
rm -rf binary/lcdkOMAPL138/mmcsd/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
