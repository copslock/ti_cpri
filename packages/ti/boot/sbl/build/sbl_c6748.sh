# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=lcdkC6748
make mmcsd_flashwriter_clean BOARD=lcdkC6748


make all BOARD=lcdkC6748 SOC=C6748 BOOTMODE=mmcsd
make mmcsd_flashwriter BOARD=lcdkC6748 SOC=C6748

# removes the object files
rm -rf binary/lcdkC6748/mmcsd/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
