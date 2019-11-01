# Go to PDK SBL root directory
cd ..

#build instructions for bootloader

ls
make clean BOARD=idkAM572x
make clean BOARD=idkAM571x
make clean BOARD=evmAM572x
make clean BOARD=idkAM574x
make flashwriter_clean BOARD=idkAM572x
make flashwriter_clean BOARD=idkAM571x
make flashwriter_clean BOARD=idkAM574x

make all BOARD=idkAM572x SOC=AM572x BOOTMODE=mmcsd
make all BOARD=idkAM572x SOC=AM572x BOOTMODE=qspi
make all BOARD=idkAM571x SOC=AM571x BOOTMODE=mmcsd
make all BOARD=idkAM571x SOC=AM571x BOOTMODE=qspi
make all BOARD=evmAM572x SOC=AM572x BOOTMODE=mmcsd
make all BOARD=evmAM572x SOC=AM572x BOOTMODE=emmc
make flashwriter BOARD=idkAM572x SOC=AM572x
make flashwriter BOARD=idkAM571x SOC=AM571x
make flashwriter BOARD=idkAM574x SOC=AM574x

make all BOARD=idkAM574x SOC=AM574x BOOTMODE=mmcsd DDRECC=yes
make all BOARD=idkAM574x SOC=AM574x BOOTMODE=qspi DDRECC=yes

# removes the object files
rm -rf binary/evmAM572x/mmcsd/obj/
rm -rf binary/idkAM572x/mmcsd/obj/
rm -rf binary/idkAM572x/qspi/obj/
rm -rf binary/idkAM571x/mmcsd/obj/
rm -rf binary/idkAM571x/qspi/obj/
rm -rf binary/idkAM574x/mmcsd/obj/
rm -rf binary/idkAM574x/qspi/obj/

#archive
tar -cf sbl.tar --exclude='*.tar' --exclude='*.git*' ./*
