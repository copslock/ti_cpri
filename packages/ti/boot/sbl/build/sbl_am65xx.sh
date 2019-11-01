#build instructions for bootloader

ls

make clean all

#archive
cd ..
tar -cf sbl.tar --exclude='obj' --exclude='*.tar' --exclude='*.git*' ./*
