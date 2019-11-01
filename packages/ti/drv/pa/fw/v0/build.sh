# rem  build the pdsp code
pasm_linux -EBldXc -Cc1_0 classify1_0.p
pasm_linux -EBldXc -Cc1_1 classify1_1.p
pasm_linux -EBldXc -Cc1_2 classify1_2.p
pasm_linux -EBldXc -Cc2 classify2.p
pasm_linux -EBldXc -Cm  pam.p

# rem Change the output from a .h to a .c, and create a new .h file
cat classify1_0_desc.c > classify1_0_bin.c
cat classify1_0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_0_bin.c
echo "const int c1_0Size = sizeof(c1_0);" >> classify1_0_bin.c

cat classify1_1_desc.c > classify1_1_bin.c
cat classify1_1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_1_bin.c
echo "const int c1_1Size = sizeof(c1_1);" >> classify1_1_bin.c

cat classify1_2_desc.c > classify1_2_bin.c
cat classify1_2_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_2_bin.c
echo "const int c1_2Size = sizeof(c1_2);" >> classify1_2_bin.c

cat classify2_desc.c > classify2_bin.c
cat classify2_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify2_bin.c
echo "const int c2Size = sizeof(c2);" >> classify2_bin.c

cat pam_desc.c > pam_bin.c
cat pam_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pam_bin.c
echo "const int mSize = sizeof(m);" >> pam_bin.c

rm classify1_0_bin.h
rm classify1_1_bin.h
rm classify1_2_bin.h
rm classify2_bin.h
rm pam_bin.h

# rem update the root with v0 firmware files
cp classify1_0_bin.c ..
cp classify1_1_bin.c ..
cp classify1_2_bin.c ..
cp classify2_bin.c ..
cp pam_bin.c ..

# rem create the bin files for linux
gcc -I../../../../.. -I../.. -I.. -I../../src/v0 pa2bin.c
./a.out
rm a.out

# rem check the bin files generated for linux
gcc -I../../../../.. -I../.. -I.. -I../../src/v0 pabinchk.c
./a.out
rm a.out
