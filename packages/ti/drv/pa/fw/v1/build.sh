#rem  build the pdsp code
pasm_linux -V3EBldc -Cin0_pdsp0 in0_pdsp0.p
pasm_linux -V3EBldc -Cin0_pdsp1 in0_pdsp1.p
pasm_linux -V3EBldc -Cin1_pdsp0 in1_pdsp0.p
pasm_linux -V3EBldc -Cin1_pdsp1 in1_pdsp1.p
pasm_linux -V3EBldc -Cin2_pdsp0 in2_pdsp0.p
pasm_linux -V3EBldc -Cin3_pdsp0 in3_pdsp0.p
pasm_linux -V3EBldc -Cin4_pdsp0 in4_pdsp0.p
pasm_linux -V3EBldc -Cin4_pdsp1 in4_pdsp1.p
pasm_linux -V3EBldc -Cpost_pdsp0 post_pdsp0.p
pasm_linux -V3EBldc -Cpost_pdsp1 post_pdsp1.p
pasm_linux -V3EBldc -Ceg0_pdsp0 eg0_pdsp0.p
pasm_linux -V3EBldc -Ceg0_pdsp1 eg0_pdsp1.p
pasm_linux -V3EBldc -Ceg0_pdsp2 eg0_pdsp2.p
pasm_linux -V3EBldc -Ceg1_pdsp0 eg1_pdsp0.p
pasm_linux -V3EBldc -Ceg2_pdsp0 eg2_pdsp0.p

#rem Change the output from a .h to a .c, and create a new .h file
cat pa2_pdsp_desc.c > pa2_in0_pdsp0_bin.c
cat in0_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in0_pdsp0_bin.c
echo "const int in0_pdsp0Size = sizeof(in0_pdsp0);" >> pa2_in0_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_in0_pdsp1_bin.c
cat in0_pdsp1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in0_pdsp1_bin.c
echo "const int in0_pdsp1Size = sizeof(in0_pdsp1);" >> pa2_in0_pdsp1_bin.c

cat pa2_pdsp_desc.c > pa2_in1_pdsp0_bin.c
cat in1_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in1_pdsp0_bin.c
echo "const int in1_pdsp0Size = sizeof(in1_pdsp0);" >> pa2_in1_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_in1_pdsp1_bin.c
cat in1_pdsp1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in1_pdsp1_bin.c
echo "const int in1_pdsp1Size = sizeof(in1_pdsp1);" >> pa2_in1_pdsp1_bin.c

cat pa2_pdsp_desc.c > pa2_in2_pdsp0_bin.c
cat in2_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in2_pdsp0_bin.c
echo "const int in2_pdsp0Size = sizeof(in2_pdsp0);" >> pa2_in2_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_in3_pdsp0_bin.c
cat in3_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in3_pdsp0_bin.c
echo "const int in3_pdsp0Size = sizeof(in3_pdsp0);" >> pa2_in3_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_in4_pdsp0_bin.c
cat in4_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in4_pdsp0_bin.c
echo "const int in4_pdsp0Size = sizeof(in4_pdsp0);" >> pa2_in4_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_in4_pdsp1_bin.c
cat in4_pdsp1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_in4_pdsp1_bin.c
echo "const int in4_pdsp1Size = sizeof(in4_pdsp1);" >> pa2_in4_pdsp1_bin.c

cat pa2_pdsp_desc.c > pa2_post_pdsp0_bin.c
cat post_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_post_pdsp0_bin.c
echo "const int post_pdsp0Size = sizeof(post_pdsp0);" >> pa2_post_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_post_pdsp1_bin.c
cat post_pdsp1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_post_pdsp1_bin.c
echo "const int post_pdsp1Size = sizeof(post_pdsp1);" >> pa2_post_pdsp1_bin.c

cat pa2_pdsp_desc.c > pa2_eg0_pdsp0_bin.c
cat eg0_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_eg0_pdsp0_bin.c
echo "const int eg0_pdsp0Size = sizeof(eg0_pdsp0);" >> pa2_eg0_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_eg0_pdsp1_bin.c
cat eg0_pdsp1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_eg0_pdsp1_bin.c
echo "const int eg0_pdsp1Size = sizeof(eg0_pdsp1);" >> pa2_eg0_pdsp1_bin.c

cat pa2_pdsp_desc.c > pa2_eg0_pdsp2_bin.c
cat eg0_pdsp2_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_eg0_pdsp2_bin.c
echo "const int eg0_pdsp2Size = sizeof(eg0_pdsp2);" >> pa2_eg0_pdsp2_bin.c

cat pa2_pdsp_desc.c > pa2_eg1_pdsp0_bin.c
cat eg1_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_eg1_pdsp0_bin.c
echo "const int eg1_pdsp0Size = sizeof(eg1_pdsp0);" >> pa2_eg1_pdsp0_bin.c

cat pa2_pdsp_desc.c > pa2_eg2_pdsp0_bin.c
cat eg2_pdsp0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pa2_eg2_pdsp0_bin.c
echo "const int eg2_pdsp0Size = sizeof(eg2_pdsp0);" >> pa2_eg2_pdsp0_bin.c

rm in0_pdsp0_bin.h
rm in0_pdsp1_bin.h
rm in1_pdsp0_bin.h
rm in1_pdsp1_bin.h
rm in2_pdsp0_bin.h
rm in3_pdsp0_bin.h
rm in4_pdsp0_bin.h
rm in4_pdsp1_bin.h
rm post_pdsp0_bin.h
rm post_pdsp1_bin.h
rm eg0_pdsp0_bin.h
rm eg0_pdsp1_bin.h
rm  eg0_pdsp2_bin.h
rm eg1_pdsp0_bin.h
rm eg2_pdsp0_bin.h


# rem create the bin files for linux
gcc -I../../../../.. -I../.. -I.. -I../../src/v1 pa2bin.c
./a.out
rm a.out

# rem check the bin files generated for linux
gcc -I../../../../.. -I../.. -I.. -I../../src/v1 pabinchk.c
./a.out
rm a.out
