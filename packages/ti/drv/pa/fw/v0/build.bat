rem  build the pdsp code
pasm_dos -EBldXc -Cc1_0 classify1_0.p
pasm_dos -EBldXc -Cc1_1 classify1_1.p
pasm_dos -EBldXc -Cc1_2 classify1_2.p
pasm_dos -EBldXc -Cc2 classify2.p
pasm_dos -EBldXc -Cm  pam.p

rem Change the output from a .h to a .c, and create a new .h file
cat classify1_0_desc.c > classify1_0_bin.c
cat classify1_0_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_0_bin.c
echo const int c1_0Size = sizeof(c1_0); >> classify1_0_bin.c

cat classify1_1_desc.c > classify1_1_bin.c
cat classify1_1_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_1_bin.c
echo const int c1_1Size = sizeof(c1_1); >> classify1_1_bin.c

cat classify1_2_desc.c > classify1_2_bin.c
cat classify1_2_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify1_2_bin.c
echo const int c1_2Size = sizeof(c1_2); >> classify1_2_bin.c

cat classify2_desc.c > classify2_bin.c
cat classify2_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> classify2_bin.c
echo const int c2Size = sizeof(c2); >> classify2_bin.c

cat pam_desc.c > pam_bin.c
cat pam_bin.h | sed -e s/"unsigned int"/"uint32_t"/g >> pam_bin.c
echo const int mSize = sizeof(m); >> pam_bin.c

del classify1_0_bin.h
del classify1_1_bin.h
del classify1_2_bin.h

del classify2_bin.h
del pam_bin.h
