#!/bin/sh
# 
# Copyright (c) 2013, Texas Instruments Incorporated
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 

# Converts a .dtb file to a C const byte array that can be compiled into
# programs utilizing RM that don't have direct access to a file system
#
NUM_ARGS=5

if [ $# -ne $NUM_ARGS ] ; then
  echo "Usage:"
  echo "        cify <input .dtb file> <output .c file> <const byte array name>"
  echo "             <data section name> <cache alignment>"
  echo " "
  echo "Example:"
  echo "        cify test.dtb test.c testArray testArraySect 128"
  echo "            Creates a C source file containing const char testArray[]"
  echo "            with the binary data from test.dtb"
  echo " "
  echo "            #pragma DATA_SECTION (testArray, \".testArraySect\");"
  echo "            #pragma DATA_ALIGN (testArray, 128)"
  echo "            const char testArray[] = {"
  echo "            ..."
  echo "            ...padded to the next 128 byte interval"
  echo "            };"
  exit 1
else
  echo "#pragma DATA_SECTION ($3, \".$4\");" > $2
  echo "#pragma DATA_ALIGN ($3, $5)" >> $2
  echo "const char $3[] = {" >> $2
  od -tx1 $1 -w1 -v -An | awk '{printf "0x%s,\n",$1}' >> $2
  bytes=`od  -tx1 $1 -w1 -v -An | wc -l`
  remainder=$(($bytes%$5))
  while [ $remainder -lt $5 ]
  do
    echo "0x00," >> $2
    remainder=$(($remainder+1))
  done
  echo "};" >> $2
fi
