/** 
 *  \file   tcp3d_codeBlkSeg.c
 *
 *  \brief  Calculates code block segmentation parameteres based on the block length and SW0 length. 
 *
 *  Copyright (c) Texas Instruments Incorporated 2008
 *
 *  Use of this software is controlled by the terms and conditions found in the
 *  license agreement under which this software has been supplied or provided.
 *  
 */
#include "tcp3d_codeBlkSeg.h"

/** TCP3D SW0 nominal values */
Int32 tcp3d_sw0_Tab[] = {16, 32, 48,  64, 96, 128};

/** Used for getting the sw0LenSel index values */
Int32 TAB[] = {0, 1, 2, 3, 3, 4, 4, 5};

/** Table used for division optimization logic */
Int32 shiftValTab [] = {4, 5, 4, 6, 5, 7};

/** Table used for division optimization logic */
Uint32 mulValTab [] = {32768, 32768, 10923, 32768, 10923, 32768};

/** Table used for checking bounds */
Uint32 frameLenTab[2][2] = {40,5114,40,6144};

/** 
 *  \fn     Int32  TCP3D_codeBlkSeg (
 *                    IN  Uint32  blockLengthK,
 *                    IN  Uint8   numMAP,
 *                    IN  Uint8  * const RESTRICT   sw0NomLen,
 *                    OUT Uint8  * const RESTRICT   sw0LenSel,
 *                    OUT Uint8  * const RESTRICT   sw1Len,
 *                    OUT Uint8  * const RESTRICT   sw2LenSel,
 *                    OUT Uint8  * const RESTRICT   numsw0)
 *  \brief   Calculates code block segmentation parameteres based on the block length and SW0 length.
 *           
 * 
 *  \param[in]    blockLengthK
 *              Code block length. (Number of information bits.)
 *              
 *
 *  \param[in]    numMAP
 *              Number of MAP decoders used. =1 for WCDMA, =2 for LTE and WiMAX
 *
 *  \param[in]    sw0NomLen
 *              Nominal length for the sliding window 0. Valid values are from the set: {16, 32, 
 *              48, 64, 96, 128}. Note that if [blockLengthK <= (numMAP * 128 * sw0NomLen)] does 
 *              not hold, the function will pick the first greater length value from the set for which
 *              the above inequality holds, and will return it.
 * 
 *              This will be updated with the picked value.
 *              
 *  \param[out]    sw0LenSel
 *              Input configuration register parameter. The value depends on the picked SW0 length 
 *              used, (sw0NomLen), and the possible values are: 
 *              0 - 16 bits 
 *              1 - 32 bits
 *              2 - 48 bits
 *              3 - 64 bits
 *              4 - 96 bits
 *              5 - 128 bits
 *
 *              
 *  \param[out]    sw1Len
 *              Input configuration register parameter. The value depends on the SW1 length used 
 *              and the possible values are:
 *              9 - 10 bits
 *              10 - 11 bits
 *              11 - 12 bits
 *                ...
 *              127 - 128 bits
 * 
 *              
 *  \param[out]    sw2LenSel
 *              Input configuration register parameter. The value depends on the SW1 length used 
 *              and the possible values are:
 *              0 - SW2 is not present
 *              1 - SW2 length is same as SW1
 *              2 - SW2 length is less by 2 bits from SW1
 *              
 *  \param[out]    numsw0
 *              Input configuration register parameter. Number of SW0 used in the decoder.
 *              
 *  \return     Return indicates PASS or FAIL with ZERO or non-ZERO values.
 * 
 * 
 */
Int32 TCP3D_codeBlkSeg (
            IN  Uint32  blockLengthK,
            IN  Uint8   numMAP,
            INOUT  Uint8  * const RESTRICT     sw0NomLen,
            OUT Uint8  * const RESTRICT     sw0LenSel,
            OUT Uint8  * const RESTRICT     sw1Len,
            OUT Uint8  * const RESTRICT     sw2LenSel,
            OUT Uint8  * const RESTRICT     numsw0)
{
    Int32  status = 0;
    Int32 K, Kext;
    Int32 numSWrem;
    Int32 subFrameLen;
    Int32 sw0LenSelTmp;
    Int32 sw1LenTmp;
    Int32 sw2LenSelTmp;
    Int32 numsw0Tmp;
    Int32 numSW;
    Int32 shiftVal, mulVal;
    Int32 sw0Len = *sw0NomLen;

    /**
     * Check the bounds based on numMAP value. frameLenTab is for the bound values
     *      numMAP  -   mode    - block length bounds
     *        1     -   3GPP    -   [40,5114]
     *        2     - LTE/WIMAX -   [40,6144]
     */
    if ( (blockLengthK < frameLenTab[numMAP-1][0]) ||
         (blockLengthK > frameLenTab[numMAP-1][1]) )
    {
        status = 1;
        return (status);
    }

    K = blockLengthK;
    Kext = ((K + 0x3)>>2)<<2;

    //Calculate sw0LenSelTmp, SW1Len, SW2LenSel, numsw0Tmp
    subFrameLen = Kext >> numMAP;   //Kext / (2*numMAP);

    sw0LenSelTmp = TAB[((sw0Len>>4)-1)&0x7];

    //Check that this holds: (reg->NumInfoBits <= 128 * sparms->tcp3_SW0_length * numMap)
    while((Kext > 128 * sw0Len * numMAP) && sw0LenSelTmp<6)
    {
        sw0LenSelTmp++;
        sw0Len = tcp3d_sw0_Tab[sw0LenSelTmp];
    }

    //numSW = subFrameLen/sw0Len;  Replaced by:
    shiftVal = shiftValTab[sw0LenSelTmp];
    mulVal   = mulValTab[sw0LenSelTmp];
    numSW = _mpysu((subFrameLen >> shiftVal), mulVal)>>15;

    numSWrem = subFrameLen - numSW*sw0Len;
    if(numSWrem)
    {
        numSW++;
    }

    if(numSW == 1)
    {
        numsw0Tmp = 0;
        sw1LenTmp = subFrameLen-1;             //stored value is (sw1_length -1)
        sw2LenSelTmp = 0;                      //SW2 is Off.
    }
    else if(numSW == 2)
    {
        numsw0Tmp = 0;
        if(subFrameLen & 0x3)
        {
            sw1LenTmp = 2*(subFrameLen>>2) + 1;    //stored value is (sw1_length -1)
            sw2LenSelTmp = 2;                      //sw1LenTmp > SW2Len
        }
        else
        {
            sw1LenTmp = (subFrameLen>>1) - 1;  //stored value is (sw1_length -1)
            sw2LenSelTmp = 1;                  //sw1LenTmp = SW2Len
        }
    }
    else if( numSWrem <= (sw0Len>>1) )
    {
        numsw0Tmp = numSW-2;
        numSWrem = subFrameLen - (numSW-2)*sw0Len;
        if((numSWrem) & 0x3)
        {
            sw1LenTmp = 2*(numSWrem>>2) + 1;   //stored value is (sw1_length -1)
            sw2LenSelTmp = 2;                  //sw1LenTmp > SW2Len
        }
        else
        {
            sw1LenTmp = (numSWrem>>1) - 1;     //stored value is (sw1_length -1)
            sw2LenSelTmp = 1;                  //sw1LenTmp = SW2Len
        }
    }
    else
    {
        numsw0Tmp = numSW-1;
        sw1LenTmp = numSWrem - 1;              //stored value is (sw1_length -1)
        sw2LenSelTmp = 0;                      //SW2 is Off.
    }


    *sw0LenSel  = (Uint8) sw0LenSelTmp;
    *sw1Len     = (Uint8) sw1LenTmp;
    *sw2LenSel  = (Uint8) sw2LenSelTmp;
    *numsw0     = (Uint8) numsw0Tmp;
    *sw0NomLen  = (Uint8) sw0Len;

    return ( status );

}






