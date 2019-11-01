#ifndef _TCP3D_CODE_BLK_SEG_H_
#define _TCP3D_CODE_BLK_SEG_H_

#ifndef USE_TCP3D_DRIVER_TYPES
#include "swpform.h"
#else
#include <ti/csl/tistdtypes.h>
#include <tcp3d_drv_types.h>
#endif

/** 
 *  \fn     Int32  TCP3D_codeBlkSeg (
 *                    IN  Uint32  blockLengthK,
 *                    IN  Uint8   numMAP,
 *                    IN  Uint8  * const RESTRICT   sw0NomLen,
 *                    OUT Uint8  * const RESTRICT  	sw0LenSel,
 *                    OUT Uint8  * const RESTRICT  	sw1Len,
 *                    OUT Uint8  * const RESTRICT  	sw2LenSel,
 *                    OUT Uint8  * const RESTRICT  	numsw0)
 *  \brief   Calculates code block segmentation parameteres based on the block lenght and SW0 length.
 *           
 * 
 *  \param[in]    blockLengthK
 *              Code block length. (Number of information bits.)
 *              
 *
 *  \param[in]    numMAP
 *              Number of MAP decoders used. =1 for WCDMA, =2 for LTE and WiMAX
 *
 *  \param[in,out]    sw0NomLen
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
Int32  TCP3D_codeBlkSeg (
            IN  Uint32  blockLengthK,
            IN  Uint8   numMAP,
            IN  Uint8  * const RESTRICT     sw0NomLen,
            OUT Uint8  * const RESTRICT  	sw0LenSel,
            OUT Uint8  * const RESTRICT  	sw1Len,
            OUT Uint8  * const RESTRICT  	sw2LenSel,
            OUT Uint8  * const RESTRICT  	numsw0);
#endif
