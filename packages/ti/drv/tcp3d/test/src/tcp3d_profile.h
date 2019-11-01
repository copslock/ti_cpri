#ifndef _TCP3D_PROFILE_H_
#define _TCP3D_PROFILE_H_

/**
 *  \file   tcp3d_profile.h
 *
 *  \brief  TCP3D test profile header.
 *
 *  Copyright (C) Texas Instruments Incorporated 2009
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/**
 *  Include Files
 */
#include <stdio.h>
#include <xdc/std.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/*
 * Profile taps for plotting with Matlab 
 */
#define  PROF_START             0x00000100
#define  PROF_STOP              0x00000200
#define  PROF_INST              0x00000300

#define  PROF_SOFT_RESET        0x00010000
#define  PROF_RESTART_PING      0x00020000
#define  PROF_RESTART_PONG      0x00030000
#define  PROF_SEND_TASK         0x00040000
#define  PROF_PREPROC           0x00050000
#define  PROF_ENQUE_FUNC        0x00060000
#define  PROF_START_FUNC        0x00070000
#define  PROF_PING_DECODE       0x00080000
#define  PROF_PONG_DECODE       0x00090000
#define  PROF_POST_RECV_PING    0x000A0000
#define  PROF_POST_RECV_PONG    0x000B0000
#define  PROF_RECV_TASK         0x000C0000
#define  PROF_INIT_FUNC         0x000D0000

#define  PROF_TAG_LEN           4000

typedef struct PROFILE_TAG
{
    UInt32 tag;
    UInt32 time;
} PROFILE_TAG;

/**
 * extern variables (see tcp3d_main.c)
 */
extern volatile PROFILE_TAG    profileTag[PROF_TAG_LEN];
extern volatile UInt32         profileTagInd;

/**
 * Profile Macro Functions 
 */
#define PROF_LOG_COMPLETE(instNum) {                                \
                FILE *fid;                                          \
                Int i;                                              \
                if (instNum == CSL_TCP3D_0)                         \
                    fid = fopen("c:\\profile_A.dat","w");           \
                else                                                \
                    fid = fopen("c:\\profile_B.dat","w");           \
                fprintf(fid, "0 0 0 0 %x\n", 2*profileTagInd);      \
                for(i=0; i<profileTagInd; i++)                      \
                {                                                   \
                    fprintf(fid, "0x%08x\n", profileTag[i].tag);    \
                    fprintf(fid, "0x%08x\n", profileTag[i].time);   \
                }                                                   \
                fclose(fid);                                        \
            }

#define PROF_LOG_INIT(instNum) {                                    \
                FILE *fidL;                                         \
                if (instNum == CSL_TCP3D_0)                         \
                    fidL = fopen("c:\\Tcp3dDebug_A.log","w");       \
                else                                                \
                    fidL = fopen("c:\\Tcp3dDebug_B.log","w");       \
                fclose(fidL);                                       \
                profileTagInd = 0;                                  \
            }

#if 0
#define LOG_TIME(TASK, TAG, TIME) {                                 \
                UInt32  cookie = 0;                                 \
                if(profileTagInd<PROF_TAG_LEN)                      \
                {                                                   \
                    cookie = Hwi_disable();                         \
                    profileTag[profileTagInd].tag = (TASK | TAG);   \
                    profileTag[profileTagInd++].time = TIME;        \
                    Hwi_restore(cookie);                            \
                }                                                   \
            }
#else
#define LOG_TIME(TASK, TAG, TIME) LOG_TIME_ISR(TASK, TAG, TIME)
#endif

#define LOG_TIME_ISR(TASK, TAG, TIME) {                             \
                if(profileTagInd<PROF_TAG_LEN)                      \
                {                                                   \
                    profileTag[profileTagInd].tag = (TASK | TAG);   \
                    profileTag[profileTagInd++].time = TIME;        \
                }                                                   \
            }

#endif /* _TCP3D_PROFILE_H_ */
