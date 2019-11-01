/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* return the entry point to the code if the code is authenticated 
 * null otherwise
*/
uint32_t SBL_authentication(void *addr);

uint32_t SBL_mon_init(uint32_t freq);
uint32_t SBL_startSK(void);
uint32_t SBL_startDspSecSrv(void);

int32_t SBL_MemRead(void  *buff, void  *srcAddr, uint32_t size);

/**
 *  \brief    SBL_MemSeek function to move the read head by n bytes
 *
 *  \param    srcAddr - Read head pointer
 *  \param    location - Move the read head pointer by n bytes
 *
 * \return  none
 */
void SBL_MemSeek(void *srcAddr, uint32_t location);


/* this whole structure fits inside sblbootbuff segment in memory */
#define READ_BUFF_SIZE          (512)
#define SBL_MAX_BOOT_BUFF_SIZE  (0x4000000-0x10)

typedef struct
{
    uint8_t     sbl_boot_buff[SBL_MAX_BOOT_BUFF_SIZE+1];
    uint32_t    sbl_boot_size;
    uint32_t    sbl_boot_buff_idx;
} SBL_incomingBootData_S;


#define TOC_FILE_NAME_OFFSET            (20)
#define TOC_HDR_SIZE                    (32)
#define TOC_DAT_SIZE_OFFSET             (4)

/* this following GEM MAGIC address is SOC dependent */
#define DSP_BOOT_MAGIC                  (0x8FFFFC)
#define NGEMS                           (1)
#define GEM_MAGIC(core)                 ((1<<28)|((core)<<24)|DSP_BOOT_MAGIC)

#define SYS_IPCGR                       (0x02620240) /* IPCGR registers */
#define SYS_IPAGR                       (0x02620280) /* IPCAR registers */
#define GEM_SK_START_ADDR               (0x10840000) /* The RAM SK run address */ 
#define GEM_SEC_SRV_START_ADDR          (0x10860000) /* The DSP sec. server load  addr */ 
