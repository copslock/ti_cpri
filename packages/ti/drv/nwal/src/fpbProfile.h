/*
 *  @file   emureg.h
 *
 *  @brief  File for profiling using emulation registers
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 *    This software is licensed under the standard terms and conditions in the 
 *  Texas Instruments Incorporated Technology and Software Publicly Available 
 *  Software License Agreement, a copy of which is included in the software 
 *  download.
 */
#ifndef EMUREG
#define EMUREG

extern cregister volatile unsigned int TSCL;
extern cregister volatile unsigned int TSCH;
extern cregister volatile unsigned int CSR;
extern cregister volatile unsigned int TSCH;
extern void thwCriticalBegin (void);
extern void thwCriticalEnd (void);
#define thwCriticalBeginFast() _disable_interrupts()
#define thwCriticalEndFast(x) _restore_interrupts(x)
#define GIE         0

#define EMU_MIPS_BUF_WORDS 0x0800000
#define EMU_MIPS_DUMMY_WORDS 8

#define EMU_PROFILE_TYPE_ISR   0
#define EMU_PROFILE_TYPE_TASK  1
#define EMU_PROFILE_TYPE_MOD   2
#define EMU_PROFILE_TYPE_SYS   3
#define EMU_PROFILE_TYPE_IDLE  4
#define EMU_PROFILE_TYPE_BWAIT 5
#define EMU_PROFILE_TYPE_END   6

#define EMU_TASK_ID_MARK_END  0 /* Mark end of block, no output */

/* Keeping 2+x for now, so perl script keeps txrx task */
#define EMU_TASK_ID_RX_TASK(x) (2+x),EMU_PROFILE_TYPE_TASK
#define EMU_TASK_ID_RX_MODULE1     4,EMU_PROFILE_TYPE_MOD
#define EMU_TASK_ID_RX_MODULE2     5,EMU_PROFILE_TYPE_MOD
#define EMU_TASK_ID_MODULE3      6,EMU_PROFILE_TYPE_IDLE
#define EMU_TASK_ID_TX_MODULE1     7,EMU_PROFILE_TYPE_MOD
#define EMU_TASK_ID_TX_MODULE2     8,EMU_PROFILE_TYPE_MOD
#define EMU_TASK_ID_TIMER(x)   (9+x),EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_PKT_ISR     12,EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_COMMON_POLL 13,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_PKT_TX     14,EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_MODULE4     15,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_MODULE5    16,EMU_PROFILE_TYPE_SYS 
#define EMU_TASK_ID_PRD      17,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_MODULE6  18,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_MAIN     19,EMU_PROFILE_TYPE_SYS

#define EMU_TASK_ID_MIPSWASTER  20 ,EMU_PROFILE_TYPE_IDLE
#define EMU_TASK_ID_MODULE7 21 ,EMU_PROFILE_TYPE_SYS

#define EMU_TASK_ID_GMACCOMMISR 22,EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_GMACTXISR   23,EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_GMACRXISR   24,EMU_PROFILE_TYPE_ISR
#define EMU_TASK_ID_PACKET_POLL    25,EMU_PROFILE_TYPE_SYS

#define EMU_TASK_ID_NETCP_RX_MODULE1    26,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_NETCP_RX_MODULE2    27,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_NETCP_RX_MODULE3    28,EMU_PROFILE_TYPE_SYS

#define EMU_TASK_ID_NETCP_TX_MODULE1    29,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_NETCP_TX_MODULE2    30,EMU_PROFILE_TYPE_SYS
#define EMU_TASK_ID_NETCP_TX_MODULE3    31,EMU_PROFILE_TYPE_SYS

#define EMU_PROF_NOPUSHPOP 0
#define EMU_PROF_PUSH      1
#define EMU_PROF_POP       2
#define EMU_PROFILE_MAX    128

#define EMU_PROFILE_BEGIN(sect,ch) {                                         \
                                     UInt16 emu_profile_ch = ch,              \
                                     emu_profile_type =                      \
                                       emu_profile_mark (sect,               \
                                                         /* sect has type */ \
                                                         emu_profile_ch,     \
                                                         EMU_PROF_PUSH);
#define EMU_PROFILE_END()            emu_profile_mark (EMU_TASK_ID_MARK_END, \
                                                       emu_profile_type,     \
                                                       emu_profile_ch,       \
                                                       EMU_PROF_POP);        \
                                   }                                  


void emu_profile_change_mode(void);
void emu_profile_flush(void);
void emu_profile_mark_i(UInt16 task_id, UInt16 ch, UInt16 pushpop);

void emu_profile_mark_f(UInt16 task_id, UInt16 ch, UInt16 pushpop);

static inline Int16 emu_profile_mark(UInt16 task_id, UInt16 type,UInt16 ch, UInt16 pushpop) {
  extern UInt16 emu_profile_run_mode;

  emu_profile_mark_f(task_id, ch, pushpop);
  return(type);
}
#define MAX_PROFILE_POINTS 30
#define MAX_NUM_PROFILE_CHANNELS 10

typedef struct emu_profile_data_s {
  UInt16 noOfHits;
  UInt32 start_time;
  UInt32  profile_cycles;
  UInt32 mips_measured;
  UInt32 tmp_cycles;
  UInt32 min_cycles;
  UInt32 max_cycles;
  UInt32 average_cycles;
  UInt16 averageNoOfHits;
} emu_profile_data_t;

typedef struct profile_stack_entry_s {
  UInt8 chan_id;
  UInt8 task_id; 
}profile_stack_entry_t;

typedef struct profileChannelMips_s {
  UInt32 average_mips;
  UInt32 min_mips;
  UInt32 max_mips;
} profileChannelMips_t;

extern emu_profile_data_t emu_profile_data[MAX_NUM_PROFILE_CHANNELS+1][MAX_PROFILE_POINTS];
extern UInt8 profile_stack_index;
extern profile_stack_entry_t profile_stack[MAX_PROFILE_POINTS];

void fpb_profile_periodic_update(void);
void fpb_profile_reset_mips_count (void);
void fpb_profile_prd_function(void);

#endif //EMUREG
