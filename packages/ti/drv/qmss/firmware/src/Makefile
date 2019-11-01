#  (C) Copyright 2012-2015 Texas Instruments, Inc.
# 
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions 
#  are met:
#
#    Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the 
#    documentation and/or other materials provided with the   
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


PASM = pasm_dos -BdlXmcL
PASM_BE = $(PASM) -E  -D_BE_
PASM_LE = $(PASM)     -D_LE_

.PHONY: all_ks1 all_ks2 all

all: 	all_ks1 all_ks2

all_ks2: ../v1/acc16_le_bin.h                  \
	 ../v1/acc32_le_bin.h                  \
	 ../v1/acc48_le_bin.h                  \
	 ../v1/acc16_be_bin.h                  \
	 ../v1/acc32_be_bin.h                  \
	 ../v1/acc48_be_bin.h                  \
	 ../v1/qos_le_bin.h                    \
	 ../v1/qos_be_bin.h                    \
	 ../v1/qos_sched_le_bin.h              \
	 ../v1/qos_sched_wide_le_bin.h         \
	 ../v1/qos_sched_drop_sched_le_bin.h   \
	 ../v1/qos_sched_be_bin.h              \
	 ../v1/qos_sched_wide_be_bin.h         \
	 ../v1/qos_sched_drop_sched_be_bin.h   \
	 ../v1/sriortr_be_bin.h                \
	 ../v1/sriortr_le_bin.h

all_ks1: ../v0/acc16_le_bin.h                  \
	 ../v0/acc32_le_bin.h                  \
	 ../v0/acc48_le_bin.h                  \
	 ../v0/acc16_be_bin.h                  \
	 ../v0/acc32_be_bin.h                  \
	 ../v0/acc48_be_bin.h                  \
	 ../v0/qos_le_bin.h                    \
	 ../v0/qos_be_bin.h                    \
	 ../v0/qos_sched_le_bin.h              \
	 ../v0/qos_sched_wide_le_bin.h         \
	 ../v0/qos_sched_drop_sched_le_bin.h   \
	 ../v0/qos_sched_be_bin.h              \
	 ../v0/qos_sched_wide_be_bin.h         \
	 ../v0/qos_sched_drop_sched_be_bin.h   \
	 ../v0/sriortr_be_bin.h                \
	 ../v0/sriortr_le_bin.h

../v0/acc16_le_bin.h ../v0/acc16_le.bib: copyright.txt acc16_le_hdr.txt acc16_le_v0_bin.h acc16_le_ftr.txt
	cat $^ > $@
	cp acc16_le_v0.bib ../v0/acc16_le.bib

../v1/acc16_le_bin.h ../v1/acc16_le.bib: copyright.txt acc16_le_hdr.txt acc16_le_v1_bin.h acc16_le_ftr.txt
	cat $^ > $@
	cp acc16_le_v1.bib ../v1/acc16_le.bib

../v0/acc32_le_bin.h ../v0/acc32_le.bib: copyright.txt acc32_le_hdr.txt acc32_le_v0_bin.h acc32_le_ftr.txt
	cat $^ > $@
	cp acc32_le_v0.bib ../v0/acc32_le.bib

../v1/acc32_le_bin.h ../v1/acc32_le.bib: copyright.txt acc32_le_hdr.txt acc32_le_v1_bin.h acc32_le_ftr.txt
	cat $^ > $@
	cp acc32_le_v1.bib ../v1/acc32_le.bib

../v0/acc48_le_bin.h ../v0/acc48_le.bib: copyright.txt acc48_le_hdr.txt acc48_le_v0_bin.h acc48_le_ftr.txt
	cat $^ > $@
	cp acc48_le_v0.bib ../v0/acc48_le.bib

../v1/acc48_le_bin.h ../v1/acc48_le.bib: copyright.txt acc48_le_hdr.txt acc48_le_v1_bin.h acc48_le_ftr.txt
	cat $^ > $@
	cp acc48_le_v1.bib ../v1/acc48_le.bib

../v0/acc16_be_bin.h ../v0/acc16_be.bib: copyright.txt acc16_be_hdr.txt acc16_be_v0_bin.h acc16_be_ftr.txt
	cat $^ > $@
	cp acc16_be_v0.bib ../v0/acc16_be.bib

../v1/acc16_be_bin.h ../v1/acc16_be.bib: copyright.txt acc16_be_hdr.txt acc16_be_v1_bin.h acc16_be_ftr.txt
	cat $^ > $@
	cp acc16_be_v1.bib ../v1/acc16_be.bib

../v0/acc32_be_bin.h ../v0/acc32_be.bib: copyright.txt acc32_be_hdr.txt acc32_be_v0_bin.h acc32_be_ftr.txt
	cat $^ > $@
	cp acc32_be_v0.bib ../v0/acc32_be.bib

../v1/acc32_be_bin.h ../v1/acc32_be.bib: copyright.txt acc32_be_hdr.txt acc32_be_v1_bin.h acc32_be_ftr.txt
	cat $^ > $@
	cp acc32_be_v1.bib ../v1/acc32_be.bib

../v0/acc48_be_bin.h ../v0/acc48_be.bib: copyright.txt acc48_be_hdr.txt acc48_be_v0_bin.h acc48_be_ftr.txt
	cat $^ > $@
	cp acc48_be_v0.bib ../v0/acc48_be.bib

../v1/acc48_be_bin.h ../v1/acc48_be.bib: copyright.txt acc48_be_hdr.txt acc48_be_v1_bin.h acc48_be_ftr.txt
	cat $^ > $@
	cp acc48_be_v1.bib ../v1/acc48_be.bib

../v0/qos_le_bin.h ../v0/qos_le.bib: copyright.txt qos_le_hdr.txt qos_le_v0_bin.h qos_le_ftr.txt
	cat $^ > $@
	cp qos_le_v0.bib ../v0/qos_le.bib

../v1/qos_le_bin.h ../v1/qos_le.bib: copyright.txt qos_le_hdr.txt qos_le_v1_bin.h qos_le_ftr.txt
	cat $^ > $@
	cp qos_le_v1.bib ../v1/qos_le.bib

../v0/qos_be_bin.h ../v0/qos_be.bib: copyright.txt qos_be_hdr.txt qos_be_v0_bin.h qos_be_ftr.txt
	cat $^ > $@
	cp qos_be_v0.bib ../v0/qos_be.bib

../v1/qos_be_bin.h ../v1/qos_be.bib: copyright.txt qos_be_hdr.txt qos_be_v1_bin.h qos_be_ftr.txt
	cat $^ > $@
	cp qos_be_v1.bib ../v1/qos_be.bib

../v0/qos_sched_le_bin.h ../v0/qos_sched_le.bib: copyright.txt qos_sched_le_hdr.txt qos_sched_le_v0_bin.h qos_sched_le_ftr.txt
	cat $^ > $@
	cp qos_sched_le_v0.bib ../v0/qos_sched_le.bib

../v1/qos_sched_le_bin.h ../v1/qos_sched_le.bib: copyright.txt qos_sched_le_hdr.txt qos_sched_le_v1_bin.h qos_sched_le_ftr.txt
	cat $^ > $@
	cp qos_sched_le_v1.bib ../v1/qos_sched_le.bib

../v0/qos_sched_be_bin.h ../v0/qos_sched_be.bib: copyright.txt qos_sched_be_hdr.txt qos_sched_be_v0_bin.h qos_sched_be_ftr.txt
	cat $^ > $@
	cp qos_sched_be_v0.bib ../v0/qos_sched_be.bib

../v1/qos_sched_be_bin.h ../v1/qos_sched_be.bib: copyright.txt qos_sched_be_hdr.txt qos_sched_be_v1_bin.h qos_sched_be_ftr.txt
	cat $^ > $@
	cp qos_sched_be_v1.bib ../v1/qos_sched_be.bib

../v0/qos_sched_wide_le_bin.h ../v0/qos_sched_wide_le.bib: copyright.txt qos_sched_wide_le_hdr.txt qos_sched_wide_le_v0_bin.h qos_sched_wide_le_ftr.txt
	cat $^ > $@
	cp qos_sched_wide_le_v0.bib ../v0/qos_sched_wide_le.bib

../v1/qos_sched_wide_le_bin.h ../v1/qos_sched_wide_le.bib: copyright.txt qos_sched_wide_le_hdr.txt qos_sched_wide_le_v1_bin.h qos_sched_wide_le_ftr.txt
	cat $^ > $@
	cp qos_sched_wide_le_v1.bib ../v1/qos_sched_wide_le.bib

../v0/qos_sched_wide_be_bin.h ../v0/qos_sched_wide_be.bib: copyright.txt qos_sched_wide_be_hdr.txt qos_sched_wide_be_v0_bin.h qos_sched_wide_be_ftr.txt
	cat $^ > $@
	cp qos_sched_wide_be_v0.bib ../v0/qos_sched_wide_be.bib

../v1/qos_sched_wide_be_bin.h ../v1/qos_sched_wide_be.bib: copyright.txt qos_sched_wide_be_hdr.txt qos_sched_wide_be_v1_bin.h qos_sched_wide_be_ftr.txt
	cat $^ > $@
	cp qos_sched_wide_be_v1.bib ../v1/qos_sched_wide_be.bib

../v0/qos_sched_drop_sched_le_bin.h ../v0/qos_sched_drop_sched_le.bib: copyright.txt qos_sched_drop_sched_le_hdr.txt qos_sched_drop_sched_le_v0_bin.h qos_sched_drop_sched_le_ftr.txt
	cat $^ > $@
	cp qos_sched_drop_sched_le_v0.bib ../v0/qos_sched_drop_sched_le.bib

../v1/qos_sched_drop_sched_le_bin.h ../v1/qos_sched_drop_sched_le.bib: copyright.txt qos_sched_drop_sched_le_hdr.txt qos_sched_drop_sched_le_v1_bin.h qos_sched_drop_sched_le_ftr.txt
	cat $^ > $@
	cp qos_sched_drop_sched_le_v1.bib ../v1/qos_sched_drop_sched_le.bib

../v0/qos_sched_drop_sched_be_bin.h ../v0/qos_sched_drop_sched_be.bib: copyright.txt qos_sched_drop_sched_be_hdr.txt qos_sched_drop_sched_be_v0_bin.h qos_sched_drop_sched_be_ftr.txt
	cat $^ > $@
	cp qos_sched_drop_sched_be_v0.bib ../v0/qos_sched_drop_sched_be.bib

../v1/qos_sched_drop_sched_be_bin.h ../v1/qos_sched_drop_sched_be.bib: copyright.txt qos_sched_drop_sched_be_hdr.txt qos_sched_drop_sched_be_v1_bin.h qos_sched_drop_sched_be_ftr.txt
	cat $^ > $@
	cp qos_sched_drop_sched_be_v1.bib ../v1/qos_sched_drop_sched_be.bib

../v0/sriortr_le_bin.h ../v0/sriortr_le.bib: copyright.txt sriortr_le_hdr.txt sriortr_le_v0_bin.h sriortr_le_ftr.txt
	cat $^ > $@
	cp sriortr_le_v0.bib ../v0/sriortr_le.bib

../v1/sriortr_le_bin.h ../v1/sriortr_le.bib: copyright.txt sriortr_le_hdr.txt sriortr_le_v1_bin.h sriortr_le_ftr.txt
	cat $^ > $@
	cp sriortr_le_v1.bib ../v1/sriortr_le.bib

../v0/sriortr_be_bin.h ../v0/sriortr_be.bib: copyright.txt sriortr_be_hdr.txt sriortr_be_v0_bin.h sriortr_be_ftr.txt
	cat $^ > $@
	cp sriortr_be_v0.bib ../v0/sriortr_be.bib

../v1/sriortr_be_bin.h ../v1/sriortr_be.bib: copyright.txt sriortr_be_hdr.txt sriortr_be_v1_bin.h sriortr_be_ftr.txt
	cat $^ > $@
	cp sriortr_be_v1.bib ../v1/sriortr_be.bib

acc16_le_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL16_ -DKEYSTONE1 -Cacc16_le accnew.p acc16_le_v0

acc16_le_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL16_ -DKEYSTONE2 -Cacc16_le accnew.p acc16_le_v1

acc32_le_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL32_ -DKEYSTONE1 -Cacc32_le accnew.p acc32_le_v0

acc32_le_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL32_ -DKEYSTONE2 -Cacc32_le accnew.p acc32_le_v1

acc48_le_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL48_ -DKEYSTONE1 -Cacc48_le accnew.p acc48_le_v0

acc48_le_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_LE) -D_CHANNEL48_ -DKEYSTONE2 -Cacc48_le accnew.p acc48_le_v1

acc16_be_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL16_ -DKEYSTONE1 -Cacc16_be accnew.p acc16_be_v0

acc16_be_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL16_ -DKEYSTONE2 -Cacc16_be accnew.p acc16_be_v1

acc32_be_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL32_ -DKEYSTONE1 -Cacc32_be accnew.p acc32_be_v0

acc32_be_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL32_ -DKEYSTONE2 -Cacc32_be accnew.p acc32_be_v1

acc48_be_v0_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL48_ -DKEYSTONE1 -Cacc48_be accnew.p acc48_be_v0

acc48_be_v1_bin.h: Makefile accnew.p pm_config.h
	$(PASM_BE) -D_CHANNEL48_ -DKEYSTONE2 -Cacc48_be accnew.p acc48_be_v1

qos_le_v0_bin.h: Makefile qosrr.p pm_config.h
	$(PASM_LE) -DKEYSTONE1 -Cqos_le qosrr.p qos_le_v0

qos_le_v1_bin.h: Makefile qosrr.p pm_config.h
	$(PASM_LE) -DKEYSTONE2 -Cqos_le qosrr.p qos_le_v1

qos_be_v0_bin.h: Makefile qosrr.p pm_config.h
	$(PASM_BE) -DKEYSTONE1 -Cqos_be qosrr.p qos_be_v0

qos_be_v1_bin.h: Makefile qosrr.p pm_config.h
	$(PASM_BE) -DKEYSTONE2 -Cqos_be qosrr.p qos_be_v1

qos_sched_le_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DSIMUBYTEPKT -DMULTIGROUP -DKEYSTONE1 -Cqos_sched_le qos_sched.p qos_sched_le_v0

qos_sched_le_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DSIMUBYTEPKT -DMULTIGROUP -DKEYSTONE2 -Cqos_sched_le qos_sched.p qos_sched_le_v1

qos_sched_be_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DSIMUBYTEPKT -DMULTIGROUP -DKEYSTONE1 -Cqos_sched_be qos_sched.p qos_sched_be_v0

qos_sched_be_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DSIMUBYTEPKT -DMULTIGROUP -DKEYSTONE2 -Cqos_sched_be qos_sched.p qos_sched_be_v1

qos_sched_wide_le_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DWIDE -DMULTIGROUP -DKEYSTONE1 -Cqos_sched_wide_le qos_sched.p qos_sched_wide_le_v0

qos_sched_wide_le_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DWIDE -DMULTIGROUP -DKEYSTONE2 -Cqos_sched_wide_le qos_sched.p qos_sched_wide_le_v1

qos_sched_wide_be_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DWIDE -DMULTIGROUP -DKEYSTONE1 -Cqos_sched_wide_be qos_sched.p qos_sched_wide_be_v0

qos_sched_wide_be_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DWIDE -DMULTIGROUP -DKEYSTONE2 -Cqos_sched_wide_be qos_sched.p qos_sched_wide_be_v1

qos_sched_drop_sched_le_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DDROP_SCHED -DKEYSTONE1 -Cqos_sched_drop_sched_le qos_sched.p qos_sched_drop_sched_le_v0

qos_sched_drop_sched_le_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_LE) -DDROP_SCHED -DKEYSTONE2 -Cqos_sched_drop_sched_le qos_sched.p qos_sched_drop_sched_le_v1

qos_sched_drop_sched_be_v0_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DDROP_SCHED -DKEYSTONE1 -Cqos_sched_drop_sched_be qos_sched.p qos_sched_drop_sched_be_v0

qos_sched_drop_sched_be_v1_bin.h: Makefile qos_sched.p pm_config.h
	$(PASM_BE) -DDROP_SCHED -DKEYSTONE2 -Cqos_sched_drop_sched_be qos_sched.p qos_sched_drop_sched_be_v1

sriortr_le_v0_bin.h: Makefile sriortr.p pm_config.h
	$(PASM_LE) -DKEYSTONE1 -Csriortr_le sriortr.p sriortr_le_v0

sriortr_le_v1_bin.h: Makefile sriortr.p pm_config.h
	$(PASM_LE) -DKEYSTONE2 -Csriortr_le sriortr.p sriortr_le_v1

sriortr_be_v0_bin.h: Makefile sriortr.p pm_config.h
	$(PASM_BE) -DKEYSTONE1 -Csriortr_be sriortr.p sriortr_be_v0

sriortr_be_v1_bin.h: Makefile sriortr.p pm_config.h
	$(PASM_BE) -DKEYSTONE2 -Csriortr_be sriortr.p sriortr_be_v1


