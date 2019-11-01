*************************************************************************
*	FFTC Driver Unit Test Suite 
*	---------------------------------
*
*	(C) Copyright 2009, 2014, Texas Instruments, Inc.
*
*	Use of this software is controlled by the terms and conditions found 
*	in the license agreement under which this software has been supplied.
*************************************************************************

The FFTC Driver Unit Test Suite is aimed at testing all the layers of FFTC driver, 
i.e, CSLR, LLD and the FFTC higher layer APIs.

-------------------------
Unit Tests Description:
-------------------------
The Unit tests here test the following:

	* Verify the defaults for all FFTC Registers defined in FFTC CSLR at reset against the spec.

	* Test all the FFTC Low Level driver (LLD) APIs working.

	* Test all the FFTC Higher layer APIs using CPPI and QMSS libraries. Run tests to verify various use cases. 
	  Specifically, the tests here test for:
		* Various sample sizes (16, 48, 540, 2048) with multiple FFT blocks
		* Single core testing 
		* Multi core testing
		* Host mode and Monolithic descriptors
        * Protocol specific pass through Info (PS Info)
        * Various FFT Configurations such as Zero padding, Variable shifting, cyclic prefix addition, mixed size DFT list
          configuration.

Following is a description of the test contained in each of the test files. 

    Test File                   Test Description
    ----------                  ------------------
    test_lld.c                  Tests all register access APIs exposed by LLD for FFTC_A and FFTC_B instances.

    test_singlecore.c           Single core test using Host descriptors + No PS Info using FFTC_A + using interrupts.
                                Tests following using all 4 FFTC Tx queues:
                                16 block size + 5 blocks
                                48 block size + 5 blocks
                                540 block size + 3 blocks
                                2048 block size + 1 block

    test_singlecore_poll.c      Single core test using Host descriptors + No PS Info using FFTC_A + using polling.
                                When run on Core 0 uses 16 block size + 5 blocks + 10 packets; 
                                When run on Core 1 uses 48 block size + 5 blocks + 10 packets; 
                                When run on Core 2 uses 540 block size + 5 blocks + 10 packets; 
                                When run on Core 3 uses 2048 block size + 4 blocks + 10 packets; 

    test_singlecore_psinfo.c    Single core test using Host descriptors + 4 words of PS Info using FFTC_A + using interrupts.
                                When run on Core 0 uses 16 block size + 5 blocks + 10 packets; 
                                When run on Core 1 uses 48 block size + 5 blocks + 10 packets; 
                                When run on Core 2 uses 540 block size + 5 blocks + 10 packets; 
                                When run on Core 3 uses 2048 block size + 4 blocks + 10 packets; 

    test_singlecore_task.c      Single core test using Host descriptors + 4 words of PS Info using FFTC_A + using interrupts.
                                Creates separate Rx, Tx tasks and runs the tests in a loop.
                                When run on Core 0 uses 16 block size + 5 blocks + 10 packets; 
                                When run on Core 1 uses 48 block size + 5 blocks + 10 packets; 
                                When run on Core 2 uses 540 block size + 5 blocks + 10 packets; 
                                When run on Core 3 uses 2048 block size + 4 blocks + 10 packets; 

    test_multicore.c            Multi-core test. Must be load and run on all 4 cores set in "synchronous mode".
                                Uses Host descriptors + No PS Info + interrupts + FFTC_A instance +
                                16 block size + 5 blocks + 2 packets.

    test_mono_singlecore.c      Single core test using Monolithic descriptors + No PS Info using FFTC_A + using interrupts.
                                Tests following using all 4 FFTC Tx queues:
                                16 block size + 5 blocks
                                48 block size + 5 blocks
                                540 block size + 3 blocks
                                2048 block size + 1 block

    test_mono_singlecore_psinfo.c      
                                Single core test using Monolithic descriptors + 4 words of PS Info using FFTC_A + using interrupts.
                                Tests following using all 4 FFTC Tx queues:
                                16 block size + 5 blocks
                                48 block size + 5 blocks
                                540 block size + 3 blocks
                                2048 block size + 1 block

    test_singlecore_dftlist.c   Tests mixed size DFT list configuration. Uses Host descriptors + No PS Info + interrupts + FFTC_A instance +
                                20 blocks of mixed sizes.


    test_singlecore_shift.c     Runs the following test combinations on all 4 FFTC Tx queues:
                                1024 block size +  1 block + variable input shifting 
                                1024 block size +  1 block + zero pad addition 
                                1024 block size +  1 block + cyclic prefix addition 
                                1024 block size +  1 block + zero pad addition + Variable input shifting
                                1024 block size +  1 block + zero pad addition + Variable input shifting + Cyclic Prefix addition


    *   All the above tests have been run on FFTC_B instance too.
    *   All PS info tests have been done with PS info both in Descriptor and SOP positions.

    *   Note, that for all tests and projects, the same .out file should be
        loaded on all (4) cores and run. 

-------------------------
Future Extensions:
-------------------------
Due to lack of simulator support / bugs the following havent been tested:
	* To/From AIF use cases

-------------------------
Prerequisites
-------------------------

Please refer to the Release Notes for these details.

-------------------------
CCS Environment
-------------------------

Please refer to the Release Notes for these details.

-------------------------
Project Setup
-------------------------

Please refer to the Release Notes for these details.
