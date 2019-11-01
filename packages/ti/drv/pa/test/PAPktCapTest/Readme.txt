*************************************************************************
*	Packet Capture Single Core Example 
*	---------------------------------
*
*	(C) Copyright 2014, Texas Instruments, Inc.
*
*	Use of this software is controlled by the terms and conditions found 
*	in the license agreement under which this software has been supplied.
*************************************************************************

The Packet Capture (HOST)/Port Mirror (EMAC) Single core example demonstrates the use of CPSW CSL APIs to 
configure the ethernet switch on target (silicon and simulator). It further demonstrates 
the use of CPPI/QMSS/PA LLDs to send/receive data through the ethernet switch using 
Host descriptors on any given DSP core and do either an ingress/egress packet capture or port mirror.

The example is designed to run in loopback modes, i.e. it expects to receive the transmitted packet.
The following two loopback modes are supported:
    CPSW_LOOPBACK_INTERNAL(default): The transmitted packet is loopbacked at the SGMII through SGMII internal loopback
    CPSW_LOOPBACK_EXTERNAL: The transmitted packet should bo loopbacked by application outsite the SoC
    
This example also demonstrates the normal CPSW configuration as CPSW_LOOPBACK_NONE mode. In this configuration, it is up to the user
to modify PASS configuration to receive desired ingress packets.

Check the release notes for prerequisites, version information and steps on how to 
run examples

-------------------------
Execute Project
-------------------------

1. Simulator configuration

In order for the example to work in external loopback mode successfully, i.e. to be able to send/receive data packets from the wire, 
Please refer to http://ap-fpdsp-swapps.dal.design.ti.com/index.php/66AK2H12_Device_Simulator_User_Guide#Configurable_parameters_for_simulation
to configure the following EMAC config params in CCSEup
    a) EMAC Adaptor Enable: ON
    b) EMAC Network Adaptor: Broadcom (NIC card using in PC)
    c) EMAC MAC Adress Port 0: other
       EMAC_MAC_ADDR_PORT0: 10-11-12-13-14-15
       
2. Execute using CCS
    
Launch the CCS Debugger and go to the Debug Perspective.

To execute the project ensure the following is done:-
    a) System reset (applicable for EVM under NO_BOOT mode only)
    b) Run EVM specific GEL scripts (applicable for EVM under NO_BOOT mode only)
    b) Load Program
    c) Once the project is loaded; Run to execute it.
    
NOTE:
If EVM is *NOT* running in NO_BOOT mode, the below assumptions are made
2a. The PDSPs are downloaded for QMSS and PA outside the example
2b. CPSW Switch is configured outside the example
2c. To disable the auto detect logic, please set 'autodetectLogic' variable to '0' using CCS. When this is done, the all the steps as mentioned in 'Execute' section are mandatory.
	
3. Result

The application will output to the console its status progress and the number of sent/received packets along with captured/cloned packets
and declare pass/fail.

4. Loading and Executing the project using MPM Client Utility - Not supported
    
    
    
