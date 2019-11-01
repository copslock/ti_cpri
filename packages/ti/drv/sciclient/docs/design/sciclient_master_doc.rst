.. sachin documentation master file, created by
   sphinx-quickstart on Thu Jun  7 15:54:11 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. toctree::
  :maxdepth: 6


**************************
Sciclient Design Document
**************************

Introduction
=============

Overview
-----------

Traditional Texas Instruments SoCs implement system control functions such as power management within operating systems on each of the processing units (ARM/DSP etc). However the traditional approach has had tremendous challenges to ensure system stability. Few of the challenges include:

* Complex interactions between Operating Systems on heterogeneous SoCs for generic features.
* Lack of centralized knowledge of system state.
* Complex implementation challenges when implementing workarounds for SoC errata.
* Equivalent SoC power or device management entitlement on all variations of Operating Systems.

DMSC controls the power management of the device, hence is responsible for bringing the device out of
reset, enforce clock and reset rules. DMSC power management functions are critical to bring device to low
power modes, for example DeepSleep, and sense wake-up events to bring device back online to active
state.
There is one instance of DMSC in this family of devices - WKUP_DMSC0.

Texas Instruments' System Control Interface defines the communication protocol between various processing entities to the System Control Entity on TI SoCs. This is a set of message formats and sequence of operations required to communicate and get system services processed from System Control entity in the SoC.

More information regarding the TI-SCI is given `here <http://processors.wiki.ti.com/index.php/TISCI>`_ .

The SCIClient is an interface to the TI-SCI protocol for RTOS and non-OS based applications. It exposes the core message details, valid module/clock IDs to the higher level software and abstracts the communication with the firmware based on the TI-SCI protocol. These APIs can be called by power, resource and security RTOS drivers or any other non-OS or RTOS based higher level software to be able to communicate with DMSC for its services. The higher level software is expected to populate the necessary message core parameters. The SCIClient would wrap the core message with the necessary protocol header and forward this to the DMSC. The SCIClient relies on the CSL-FL layer to program and interact with the Secure Proxy Threads. The SCIClient's goal is to ensure there is no duplication of the interface to the DMSC from different software components which need to interact with the DMSC or other System Control Entities in future devices.



Text Conventions
------------------


=============      =============================
style/bullet       definition or explanation 
-------------      -----------------------------
*                  This bullet indicates important information. Please read such text carefully.
+                  This bullet indicates additional information.
=============      =============================


Terms and Abbreviation
------------------------


=====     =============================
term      definition or explanation
=====     =============================
DMSC      Device Management and Security Controller
SCI       System Control Interface
SYSFW     System Firmware
RA        Ring accelerator
PM        Power Management
RM        Resource Management
=====     =============================


References
-----------

.. csv-table::
   :file: images/references.csv
   :header-rows: 0
   :widths: 5, 10, 50

Requirements
==============

 Requirements at https://confluence.itg.ti.com/display/Drivers/SciClient+Granular+Requirements .


Assumptions
------------

1. The higher level software will populate the core message payload based on the message headers which will be exposed by the SCIClient. The higher level software should include these headers and would populate the core message and send this to the SCIClient.
2. The current implementation of the SCIClient is assumed to be blocking. Until decided later the SCIClient APIs will wait for a completion response from the DMSC firmware on completion of processing before exiting. The API will allow for context switch to other tasks while it waits for the service to complete. In the non-OS case this will be a spinlock.

Constraints
------------

The host can have multiple outstanding messages to the DMSC firmware. In order to keep track of what messages were being sent out we use the message count and an array to read back the response corresponding to the particular message count. This is especially important when interrupts are being used to understand if the message being received corresponds to the message that we sent. The array size is chosen to be a maximum of how many messages the core can possibly sent out based on the thread ID allocation from DMSC firmware. This may not be optimal for all cores for DDR less systems but is exposed through a macro which if required the user can optimize and re-build the library for. Static allocation is considered for the array hence the macro.

 
Design Description
===================

The SCIClient has two major functions:

1. Interact with DMSC ROM and load the DMSC Firmware. 
2. Pass on service requests from higher level software to the DMSC firmware and forward the response from DMSC firmware to the higher level software. 

The `Sciclient_loadfirmware`_ API is used to cater to the first requirement and the `Sciclient_service`_ is used to cater to the second. The SCIClient library requires initialization of the a handle which is used by the subsequent API calls. This handle is allocated by the higher level software and is initialized by the [Sciclient_init] function. Once the application/higher level software is being torn down or exiting the `Sciclient_deinit`_ can be used to de-initialize this handle. 

The SCIClient can operate in the following combinations:

1. Non-OS, Polling based message completion.
2. Non-OS, Interrupt Based message completion.
3. RTOS, Polling based message completion.
4. RTOS, Interrupt based message completion.

The SCIClient depends on the PDK OSAL layer to differentiate between the Non-OS and the RTOS implementation of Semaphores and Interrupts (HWIs). The build parameter of the OSAL library would determine if the application is bare metal or RTOS based. The polling versus interrupt based wait for message completion is a run time configuration passed during the SCIClient_init initialization.

All the APIs for interacting with the firmware are blocking with a specified timeout . A common API `Sciclient_service`_ is implemented for all types of calls to the firmware which takes 3 arguments : 

1. **pHandle**
2. **pInPrm**
3. **pOutPrm**

The API serves a particular request, based on the value of messageType parameter in **pInPrms**, whose response is given to the higher level API through **pOutPrms** . The **pInPrms** contains the required inputs from the higher level software corresponding to the message_type, timeout value and the core message as a byte stream. A pointer **pOutPrms** has to be passed to the sciclient ,which shall be modified by sciclient. 

The Sciclient shall be responsible for abstracting all interaction with proxy and RA .

Please refer `TISCI <http://processors.wiki.ti.com/index.php/TISCI>`_ for details of message manager protocol which is used for the requests and responses.


Component Interaction
----------------------

Sciclient interacts with CSL-FL modules, secure proxy and RA, to interact with the DMSC firmware . Higher level libraries like PM LIB , RM LIB and SECURITY LIB will use Sciclient_service API for interaction and will be responsible for  filling the core buffer of the request.

    .. image:: images/sciclientSoftwareStack.png
                :height: 300
                :width: 600


Dynamic behaviour
------------------

The high level sequence of operations and its interaction with over components of the Sciclient_service API  is described in fig.2:
    
    .. image:: images/sciclientSequenceDiag.png
                :height: 300
                :width: 600

Key Steps of the [Sciclient_service] API are:  

Construct Message
~~~~~~~~~~~~~~~~~~~

The message(header+payload) is constructed in normal memory instead of secure proxy memory .

*   Find hostId . Refer section 2.2 of `SYSFW <https://bitbucket.itg.ti.com/projects/SYSFW/repos/system-firmware/attachments/506ec01f0e/refman-public.pdf>`_ for hostIds .
*   Populate header flags(current support only for TI_SCI_FLAG_REQ_ACK_ON_PROCESSED) and create Message Header. For details on parameters in header, refer `TISCI <http://processors.wiki.ti.com/index.php/TISCI#Generic_Messaging_Header>`_.
*   Append payload to header.

Identify Secure Proxy threads
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

*   Select proxy thread for Tx and Rx based on the pHandle->map . Here, pHandle is a pointer of type ([Sciclient_ServiceHandle_t] * ) initialized by `Sciclient_init`_ .

Send Message
~~~~~~~~~~~~~~

*   Wait for binary semaphore pHandle->proxySem . Refer [Sciclient_ServiceHandle_t] for definition of  proxySem .
*   (pHandle->currSeqId ++)%[SCICLIENT_MAX_QUEUE_SIZE] . This is for differentiating different [Sciclient_service] calls.
*   seqId = pHandle->currSeqId .
*   Wait till queue has space till timeout (THREAD[a]_STATUS.curr_cnt > 0)
*   initialCount = Rx thread message count .
*   Write to Tx thread via secure proxy CSL-FL(CSL_secProxyAccessTarget) .
*   Release semaphore pHandle->proxySem.

Wait for response
~~~~~~~~~~~~~~~~~~

Sciclient waits for response when *flags* parameter in [Sciclient_ServiceInPrm_t] is TI_SCI_FLAG_REQ_ACK_ON_PROCESSED. Depending on the value of *opModeFlag* parameter in [Sciclient_ServiceHandle_t], there may be polling based or interrupt based execution . A global pointer *gSciclientHandle* = *pHandle* is also maintainded for ISR .

+------------------------------------------------------------+---------------------------------------------------------+
|**opModeFlag=0,Polling**                                    |   **pHandle->opModeFlag=1, Interrupt based**            |
+------------------------------------------------------------+---------------------------------------------------------+
|isMsgReceived = 0;                                          |    retVal=SemaphoreP_pend(pHandle->semSeqId[seqId],     |
|do  {                                                       |                           pInPrm->timeOut);             |
|Rx count = Rx thread message count                          |                                                         |
|if (Rx count > initialCount)                                |    **ISR**:                                             |
|{ Peek into Rx thread;                                      |    {Peek for message SeqId                              |
|  if(sequenceId received == sequenceId sent)                |    gSciclientHandle->respMsgArr[seqId] = Read Message   |
|  {     isMsgReceived=1;                                    |    SemaphoreP_post(gSciclientHandle->semSeqId[SeqId])   |
|        Read full message to pHandle->respMsgArr;           |    }                                                    |
|  }                                                         |                                                         |
| }  while(!isMsgReceived)                                   |                                                         |
+------------------------------------------------------------+---------------------------------------------------------+



Construct outPrms
~~~~~~~~~~~~~~~~~~~

*   Extract response header to construct **outPrm** structure
*   Copy payload from pHandle->respMsgArr[pHandle->currSeqId] to pOutPayload.



Resource Consumption
----------------------
Sciclient uses the following resources:

- A global pointer *gSciclientHandle* is allocated for ISR. Refer [Sciclient_ServiceHandle_t] .
- A structure for secure proxy base addreses *gSciclient_secProxyCfg* is defined.
- The linker command file for an application using the lib must allocate a section *boardcfg_data* in OC-MSRAM for the default board configuration data . 

 
.. include:: lld.rst

 
Design Analysis and Resolution(DAR)
====================================

DAR Criteria 1
----------------

SCIClient should support OS and non OS applications.
However, SCIClient in an OS context should be able to prevent mutliple threads from writing to the secure proxy.


Available Alternatives
~~~~~~~~~~~~~~~~~~~~~~~~

Alternative 1
^^^^^^^^^^^^^^^

Built the library separately for OS and non OS. Each API will have its own OS and Non-OS implementation.

Alternative 2
^^^^^^^^^^^^^^^

Use PDK OSAL and have the OSAL support for non-OS and OS implementation. The SCIClient APIs will have one implementation. The application should link OSAL as well.

Decision
~~~~~~~~~

Use Alternative 2 as the OSAL already has OS and non-OS implementation. The SCIClient need not duplicate code. Consistent with other libraries as well.


DAR Criteria 2
----------------

SCIClient constructing the SCI core message based on fields given as input by the higher level software.

Available Alternatives
~~~~~~~~~~~~~~~~~~~~~~~

Alternative 1
^^^^^^^^^^^^^^^

SCIClient maintains separate APIs for all the message types possible and the PM/RM/Security Libs pass parameters to this. Eg.
So if a SCI message looks like 
::

    struct msg_rm_alloc_foo {  
            struct message_hdr hdr;  
            s32 param_a;  
            u16 param_b;  
            u16 param_c;  
    } __attribute__((__packed__));  

The SCILIB API looks something like
::

    int32_t sci_client_rm_alloc_foo(int32_t param_a, uint16_t param_b, uint16_t param_c);

SCILIB abstracts the packed message format and the header details.


Alternative 2
^^^^^^^^^^^^^^^

SCI message structure is instead of 
::
    struct msg_rm_alloc_foo {  
            struct message_hdr hdr;  
            s32 param_a;  
            u16 param_b;  
            u16 param_c; 
    } __attribute__((__packed__));  

    int32_t sci_client_rm_alloc_foo(int32_t param_a, uint16_t param_b, uint16_t param_c);  

Do the following
::
    struct msg_rm_alloc_foo_body {  
            s32 param_a;  
            u16 param_b;  
            u16 param_c;  
    } __attribute__((__packed__));  

    int32_t sci_client_rm_alloc_foo(const struct msg_rm_alloc_foo_body * body);  

Just to extend that a bit further:  
::

    int32_t sci_client_rm_alloc_foo(const struct msg_rm_alloc_foo_body * body);  

becomes a common API
::
    int32_t sci_client_send_message(uint32 message_type, const struct void * body);  

Apart from message_type we have some more inputs which we finally capture in an inPrm structure.


Decision
~~~~~~~~~~

Use alternate 2 to not have multiple APIs in the SCIClient HAL and the SCIClient HAL will expose the core message structure for the higher level software to work on . Above this, SCIClient FL will have seperate API definitions for each type of supported message separately for PM, RM and Security .


 
Document revision history
============================

============  ==========    ===============  ===========================  ================
Version #      Date         Author Name      Revision History             Status  
============  ==========    ===============  ===========================  ================
 01.00        2-Jan-2018    SACHIN PUROHIT   Added design info for        Draft    
                                                     sciclient.h                        
============  ==========    ===============  ===========================  ================
