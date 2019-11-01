=======================
J721E Host Descriptions
=======================

.. _soc_doc_j721e_public_host_desc_intro:

Introduction
============

This chapter provides information of Host IDs that are permitted in
the J721E SoC. These host IDs represent processing entities (or PEs)
which is mandatory identification of a Host in a processor.

Typically a host is a 'compute entity' which may be an actual
processor or even a virtual machine. We just use host or processing
entity to indicate the same thing.

.. _soc_doc_j721e_public_host_desc_host_list:

Enumeration of Host IDs
=======================

+-----------+-------------+-------------------+-----------------------------------------+
|   Host ID | Host Name   | Security Status   | Description                             |
+===========+=============+===================+=========================================+
|         0 | DMSC        | Secure            | Device Management and Security Control  |
+-----------+-------------+-------------------+-----------------------------------------+
|         3 | MCU_0_R5_0  | Non Secure        | Cortex R5 context 0 on MCU island       |
+-----------+-------------+-------------------+-----------------------------------------+
|         4 | MCU_0_R5_1  | Secure            | Cortex R5 context 1 on MCU island(Boot) |
+-----------+-------------+-------------------+-----------------------------------------+
|         5 | MCU_0_R5_2  | Non Secure        | Cortex R5 context 2 on MCU island       |
+-----------+-------------+-------------------+-----------------------------------------+
|         6 | MCU_0_R5_3  | Secure            | Cortex R5 context 3 on MCU island       |
+-----------+-------------+-------------------+-----------------------------------------+
|        10 | A72_0       | Secure            | Cortex A72 context 0 on Main island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        11 | A72_1       | Secure            | Cortex A72 context 1 on Main island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        12 | A72_2       | Non Secure        | Cortex A72 context 2 on Main island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        13 | A72_3       | Non Secure        | Cortex A72 context 3 on Main island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        14 | A72_4       | Non Secure        | Cortex A72 context 4 on Main island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        20 | C7X_0       | Secure            | C7x Context 0 on Main island            |
+-----------+-------------+-------------------+-----------------------------------------+
|        21 | C7X_1       | Non Secure        | C7x context 1 on Main island            |
+-----------+-------------+-------------------+-----------------------------------------+
|        25 | C6X_0_0     | Secure            | C6x_0 Context 0 on Main island          |
+-----------+-------------+-------------------+-----------------------------------------+
|        26 | C6X_0_1     | Non Secure        | C6x_0 context 1 on Main island          |
+-----------+-------------+-------------------+-----------------------------------------+
|        27 | C6X_1_0     | Secure            | C6x_1 Context 0 on Main island          |
+-----------+-------------+-------------------+-----------------------------------------+
|        28 | C6X_1_1     | Non Secure        | C6x_1 context 1 on Main island          |
+-----------+-------------+-------------------+-----------------------------------------+
|        30 | GPU_0       | Non Secure        | RGX context 0 on Main island            |
+-----------+-------------+-------------------+-----------------------------------------+
|        35 | MAIN_0_R5_0 | Non Secure        | Cortex R5_0 context 0 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        36 | MAIN_0_R5_1 | Secure            | Cortex R5_0 context 1 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        37 | MAIN_0_R5_2 | Non Secure        | Cortex R5_0 context 2 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        38 | MAIN_0_R5_3 | Secure            | Cortex R5_0 context 3 on MCU island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        40 | MAIN_1_R5_0 | Non Secure        | Cortex R5_1 context 0 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        41 | MAIN_1_R5_1 | Secure            | Cortex R5_1 context 1 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        42 | MAIN_1_R5_2 | Non Secure        | Cortex R5_1 context 2 on Main island    |
+-----------+-------------+-------------------+-----------------------------------------+
|        43 | MAIN_1_R5_3 | Secure            | Cortex R5_1 context 3 on MCU island     |
+-----------+-------------+-------------------+-----------------------------------------+
|        50 | ICSSG_0     | Non Secure        | ICSSG context 0 on Main island          |
+-----------+-------------+-------------------+-----------------------------------------+

.. note::

   * Description provides an intended purpose
     of the host ID, though on some systems,
     this might be used differently, backing memory and
     link allocations are made with the specified purpose
     in mind
   * Security Status provides an intended purpose for the
     Host context
