/******************************************************************************
 *
 * Copyright (C) 2012-2019 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 *
 * dp_topology_utils.c
 *
 ******************************************************************************
 */

/*
 * Copyright (C) 2014 Red Hat.
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <stdlib.h>
#include "dp_aux.h"
#include "dp_topology_utils.h"
#include "cdn_log.h"

/*
 * Clean branch parameters
 * @param[in] mstBranch pointer to branch object to be cleaned
 */
static void cleanBranch(MST_drm_dp_branch* mstBranch)
{
    uint8_t i;
    mstBranch->refcount = 0U;

    /* clear relative address of branch */
    for (i = 0U; i < 8U; i++) {
        mstBranch->rad[i] = 0U;
    }

    mstBranch->lct = 0U;
    mstBranch->num_ports = 0U;
    mstBranch->msg_slots = 0;

    /* clean ports */
    for (i = 0U; i < MST_BRANCH_MAX_PORTS; i++) {
        mstBranch->ports[i] = (MST_drm_dp_port){0};
    }

    mstBranch->port_parent = NULL;
    mstBranch->mgr = NULL;
    mstBranch->tx_slots[0] = NULL;
    mstBranch->tx_slots[1] = NULL;
    mstBranch->last_seqno = 0;
    mstBranch->link_address_sent = false;

    /* clean global unique identifier */
    for (i = 0U; i < 16U; i++) {
        mstBranch->guid[i] = 0U;
    }
}

/*
 * Set 'occupied' flag and return reference for branch
 * @param[in] topologyManager pointer to topology manager object
 * @return reference to branch if success
 * @return NULL if not enough space for new branch
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter 'topologyManager' as const, DRV-3885' */
static MST_drm_dp_branch* allocateBranch(MST_drm_dp_topology_mgr* topologyManager)
{
    uint8_t i;
    MST_drm_dp_branch* branch = NULL;
    MST_drm_dp_branch* retBranch = NULL;

    /* looks for first empty slot */
    for (i = 0U; i < MST_MAX_BRANCH_NUMBER; i++) {
        branch = &(topologyManager->branch_list[i]);
        if (branch->isOccupied == false) {
            branch->isOccupied = true;
            retBranch = branch;
            break;
        }
    }

    /* clean branch */
    if (retBranch != NULL) {
        cleanBranch(retBranch);
    }

    return retBranch;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

/*
 * Unset ocuppied flag for branch memory space
 * @param[in] topologyManager pointer to topology manager object
 * @param[in] mstBranch pointer to deallocated branch
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter 'topologyManager' as const, DRV-3885' */
static void deallocateBranch(MST_drm_dp_topology_mgr* topologyManager,
                             const MST_drm_dp_branch* mstBranch)
{
    uint8_t i;
    MST_drm_dp_branch* branch = NULL;

    for (i = 0U; i < MST_MAX_BRANCH_NUMBER; i++) {
        branch = &(topologyManager->branch_list[i]);
        /* check if branches are the same */
        if (branch == mstBranch) {
            /* reset 'isOcuppied' flag */
            branch->isOccupied = false;
            break;
        }
    }
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

MST_drm_dp_branch *DRM_DP_addBranchDevice(MST_drm_dp_topology_mgr* topologyManager,
                                          uint8_t                  linkCount,
                                          uint8_t *                relativeAddress)
{
    MST_drm_dp_branch *mstBranch;

    /* get address of new branch */
    mstBranch = allocateBranch(topologyManager);

    if (mstBranch != NULL) {

        mstBranch->lct = linkCount;

        /* copy realtive address to branch */
        if (linkCount > 1U) {
            uint8_t size = linkCount / 2U;
            (void)memcpy(mstBranch->rad, relativeAddress, size);
        }

        mstBranch->refcount = 1U;
    }
    return mstBranch;
}

/*
 * Compare two branch references
 * @param[in] mstBranch pointer to first branch
 * @param[in] branchToFind pointer to second branch
 * @return true if equal
 * @return false if not equal
 */
static bool checkBranch(MST_drm_dp_branch*       mstBranch,
                        const MST_drm_dp_branch *branchToFind)
{
    bool addressEqual = false;

    if (branchToFind == mstBranch) {
        mstBranch->refcount++;
        addressEqual = true;
    }

    return addressEqual;
}

/*
 * Looking for branch in device tree
 * @param[in] mstBranch pointer to MST branch object
 * @param[in] branchToFind pojnter to MST branch object which is searched
 * @return reference to searched branch if success
 * @return NULL if branch not found
 */
static MST_drm_dp_branch *getValidatedBranchRef(MST_drm_dp_branch*       mstBranch,
                                                const MST_drm_dp_branch *branchToFind)
{
    MST_drm_dp_branch* actualBranch = mstBranch;
    MST_drm_dp_branch* retBranch = NULL;
    uint8_t analyzedPortNumber = 0U;
    uint8_t stack[MST_MAX_BRANCH_DEEP_LEVEL] = {0U};
    uint8_t stackPointer = 0U;
    MST_drm_dp_port* port;

    bool foundBranch = checkBranch(mstBranch, branchToFind);

    if (foundBranch) {
        retBranch = mstBranch;
    }

    /* check if branch was founded */
    while (retBranch == NULL) {

        /* analyze each port for branch */
        while (analyzedPortNumber < MST_BRANCH_MAX_PORTS) {
            port  = &actualBranch->ports[analyzedPortNumber];
            analyzedPortNumber++;

            if ((port->mstb != NULL) && (port->refcount > 0U)) {

                foundBranch = checkBranch(port->mstb, branchToFind);

                /* if actualBranch was not found, remember port number */
                /* and check next actualBranch */
                if (!foundBranch) {
                    stack[stackPointer] = analyzedPortNumber;
                    stackPointer++;
                    actualBranch = port->mstb;
                    analyzedPortNumber = 0U;
                    continue;
                } else {
                    retBranch = port->mstb;
                    break;
                }
            }
        }

        /* break searching when no branches to check */
        if ((stackPointer > 0U)) {
            stackPointer--;
            analyzedPortNumber = stack[stackPointer];
            actualBranch = actualBranch->port_parent->parent;
        } else {
            break;
        }

    }

    return retBranch;

}

MST_drm_dp_branch* DRM_DP_getValidatedBranchRef(MST_drm_dp_topology_mgr* topologyManager,
                                                const MST_drm_dp_branch* mstBranch)
{
    MST_drm_dp_branch* retBranch = NULL;

    /* test if topology has branch */
    if (topologyManager->mst_primary != NULL) {
        retBranch = getValidatedBranchRef(topologyManager->mst_primary, mstBranch);
    }

    return retBranch;
}

/* Check if ports are the same and have any references to itself
 * @param[in] port pointer to first port
 * @param[in] portToFind pointer to second port
 * @return true if same
 * @return false if not same
 */
static bool checkPort( MST_drm_dp_port*       port,
                       const MST_drm_dp_port* portToFind)
{
    bool addressEqual = false;

    if ((port == portToFind) && (port->refcount > 0U)) {
        port->refcount++;
        addressEqual = true;
    }

    return addressEqual;
}

/*
 * Returns reference to searched port
 * @param[in] mstBranch pointer to analyzed MST branch
 * @param[in] portToFind pointer to searched port
 * @return pointer to searched port if success
 * @return NULL if cannot find port
 */
static MST_drm_dp_port *getPortRef(MST_drm_dp_branch*     mstBranch,
                                   const MST_drm_dp_port* portToFind)
{
    MST_drm_dp_branch* actualBranch = mstBranch;
    MST_drm_dp_port* retPort = NULL;
    uint8_t actualPortNumber = 0U;
    uint8_t stack[MST_MAX_BRANCH_DEEP_LEVEL] = {0U};
    uint8_t stackPointer = 0U;
    MST_drm_dp_port* port;

    bool check = false;

    while (retPort == NULL) {

        /* analyze for each port in branch */
        while (actualPortNumber < MST_BRANCH_MAX_PORTS) {
            port  = &actualBranch->ports[actualPortNumber];
            actualPortNumber++;

            /* check if ports are the same */
            check = checkPort(port, portToFind);

            if (!check) {
                /* put into stack when ports not the same but port have */
                /* branch and no reference */
                if ((port->mstb != NULL) && (port->refcount > 0U)) {
                    stack[stackPointer] = actualPortNumber;
                    stackPointer++;
                    actualBranch = port->mstb;
                    actualPortNumber = 0U;
                    continue;
                }
            } else {
                retPort = port;
                break;
            }
        }

        /*  break when any ports to check */
        if (stackPointer > 0U) {
            stackPointer--;
            actualPortNumber = stack[stackPointer];
            actualBranch = actualBranch->port_parent->parent;
        } else {
            break;
        }

    }

    return retPort;
}

MST_drm_dp_port *DRM_DP_getValidatedPortRef(MST_drm_dp_topology_mgr *topologyManager,
                                            const MST_drm_dp_port*   port)
{
    MST_drm_dp_port *retPort = NULL;

    /* test if topology have any (main) branch */
    if (topologyManager->mst_primary != NULL) {
        retPort = getPortRef(topologyManager->mst_primary, port);
    }

    return retPort;
}

/*
 * Check if reference count for each port in branch is 0
 * @patam[in] branch pointer to branch object
 * @return true if branch have no references to ports
 * @return false if not
 */
static bool checkIfBranchEmpty(const MST_drm_dp_branch* branch)
{
    uint8_t i;
    bool retVal = true;

    for (i = 0U; i < MST_BRANCH_MAX_PORTS; i++) {
        if (branch->ports[i].refcount > 0U) {
            retVal = false;
            break;
        }
    }
    return retVal;
}

/*
 * Check if branch has reference to parent
 * @param[in] branch pointer to branch object
 * @return true if parent exists
 * @return false if not exists
 */
static bool parentChecker(const MST_drm_dp_branch* branch)
{
    bool hasParent = false;

    if (branch->port_parent != NULL) {
        if (branch->port_parent->parent != NULL) {
            hasParent = true;
        }
    }

    return hasParent;
}

/*
 * Check if branch's parent exists and has empty ports
 * @param[in] branch pointer to branch object
 * @return true if success
 * @return false if parent have references
 */
static bool checkIfParentEmpty(const MST_drm_dp_branch* branch)
{
    bool hasParent = false;
    bool isEmpty = false;

    hasParent = parentChecker(branch);

    if (hasParent) {
        isEmpty = checkIfBranchEmpty(branch->port_parent->parent);
    }

    return isEmpty;
}

/*
 * Free port
 * @param[in] port pointer to port object
 * @return NULL if success
 * @return pointer to next branch which should be deallocated
 */
static MST_drm_dp_branch* freePort(MST_drm_dp_port*port);

/*
 * Deallocate memory space for branch
 * @param[in] mstBranch pointer to branch
 */
static void removeBranch(MST_drm_dp_branch* mstBranch)
{
    MST_drm_dp_branch* branchStack[MST_MAX_BRANCH_DEEP_LEVEL] = {NULL};
    uint8_t stackPointer = 0U;

    MST_drm_dp_port* portParent = NULL;
    MST_drm_dp_branch* branchParent = NULL;
    MST_drm_dp_branch* branch = mstBranch;
    bool isEmpty;

    branchStack[0] = mstBranch;
    stackPointer++;

    while (branch != NULL) {
        isEmpty = checkIfParentEmpty(branch);

        if (isEmpty) {
            portParent = branch->port_parent;
            portParent->refcount--;

            /* free ports of branch when reference count is 0 */
            if (portParent->refcount == 0U) {
                branchParent = freePort(portParent);

                /* if branch parent should be deallocated, push branch into stack and */
                /* analyze branch parent */
                if (branchParent != NULL) {
                    branchStack[stackPointer] = branch;
                    branch = branchParent;
                    stackPointer++;
                    continue;
                }
            }
        }
        break;
    }

    /* deallocate branches from stack */
    while (stackPointer > 0U) {
        stackPointer--;
        branch = branchStack[stackPointer];
        deallocateBranch(branch->mgr, branch);
    }

}

/*
 * Assign 0 to portreference counter
 * @param[in] port pointer to port object
 * @return reference to branch if branch should be removed
 * @return NULL if branch should be not removed
 */
static MST_drm_dp_branch*  freePort(MST_drm_dp_port *port) {

    MST_drm_dp_branch* mstBranch = port->parent;
    MST_drm_dp_branch* retBranch = NULL;

    mstBranch->refcount--;
    if (mstBranch->refcount == 0U) {
        retBranch = mstBranch;
    }

    /* clean reference count for port */
    port->refcount = 0U;

    return retBranch;
}

/*
 * Drop messages from sideband slots for branch
 * @param[in] mstBranch pointer to MST branch object
 */
static void dropSidebandTxSlotsMessages(MST_drm_dp_branch *mstBranch)
{
    /* test if first slot exists */
    if (mstBranch->tx_slots[0] != NULL) {
        mstBranch->tx_slots[0]->state = DRM_DP_SIDEBAND_TX_TIMEOUT;
        mstBranch->tx_slots[0] = NULL;
    }

    /* test if second slot exists */
    if (mstBranch->tx_slots[1] != NULL) {
        mstBranch->tx_slots[1]->state = DRM_DP_SIDEBAND_TX_TIMEOUT;
        mstBranch->tx_slots[1] = NULL;
    }
}

/*
 * Free (deallocate) memory for branch
 * @param[in] mstBranch pointer to branch object
 */
static void freeBranch(MST_drm_dp_branch *mstBranch)
{
    dropSidebandTxSlotsMessages(mstBranch);

    mstBranch->refcount--;

    if (mstBranch->refcount == 0U) {
        removeBranch(mstBranch);
    }

}

/*
 * Decrease reference counter for branch
 * @param[in] pointer to branch object
 * @return pointer to branch if should be deallocated
 * @return NULL if branch should not be deallocated
 */
static MST_drm_dp_branch* putBranch(MST_drm_dp_branch* branch)
{
    MST_drm_dp_branch* retBranch = NULL;

    branch->refcount--;
    /* test if branch have any reference */
    if (branch->refcount  == 0U) {
        branch->refcount = 1U;
        retBranch = branch;
    }

    return retBranch;
}

/*
 * Clean reference to device if branch
 * @param[in] port pointer to port object
 * @param[in] oldPdt peer device type
 * @return pointer to branch if should be deallocated
 * @return NULL if branch should not be deallocated
 */
static MST_drm_dp_branch* teardownPdt(MST_drm_dp_port *port, uint8_t oldPdt)
{
    MST_drm_dp_branch* retBranch = NULL;

    if (oldPdt == DP_PEER_DEVICE_MST_BRANCHING) {
        retBranch = putBranch(port->mstb);
        port->mstb = NULL;
    }

    return retBranch;
}

/*
 * Destroy port if not input
 * @param[in] port pointer to port object
 * @return pointer to branch if should be deallocated
 * @return NULL if branch should not be deallocated
 */
static MST_drm_dp_branch* destroyPort(MST_drm_dp_port *port)
{
    MST_drm_dp_branch* retBranch = NULL;

    /* test if port is output */
    if (!port->input) {
        port->vcpi.num_slots = 0;
        retBranch = teardownPdt(port, port->pdt);
        port->refcount = 0U;
    }

    return retBranch;
}

/*
 * Decrease reference count for port
 * @param[in] port pointer to port object
 * @return pointer to branch if should be deallocated
 * @return NULL if branch should not be deallocated
 */
static MST_drm_dp_branch* putPort(MST_drm_dp_port *port)
{
    MST_drm_dp_branch* retBranch = NULL;

    port->refcount--;
    if (port->refcount == 0U) {
        retBranch = destroyPort(port);
    }

    return retBranch;
}

/*
 * Destroy branch
 * @param[in] mstb pointer to branch object
 */
/* parasoft-begin-suppress METRICS-36-3 "Function is called from more than 5 functions, DRV-3823" */
static void destroyBranch(MST_drm_dp_branch* mstb)
{
    uint8_t stackPointer = 0U;
    MST_drm_dp_branch* branch = mstb;
    MST_drm_dp_branch* childBranch = mstb;
    MST_drm_dp_branch* branchParent = NULL;
    uint8_t i = 0U;
    uint8_t stack[16U] = {0U};

    while (true) {

        while (i < MST_BRANCH_MAX_PORTS) {
            childBranch = putPort(&(branch->ports[i]));
            i++;

            /* if branch have child, push into stack and analyze child */
            if (childBranch != NULL) {
                stack[stackPointer] = i;
                stackPointer++;
                branch = childBranch;
                i = 0U;
                continue;
            }
        }

        /* mstb is top of device tree */
        if (branch != mstb) {
            branchParent = branch->port_parent->parent;
        }

        freeBranch(branch);

        /* break if any branch to check */
        if (stackPointer > 0U) {
            stackPointer--;
            i = stack[stackPointer];
            branch = branchParent;

        } else {
            break;
        }

    }
}
/* parasoft-end-suppress METRICS-36-3 */

void DRM_DP_putBranchDevice(MST_drm_dp_branch *mstBranch)
{
    MST_drm_dp_branch* branch = NULL;

    if (mstBranch != NULL) {
        branch = putBranch(mstBranch);
    }

    if (branch != NULL) {
        destroyBranch(branch);
    }
}

void DRM_DP_teardownPort(MST_drm_dp_port *port, uint8_t oldPeerDeviceType)
{
    MST_drm_dp_branch* branch = NULL;

    /* check if port exists */
    if (port != NULL) {
        branch = teardownPdt(port, oldPeerDeviceType);
    }

    /* destroy branch */
    if (branch != NULL) {
        destroyBranch(branch);
    }
}

void DRM_DP_putPort(MST_drm_dp_port *port)
{
    MST_drm_dp_branch* branch = NULL;

    /* here we analyze main port, if we need destroy branch, do it again for */
    /* each port in destroyed branch */
    if (port != NULL) {
        branch = putPort(port);
    }

    if (branch != NULL) {
        destroyBranch(branch);
    }

}

/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter 'topologyManager' as const, DRV-3885" */
MST_drm_dp_branch *DRM_DP_getBranchDevice(MST_drm_dp_topology_mgr *topologyManager,
                                          uint8_t                  linkCount,
                                          const uint8_t *          relativeAddress)
{
    const uint8_t PortNumberMask = 0x0FU;
    MST_drm_dp_branch *mstb = topologyManager->mst_primary;
    MST_drm_dp_port port;
    uint8_t i;
    uint8_t j;
    uint8_t shift;
    uint8_t portNumber;

    for (i = 0U; i < (linkCount - 1U); i++) {

        if (mstb == NULL) {
            DbgMsg(DBG_GEN_MSG, DBG_WARN,
                   "WARNING: Failed to lookup MSTB with linkCount %d, relativeAddress %02x\n",
                   linkCount, relativeAddress[0]);
            break;
        }

        shift = ((i % 2U) != 0U) ? 0U : 4U;

        /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 "Shifting operation should be checked, DRV-3822" */
        portNumber = (relativeAddress[i / 2U] >> shift) & PortNumberMask;
        /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 */

        for (j = 0U; j < MST_BRANCH_MAX_PORTS; j++) {
            port = mstb->ports[j];
            if ((port.refcount > 0U) && (port.port_num == portNumber)) {
                mstb = port.mstb;
                break;
            }
        }
    }

    /* increase reference count */
    if (mstb != NULL) {
        mstb->refcount++;
    }

    return mstb;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter 'mstBranch as const, DRV-3885" */
MST_drm_dp_port* DRM_DP_getPort(MST_drm_dp_branch *mstBranch,
                                uint8_t            portNumber) {
    uint8_t i;
    MST_drm_dp_port* currPort;
    MST_drm_dp_port* retPort = NULL;

    for (i = 0U; i < MST_BRANCH_MAX_PORTS; i++) {
        currPort = &(mstBranch->ports[i]);
        /* check if port have references and number of port equals */
        if ((currPort->refcount > 0U) && (currPort->port_num == portNumber)) {
            currPort->refcount++;
            retPort = &(mstBranch->ports[i]);
            break;
        }
    }

    return retPort;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

/*
 * Looking for device using GUID
 * @param[in] mstBranch pointer to branch object
 * @param[in] guid pointer to global unique identifier
 */
static MST_drm_dp_branch *getBranchDeviceByGuidHelper(MST_drm_dp_branch *mstBranch,
                                                      uint8_t *          guid)
{
    MST_drm_dp_branch *foundBranch = NULL;
    MST_drm_dp_branch *actualBranch = mstBranch;
    MST_drm_dp_port* actualPort = NULL;
    uint8_t stack[MST_MAX_BRANCH_DEEP_LEVEL] = {0U};
    uint8_t stackPointer = 0U;
    uint8_t actualPortNumber = 0U;

    uint8_t memoryEqual = (uint8_t)(memcmp(mstBranch->guid, guid, 16));

    while (memoryEqual != 0U) {

        while (actualPortNumber < MST_BRANCH_MAX_PORTS) {

            actualPort = &(actualBranch->ports[actualPortNumber]);
            actualPortNumber++;

            /* test if port have references */
            if (actualPort->refcount > 0U) {

                /* test if port have connected branch */
                if (actualPort->mstb == NULL) {
                    continue;
                }

                /* check if guid equals */
                memoryEqual = (uint8_t)(memcmp(actualPort->mstb->guid, guid, 16));

                if (memoryEqual == 0U) {
                    foundBranch = actualPort->mstb;
                    break;
                }

                /* push port number into stack an analyze next branch */
                actualBranch = actualPort->mstb;
                stack[stackPointer] = actualPortNumber;
                stackPointer++;
                actualPortNumber = 0U;
            }
        }

        /* if any port to check, pop it from stack */
        if (stackPointer > 0U) {
            stackPointer--;
            actualBranch = actualBranch->port_parent->parent;
            actualPortNumber = stack[stackPointer];
        } else {
            break;
        }
    }

    return foundBranch;
}

MST_drm_dp_branch *DRM_DP_getBranchDeviceByGuid(MST_drm_dp_topology_mgr *topologyManager,
                                                uint8_t *                guid)
{
    MST_drm_dp_branch *mstb;

    mstb = getBranchDeviceByGuidHelper(topologyManager->mst_primary, guid);

    /* increase reference counter for branch if not NUL */
    if (mstb != NULL) {
        mstb->refcount++;
    }

    return mstb;
}
