/**
 *   @file  rm_nameserver.c
 *
 *   @brief   
 *      Resource Manager NameServer source.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2015, Texas Instruments, Inc.
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
 *  \par
*/

/* RM external API includes */
#include <ti/drv/rm/rm.h>

/* RM internal API includes */
#include <ti/drv/rm/include/rm_internal.h>
#include <ti/drv/rm/include/rm_loc.h>
#include <ti/drv/rm/include/rm_nameserverloc.h>
#include <ti/drv/rm/include/rm_treeloc.h>

/* Tree algorithm includes */
#include <ti/drv/rm/util/tree.h>

/* RM OSAL layer */
#include <rm_osal.h>

/**********************************************************************
 ************************ Local Functions *****************************
 **********************************************************************/

/* FUNCTION PURPOSE: Returns a pointer to the NameServer tree
 ***********************************************************************
 * DESCRIPTION: Returns a pointer to the instance's NameServer tree
 *              based on the instance type
 */
static Rm_NameServerTree *nameServerGetTree(Rm_Handle rmHandle)
{
    Rm_Inst           *rmInst = (Rm_Inst *)rmHandle;
    Rm_NameServerTree *tree = NULL;

    if ((rmInst->instType == Rm_instType_SERVER) ||
        (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        tree = rmInst->u.server.nameServer;
    }
    return(tree);
}

/**********************************************************************
 ******************* Internal NameServer APIs *************************
 **********************************************************************/

/* FUNCTION PURPOSE: Adds an object to the NameServer
 ***********************************************************************
 * DESCRIPTION: Adds an object mapping a name to a resource to
 *              the NameServer tree.
 */
int32_t rmNameServerAddObject(Rm_NameServerObjCfg *objCfg)
{
    Rm_NameServerNode *newNode = NULL;
    int32_t            retVal = RM_SERVICE_APPROVED;

    if ((newNode = rmNameServerNodeNew(&objCfg->nodeCfg))) {
        if (RB_INSERT(_Rm_NameServerTree, objCfg->nameServerTree, newNode)) {
            /* Collision */
            rmNameServerNodeFree(newNode);
            retVal = RM_SERVICE_DENIED_NAME_EXISTS_IN_NS;
        }
    } else {
        retVal = RM_ERROR_NAMESERVER_NAME_ADD_FAILED;
    }

    return(retVal);
}

/* FUNCTION PURPOSE: Finds a NameServer object based on the name
 ***********************************************************************
 * DESCRIPTION: Finds the NameServer node with the name specified
 *              in the objCfg structure and returns the resource
 *              values mapped to the name via the objCfg structure
 */
int32_t rmNameServerFindObject(Rm_NameServerObjCfg *objCfg)
{
    Rm_NameServerNode  findNode;
    Rm_NameServerNode *matchingNode;
    int32_t            retVal = RM_ERROR_NAMESERVER_NAME_DOES_NOT_EXIST;

    memset((void *)&findNode, 0, sizeof(Rm_NameServerNode));
    rm_strncpy(findNode.objName, objCfg->nodeCfg.objName, RM_NAME_MAX_CHARS);
    
    if ((matchingNode = RB_FIND(_Rm_NameServerTree, objCfg->nameServerTree,
                                &findNode))) {
        /* Found in NameServer */
        objCfg->nodeCfg.resourceName = matchingNode->resourceName;
        objCfg->nodeCfg.resourceBase = matchingNode->resourceBase;
        objCfg->nodeCfg.resourceLength = matchingNode->resourceLength;

        retVal = RM_SERVICE_PROCESSING;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Deletes an object from the NameServer
 ***********************************************************************
 * DESCRIPTION: Deletes the object with the name defined in the
 *              objCfg structure from the NameServer tree.  Frees the
 *              memory associated with the NameServer node
 */
int32_t rmNameServerDeleteObject(Rm_NameServerObjCfg *objCfg)
{
    Rm_NameServerNode  findNode;
    Rm_NameServerNode *matchingNode;
    int32_t            retVal = RM_ERROR_NAMESERVER_NAME_DOES_NOT_EXIST;

    memset((void *)&findNode, 0, sizeof(Rm_NameServerNode));
    rm_strncpy(findNode.objName, objCfg->nodeCfg.objName, RM_NAME_MAX_CHARS);

    if ((matchingNode = RB_FIND(_Rm_NameServerTree, objCfg->nameServerTree,
                                &findNode))) {
        /* Remove from NameServer */
        RB_REMOVE(_Rm_NameServerTree, objCfg->nameServerTree, matchingNode);
        rmNameServerNodeFree(matchingNode);

        retVal = RM_SERVICE_APPROVED;
    }
    return(retVal);
}

/* FUNCTION PURPOSE: Prints the NameServer objects
 ***********************************************************************
 * DESCRIPTION: Prints the names and the resources they're mapped
 *              to for all objects stored in the NameServer tree.
 */
void rmNameServerPrintObjects(Rm_Handle rmHandle)
{
    Rm_Inst           *rmInst = (Rm_Inst *)rmHandle;
    Rm_NameServerTree *root = rmInst->u.server.nameServer;
    Rm_NameServerNode *node;

    if (rmInst->instType == Rm_instType_SHARED_SERVER) {
        rmNameServerTreeInv(root);
    }
    RB_FOREACH(node, _Rm_NameServerTree, root) {
        Rm_osalLog("Name: %s resourceName: %s base: %d length: %d\n",
                   node->objName, node->resourceName, node->resourceBase,
                   node->resourceLength);
    }
}

/* FUNCTION PURPOSE: Initializes the NameServer tree
 ***********************************************************************
 * DESCRIPTION: Creates and initializes the NameServer tree
 *              root entry.
 */
void rmNameServerInit(Rm_Handle rmHandle)
{
    Rm_Inst           *rmInst = (Rm_Inst *)rmHandle;
    Rm_NameServerTree *rootEntry = NULL;

    /* NameServer only exists in Server instances for now */
    if ((rmInst->instType == Rm_instType_SERVER) ||
        (rmInst->instType == Rm_instType_SHARED_SERVER)) {
        rootEntry = Rm_osalMalloc(sizeof(Rm_NameServerTree));
        RB_INIT(rootEntry);
        RM_SS_OBJ_WB(rmInst, rootEntry, Rm_NameServerTree);
        rmInst->u.server.nameServer = rootEntry;
    }
}

/* FUNCTION PURPOSE: Deletes the NameServer tree
 ***********************************************************************
 * DESCRIPTION: Removes all objects from the NameServer tree
 *              and deletes the NameServer tree root node.
 */
void rmNameServerDelete(Rm_Handle rmHandle)
{
    Rm_Inst             *rmInst = (Rm_Inst *)rmHandle;
    Rm_NameServerTree   *treeRoot = nameServerGetTree(rmHandle);
    Rm_NameServerNode   *node;
    Rm_NameServerNode   *nextNode;
    Rm_NameServerObjCfg  objCfg;

    if (treeRoot) {
        if (rmInst->instType == Rm_instType_SHARED_SERVER) {
            rmNameServerTreeInv(treeRoot);
        }

        for (node = RB_MIN(_Rm_NameServerTree, treeRoot);
             node != NULL;
             node = nextNode) {
            nextNode = RB_NEXT(_Rm_NameServerTree, treeRoot, node);
            objCfg.nameServerTree = treeRoot;
            objCfg.nodeCfg.objName = node->objName;
            rmNameServerDeleteObject(&objCfg);
        }

        /* Don't need to writeback tree node changes since NameServer will be
         * made NULL in instance */

        if (RB_MIN(_Rm_NameServerTree, treeRoot) == NULL) {
            Rm_osalFree((void *)treeRoot, sizeof(Rm_NameServerTree));
        }
        rmInst->u.server.nameServer = NULL;
    }
}

