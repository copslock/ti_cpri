/**
 *   @file  rm.h
 *
 *   @brief   
 *      This is the Resource Manager include file.
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

#ifndef RM_H_
#define RM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <stdint.h>

/**  @mainpage Resource Manager
 *
 *   @section intro  Introduction
 *
 *   The Resource Manager (RM) is designed to provide an easy to use,
 *   extensible uinified resource management solution on TI devices.  RM can be
 *   integrated by a system application to provide resource management services
 *   to the system temporally (pre-main/post-main) and spatially (system
 *   subcomponents, task, cores).
 *
 *   The RM architecture is instance based.  All RM resource management
 *   services must originate from a RM instance.  Resource permissions are
 *   assigned by a system integrator using RM instance names provided during the
 *   initialization of each RM instance in use.  There are three types of RM
 *   instances, all of which can be used by the system to request resource
 *   management services:
 *    - Server - Manages all resource data structures including the resource
 *               allocators, the permissions policies, and a simple NameServer.
 *               There can only be one Server per RM ecosystem.  If two Servers
 *               are initialized within a system they must manage mutually
 *               exclusive sets of device resources.  There is no limit to the
 *               number of Client and Client Delegate instances that can connect
 *               to the Server
 *    - Client - Used by system components to request resource services.  There
 *               is no limit to the number of Clients a system can have.  Client
 *               names must be unique since resource permissions are assigned
 *               based on RM instance names.  Clients can connect to at most
 *               one Server or Client Delegate, but not both at the same time.
 *    - Client Delegate (CD) - At the moment the CD is not different from the
 *                             Client.  However, in the future the CD will be
 *                             able to act as a proxy for the Server.  The CD
 *                             will be able to satisfy some service requests
 *                             from Clients, offloading some processing from
 *                             the Server.  This feature will be helpful in
 *                             situations where a slower data path exists
 *                             between the Server and CD/Client instances.
 *                             There is no limit to the number of Clients that
 *                             can connect to a CD.  The CD can connect to at
 *                             most one Server.
 *
 *   RM instances communicate via a generic transport interface.  The RM
 *   transport interface expects the application to configure and manage the
 *   transport data paths between RM instances.  This allows RM to easily
 *   extend to different device configurations and different devices entirely.
 *
 *   Shared memory versions of the Server and Client are available for
 *   configuration in cases where the DSP applications cannot tolerate blocking
 *   operations or long wait times for resources.  The Shared Server - Shared
 *   Client model assumes all memory allocated via the OSAL layer is within
 *   shared memory.  RM service requests received from Shared Servers and
 *   Shared Clients will be handled via accesses to the resource management
 *   data structures existing in shared memory.
 *   - Shared Server - Essentially a Server instance that expects to be
 *                     allocated from shared memory via the application-supplied
 *                     OSAL functions.  Shared Client instances will piggyback
 *                     on the Shared Server instance to allocate/free resources
 *                     without the need to setup transports between the
 *                     instances.  Access to the resource management data
 *                     structures is managed through OSAL implemented cache
 *                     writeback and invalidate operations.
 *   - Shared Client - Must be provided a Shared Server handle at initialization
 *                     time.  The Shared Client will essentially use the
 *                     resource management data structures, created in shared
 *                     memory when the Shared Server was initialized, to handle
 *                     any server requests.
 *
 *   RM utilizes the BDS-licensed, open source, Flattened Device Tree format to
 *   specify what resources are managed by RM as well as the RM instance
 *   permissions for managed resources.  The Global Resource List or GRL defines
 *   all device resources and their ranges that will be tracked by the
 *   RM Server.  Addition or subtraction of resources from RM requires one
 *   modify only the GRL.  RM source code changes are not required to add or
 *   subtract resources from RM's umbrella of management.  RM Policies specify
 *   resource permissions for the RM instances.  There are two types of
 *   Policies:
 *    - Global Policy - Provided to the Server at initialization and defines
 *                      the resource permissions for all RM instances in the
 *                      system.  All service requests will be validated against
 *                      the Global Policy on the Server.  If the RM instance is
 *                      found to not hold the privileges for the request a
 *                      denial of the service will be issued back to the
 *                      requesting instance.
 *    - Static Policy - Optionally provided to Client and CD instances at
 *                      initialization.  Allows these instances to statically
 *                      allocate resources.  This feature is typically used
 *                      for RM instances that must allocate resources prior
 *                      to the transport connection to the Server being
 *                      established.  Resources allocated via any Static
 *                      Policies will be validated against the Global Policy
 *                      once the transport 
 *                      to the Server has been fully established.  If a Static
 *                      Policy request fails validation with the Global Policy
 *                      the RM instance that issued the static request will be
 *                      placed into a locked state.  The locked state prevents
 *                      any further service requests from the instance.
 *
 *   Combined, the GRL and Policy Device Tree implementations allow RM to easily
 *   extend to new resources without the need to recompile the RM source code.
 *
 *   RM instances currently provides the following resource services:
 *    - Allocate (initialize) - Allocate a resource for initialization
 *    - Allocate (usage)      - Allocate a resource for use
 *    - Status                - Return the reference count for a specified
 *                              resource
 *    - Free                  - Free an allocated resource (The free must
 *                              originate from the RM instance that allocated
 *                              the resource
 *    - Map resource to name  - Map a specified resource to a NameServer name
 *    - Unmap named resource  - Unmap a resource from an existing NameServer name
 *    - Get resource by name  - Returns a resource based on a provided NameServer
 *                              name
 */

/* Define RM_API as a master group in Doxygen format and add all RM API 
   definitions to this group. */
/** @defgroup RM_API Resource Manager API
 *  @{
 */
/** @} */

/**
@defgroup RM_TRANSPORT_API  RM Transport API
@ingroup RM_API
*/
/**
@defgroup RM_SERVICES_API  RM Services API
@ingroup RM_API
*/
/**
@defgroup RM_OSAL_API RM OS Abstraction Layer API
@ingroup RM_API
*/

/**
@addtogroup RM_API
@{
*/

/* RM return and error codes */
/** RM successful return code */
#define RM_OK                                      0
/** RM processing requested service */
#define RM_SERVICE_PROCESSING                      1
/** RM CD has placed on the request on hold pending a Server response */
#define RM_SERVICE_PENDING_SERVER_RESPONSE         2
/** RM has approved requested service */
#define RM_SERVICE_APPROVED                        3
/** RM has approved requested service based on static policy.  Request will be
 *  validated against global policy once all transports have been registered */
#define RM_SERVICE_APPROVED_STATIC                 4

/** RM service request denial reasons base */
#define RM_SERVICE_DENIED_BASE                     64
/** Request resource not found in policy or allocators */
#define RM_SERVICE_DENIED_RES_DOES_NOT_EXIST       RM_SERVICE_DENIED_BASE+1
/** Request resource range within not found within resource's allocator */
#define RM_SERVICE_DENIED_RES_RANGE_DOES_NOT_EXIST RM_SERVICE_DENIED_BASE+2
/** Free request resource range not allocated to service's source inst */
#define RM_SERVICE_DENIED_RES_NOT_ALLOCD_TO_INST   RM_SERVICE_DENIED_BASE+3
/** Free request resource range already free */
#define RM_SERVICE_DENIED_RES_ALREADY_FREE         RM_SERVICE_DENIED_BASE+4
/** Allocate request resource range partially allocated (Handling of partial
 *  allocations not yet implemented) */
#define RM_SERVICE_DENIED_PARTIAL_ALLOCATION       RM_SERVICE_DENIED_BASE+5
/** Free request resource range partially free (Handling of partial frees not
 *  yet implemented) */
#define RM_SERVICE_DENIED_PARTIAL_FREE             RM_SERVICE_DENIED_BASE+6
/** Requirements of allocate request could not be satisfied (occurs for
 *  UNSPECIFIED base and/or alignment requests */
#define RM_SERVICE_DENIED_RES_ALLOC_REQS_NOT_MET   RM_SERVICE_DENIED_BASE+7
/** NameServer add request name string already exists in NameServer */
#define RM_SERVICE_DENIED_NAME_EXISTS_IN_NS        RM_SERVICE_DENIED_BASE+8
/** Service request instance not in policy "valid-instances" list */
#define RM_SERVICE_DENIED_INST_NAME_NOT_VALID      RM_SERVICE_DENIED_BASE+9
/** Init allocate request resource range not given init privileges in policy */
#define RM_SERVICE_DENIED_INIT_PERM_NOT_GIVEN      RM_SERVICE_DENIED_BASE+10
/** Use allocate request resource range not given use privileges in policy */
#define RM_SERVICE_DENIED_USE_PERM_NOT_GIVEN       RM_SERVICE_DENIED_BASE+11
/** Allocate request resource range marked as exclusive in policy has already
 *  been allocated */
#define RM_SERVICE_DENIED_EXCLUSIVE_RES_ALLOCD     RM_SERVICE_DENIED_BASE+12
/** Allocate request resource range allocated to an instance assigned exclusive
 *  privileges in policy */
#define RM_SERVICE_DENIED_ALLOCD_TO_EXCLUSIVE_INST RM_SERVICE_DENIED_BASE+13
/** Static allocate request was not an allocate-use or allocate-init request */
#define RM_SERVICE_DENIED_INVALID_STATIC_REQUEST   RM_SERVICE_DENIED_BASE+14
/** Static allocate request denied by static policy */
#define RM_SERVICE_DENIED_BY_STATIC_POLICY         RM_SERVICE_DENIED_BASE+15
/** RM instance locked from further services since a static allocation failed
 *  validation against global policy.  RM instance cannot be unlocked.  Please
 *  make sure static policy and global policy are in sync */
#define RM_SERVICE_DENIED_RM_INSTANCE_LOCKED       RM_SERVICE_DENIED_BASE+16
/** Allocate request denied because the resource is already reserved by Linux
 *  and "Shared Linux" privileges are not assigned to the requesting instance */
#define RM_SERVICE_DENIED_RES_NOT_SHARED_LINUX     RM_SERVICE_DENIED_BASE+17
/** Status request resource range partially found (Handling of partial status
 *  requests not yet implemented) */
#define RM_SERVICE_DENIED_PARTIAL_STATUS           RM_SERVICE_DENIED_BASE+18

/** RM Client Delegate instance is not stable.  RM system operation cannot be
 *  guarateed if a CD instance is used.  At the time please manage resources
 *  using Server and Client instances - tracked by SDOCM00100797 */
#define RM_WARNING_CD_INSTANCE_NOT_STABLE          (-1024)

/** Start of libfdt.h error codes */
#define RM_ERROR_LIBFDT_START                      (-1)
/** End of libfdt.h error codes */
#define RM_ERROR_LIBFDT_END                        (-13)

/** RM error base */
#define RM_ERROR_BASE                              (-64)
/** Instance name provided is NULL or greater than RM_NAME_MAX_CHARS */
#define RM_ERROR_INVALID_INST_NAME                 RM_ERROR_BASE-1
/** List of "valid-instances" not found in global or static policy */
#define RM_ERROR_NO_VALID_INST_IN_POLICY           RM_ERROR_BASE-2
/** Instance specified in permissions string does not match any instances
 *  specified in the "valid-instances" list */
#define RM_ERROR_PERM_STR_INST_NOT_VALID           RM_ERROR_BASE-3
/** Resource specified in global policy does not have an allocator */
#define RM_ERROR_UNKNOWN_RESOURCE_IN_POLICY        RM_ERROR_BASE-4
/** Permissions string has more than instance group specified.
 *  Ex: assignments = <12 1>, "iux = (RM_Client_Delegate) iu = (RM_Client)"; */
#define RM_ERROR_PERM_STR_TOO_MANY_INST_GROUPS     RM_ERROR_BASE-5
/** Permissions string has more than assignment.
 *  Ex: assignments = <12 1>, "iux = (RM_Client_Delegate) = i"; */
#define RM_ERROR_PERM_STR_TOO_MANY_ASSIGN_CHARS    RM_ERROR_BASE-6
/** Permissions string contains invalid character */
#define RM_ERROR_PERM_STR_INVALID_CHAR             RM_ERROR_BASE-7
/** Permissions string contains a permission character without the assignment
 *  operator Ex: assignments = <12 1>, "iux (RM_Client_Delegate)"; */
#define RM_ERROR_PERM_CHAR_WITHOUT_ASSIGN_CHAR     RM_ERROR_BASE-8
/** Permissions string contains a permission character on opposite side of
 *  already made assignment
 *  Ex: assignments = <12 1>, "iux = (RM_Client_Delegate) x"; */
#define RM_ERROR_INVALID_PERMS_CHAR_ON_RIGHT       RM_ERROR_BASE-9
/** Policy resource node contains an unknown property */
#define RM_ERROR_UNKNOWN_POLICY_RESOURCE_PROPERTY  RM_ERROR_BASE-10
/** Instance name provided in "valid-instances" list is greater than
 *  RM_NAME_MAX_CHARS */
#define RM_ERROR_VALID_INST_NAME_TOO_LONG          RM_ERROR_BASE-11
/** Instance name in permissions assignment is greater than RM_NAME_MAX_CHARS */
#define RM_ERROR_INST_NAME_IN_ASSIGNMENT_TOO_LONG  RM_ERROR_BASE-12
/** NameServer name in global resource list nameServer assignment and/or
 *  service request is greater than RM_NAME_MAX_CHARS */
#define RM_ERROR_NAMESERVER_NAME_TOO_LONG          RM_ERROR_BASE-13
/** Linux alias assignment in global resource list is invalid */
#define RM_ERROR_GRL_INVALID_LINUX_ALIAS_FORMAT    RM_ERROR_BASE-14
/** Error allocating memory for the service handle */
#define RM_ERROR_SERVICE_HANDLE_MEM_ALLOC_FAILED   RM_ERROR_BASE-15
/** The RM instance service handle has already been closed */
#define RM_ERROR_SERVICE_HANDLE_ALREADY_CLOSED     RM_ERROR_BASE-16
/** Global Resource List (GRL) resource node contains an unknown property */
#define RM_ERROR_GRL_UNKNOWN_RESOURCE_PROPERTY     RM_ERROR_BASE-17
/** Could not find an allocator for the specified resource */
#define RM_ERROR_RES_ALLOCATOR_DOES_NOT_EXIST      RM_ERROR_BASE-18
/** A resource node is specified more than once in either the
 *  Global Resource List (GRL) or policy.  GRL for RM Server. Policy for
 *  CD or Client */
#define RM_ERROR_RES_SPECIFIED_MORE_THAN_ONCE      RM_ERROR_BASE-19
/** No data was found at the GRL resource node's specified Linux alias path */
#define RM_ERROR_DATA_NOT_FOUND_AT_LINUX_ALIAS     RM_ERROR_BASE-20
/** RM server was not provided a Global Resource List (GRL) and global policy
 *  at initialization */
#define RM_ERROR_INVALID_SERVER_CONFIGURATION      RM_ERROR_BASE-21
/** Service request type not recognized */
#define RM_ERROR_INVALID_SERVICE_TYPE              RM_ERROR_BASE-22
/** rmAllocPkt transport callout returned NULL for rmPkt */
#define RM_ERROR_TRANSPORT_ALLOC_PKT_ERROR         RM_ERROR_BASE-23
/** rmSendPkt transport callout returned error when attempting to send the
 *  rmPkt */
#define RM_ERROR_TRANSPORT_SEND_ERROR              RM_ERROR_BASE-24
/** A RM service transaction could not be created for the service request */
#define RM_ERROR_SERVICE_TRANS_NOT_CREATED         RM_ERROR_BASE-25
/** RM service transaction could not be found in instance's transaction queue */
#define RM_ERROR_SERVICE_TRANS_DOES_NOT_EXIST      RM_ERROR_BASE-26
/** NameServer does not exist in instance, cannot satisfy NameServer service
 *  request */
#define RM_ERROR_NAMESERVER_DOES_NOT_EXIST         RM_ERROR_BASE-27
/** Service request to add a name to the NameServer failed */
#define RM_ERROR_NAMESERVER_NAME_ADD_FAILED        RM_ERROR_BASE-28
/** Could not find name specified in service request in NameServer */
#define RM_ERROR_NAMESERVER_NAME_DOES_NOT_EXIST    RM_ERROR_BASE-29
/** Service request made on Client or CD when no transport established and no
 *  static policy registered */
#define RM_ERROR_REQ_FAILED_NO_STATIC_POLICY       RM_ERROR_BASE-30
/** RM transport handle has not been registered with the RM instance */
#define RM_ERROR_TRANSPORT_HANDLE_DOES_NOT_EXIST   RM_ERROR_BASE-31
/** RM received a packet with an unknown RM packet type */
#define RM_ERROR_RECEIVED_INVALID_PACKET_TYPE      RM_ERROR_BASE-32
/** RM response packet does not match any requests sent from instance */
#define RM_ERROR_PKT_RESP_DOES_NOT_MATCH_ANY_REQ   RM_ERROR_BASE-33
/** Server attempted to connect to another server or a CD attempted to connect
 *  to another CD or Client attempted to connect to another client */
#define RM_ERROR_INVALID_REMOTE_INST_TYPE          RM_ERROR_BASE-34
/** RM client attempted to register with more than one Server or CD or a CD
 *  attempted to register with more than one Server */
#define RM_ERROR_ALREADY_REGD_SERVER_OR_CD         RM_ERROR_BASE-35
/** Instance type not recognized */
#define RM_ERROR_INVALID_INST_TYPE                 RM_ERROR_BASE-36
/** RM attempted to allocate a transport packet but the rmAllocPkt callout was
 *  not registered */
#define RM_ERROR_TRANSPORT_ALLOC_PKT_NOT_REGD      RM_ERROR_BASE-37
/** RM attempted to send a packet but the rmSendPkt callout was not
 *  registered */
#define RM_ERROR_TRANSPORT_SEND_NOT_REGD           RM_ERROR_BASE-38
/** RM instance cannot be deleted with transports still registered */
#define RM_ERROR_CANT_DELETE_WITH_REGD_TRANSPORT   RM_ERROR_BASE-39
/** RM instance cannot be deleted with open service handle */
#define RM_ERROR_CANT_DELETE_WITH_OPEN_SERV_HNDL   RM_ERROR_BASE-40
/** RM instance cannot be deleted when there are transactions pending and the
 *  ignorePendingServices parameter is set to false */
#define RM_ERROR_CANT_DELETE_PENDING_TRANSACTIONS  RM_ERROR_BASE-41
/** Only the Server instance can be used to return resource status via the
 *  Rm_resourceStatus API */
#define RM_ERROR_INVALID_RES_STATUS_INSTANCE       RM_ERROR_BASE-42
/** RM Shared Server and Client instances should always return a finished
 *  request since the instance has access to the resource structures no matter
 *  what core the service is requested from */
#define RM_ERROR_SHARED_INSTANCE_UNFINISHED_REQ    RM_ERROR_BASE-43
/** RM Shared Server and Client instances cannot register transports */
#define RM_ERROR_SHARED_INSTANCE_CANNOT_REG_TRANS  RM_ERROR_BASE-44
/** RM Shared Client handle was provided an invalid Shared Server handle.
 *  The shared server handle was either NULL or was not an instance of type
 *  Rm_instType_SHARED_SERVER */
#define RM_ERROR_INVALID_SHARED_SERVER_HANDLE      RM_ERROR_BASE-45
/** A RM Client failed to create a new transaction to request data from the
 *  Server in order to potentially process a transaction on a Client Delegate */
#define RM_ERROR_TRANS_REQ_TO_SERVER_NOT_CREATED   RM_ERROR_BASE-46
/** Service request required a policy check but instance was not initialized
 *  with a policy */
#define RM_ERROR_INSTANCE_HAS_NO_POLICY            RM_ERROR_BASE-47
/** RM Client Delegate was not provided a policy at initialization */
#define RM_ERROR_INVALID_CD_CONFIGURATION          RM_ERROR_BASE-48
/** RM CD freed local resources which allowed a free request of local request
 *  to be sent to the Server.  The Server free failed so the CD tried to
 *  realloc the local resources that were originally freed.  The re-allocate
 *  operation failed causing a resource loss on the CD */
#define RM_ERROR_LOST_RESOURCES_ON_CD              RM_ERROR_BASE-49
/** The service source inst name and RM packet source instance names are not
 *  available for the given type of RM packet */
#define RM_ERROR_PKT_AND_SERVICE_SRC_NOT_AVAIL     RM_ERROR_BASE-50
/** The provided character buffer that will contain the service source inst
 *  name or pkt source inst name is not of size RM_NAME_MAX_CHARS */
#define RM_ERROR_SRC_NAME_BUF_INVALID_SIZE         RM_ERROR_BASE-51
/** A resource name specified in the GRL, policy, and/or service request is
 *  greater than RM_NAME_MAX_CHARS */
#define RM_ERROR_RESOURCE_NAME_TOO_LONG            RM_ERROR_BASE-52
/** RM failed to create a new allocator - Occurred because an instance that
 *  does not use allocators tried to create one (RM Client) */
#define RM_ERROR_COULD_NOT_CREATE_NEW_ALLOCATOR    RM_ERROR_BASE-53
/** Could not allocate the memory needed to store the root entry of the
 *  allocator tree */
#define RM_ERROR_COULD_NOT_INIT_ALLOC_TREE         RM_ERROR_BASE-54
/** The instance's allocator tree could not be populated because of an invalid
 *  combination of instance type, GRL, and policy.  The valid combinations are:
 *  Server OR Shared Server --> Requires GRL and Policy.  Optional Linux DTB
 *  Client Delegate         --> Requires Policy.
 *  Client                  --> Optional Static Policy. */
#define RM_ERROR_INVALID_ALLOCATOR_INIT            RM_ERROR_BASE-55
/** A resource node in the GRL does not have at least a resource range or a
 *  NameServer assignment defined.  At a minimum, each node in the GRL must
 *  have a resource range or NameServer assignment */
#define RM_ERROR_GRL_INVALID_NODE_DEF              RM_ERROR_BASE-56
/** OSAL malloc failed when allocating an allocator node's resource tree */
#define RM_ERROR_MALLOC_FAILED_RES_TREE            RM_ERROR_BASE-57
/** OSAL malloc failed when allocating an allocator node's policy tree */
#define RM_ERROR_MALLOC_FAILED_POL_TREE            RM_ERROR_BASE-58
/** OSAL malloc failed when allocating a resource tree node */
#define RM_ERROR_MALLOC_FAILED_RES_NODE            RM_ERROR_BASE-59
/** A resource specified in the GRL could not be found in the policy */
#define RM_ERROR_RES_DOES_NOT_EXIST_IN_POLICY      RM_ERROR_BASE-60
/** Server's response to CD request has invalid service type */
#define RM_ERROR_SERVER_RESP_INVALID_SERVICE_TYPE  RM_ERROR_BASE-61

/**
 * @brief Maximum number of characters allowed for RM instance, resource, and
 *        NameServer names.
 */
#define RM_NAME_MAX_CHARS (32)

/**
 * @brief RM instance handle.  The RM handle is used to register transports
 *        between RM instances and request resource services from the RM
 *        instance.
 */
typedef void *Rm_Handle;

/**
 * @brief RM instance types
 */
typedef enum {
    /** RM Server */
    Rm_instType_SERVER = 0,
    /** RM Client Delegate */
    Rm_instType_CLIENT_DELEGATE,
    /** RM Client */
    Rm_instType_CLIENT,
    /** RM Shared Server - Server instance stored in shared memory that allows
     *  multiple DSP cores to request services without the need to configure
     *  and register transports.  Allows requests to be fulfilled from any DSP
     *  core without blocking */
    Rm_instType_SHARED_SERVER,
    /** RM Shared Client - Piggybacks on the Shared Server instance to handle
     *  service requests from resource and policy data structures in shared
     *  memory */
    Rm_instType_SHARED_CLIENT,
    /** DO NOT USE: Last type */
    Rm_instType_LAST
} Rm_InstType;

/**
 * @brief RM server (includes shared server) initialization configurations
 */
typedef struct {
    /** Pointer to the device global resource list (GRL).  The GRL contains 
     *  all resources on the device that will be managed by RM.  The GRL
     *  must be in DTB format. */
    void *globalResourceList;
    /** Pointer to the Linux DTB if the RM server is running on ARM Linux.
     *  The Linux DTB will be parsed for all managed resources reserved for use
     *  by Linux.  Parsing will be based on "linux-dtb-alias" resource
     *  properties found in the GRL.  The Linux DTB must be in DTB format.  */
    void *linuxDtb;
    /** Pointer to the global policy defining the allocation policies for
     *  RM instances within the system.  The global policy must be in DTB
     *  format. */
    void *globalPolicy;
} Rm_ServerCfg;

/**
 * @brief RM client delegate (CD) initialization configurations
 */
typedef struct {
    /** Pointer to a client delegate policy used by the CD to allocate resources
     *  without contacting the server.  The cdPolicy will be used by the CD to
     *  determine whether clients connected to the CD can be allocated resources
     *  provided to the CD by the server. 
     *
     *  The cdPolicy will also act as a static policy until the transport to the
     *  server has been established.  Static allocations can occur before the
     *  instance has been attached to a server instance within the RM system.
     *  This is useful for allocating resources prior to main().  Resources
     *  allocated via the static policy will be verified against the global
     *  policy once the CD connects with the server.  The static policy must be
     *  in DTB format.
     *
     *  To guarantee proper resource permission synchronization between the CD
     *  and server the cdPolicy must either be an exact copy of the globalPolicy
     *  or a exact replica of a subset of the globalPolicy provided to the
     *  server at initialization. */
    void *cdPolicy;
} Rm_ClientDelegateCfg;

/**
 * @brief RM client initialization configurations
 */
typedef struct {
    /** Pointer to a static policy used by the client to allocate resources
     *  statically.  Static allocations can occur before the instance has been
     *  attached to a server or CD instance within the RM system.  This is
     *  useful for allocating resources prior to main().  Resources allocated
     *  via the static policy will be verified against the global policy once
     *  the client connects with the server (directly or indirectly through a
     *  CD).  The static policy must be in DTB format. */
    void *staticPolicy;
} Rm_ClientCfg;

/**
 * @brief RM shared client initialization configurations
 */
typedef struct {
    /** RM Shared Server handle.  Typically, the Shared Server handle is
     *  created on DSP core designated for system initialization.  The
     *  application then the Shared Server handle, located in shared memory, to
     *  the other DSP cores.  These DSP cores can then piggyback onto the
     *  shared memory data structures in the Shared Server via the Shared
     *  Client */
     Rm_Handle sharedServerHandle;
} Rm_SharedClientCfg;

/**
 * @brief RM instance initialization structure
 */
typedef struct {
    /** Pointer to a character array containing the instance name.  The name of
     *  the RM instance can be no greater than #RM_NAME_MAX_CHARS.  The instance
     *  name must match an instance name defined in the "valid-instances" lists
     *  contained in the global and static policy DTS files.  Resources and
     *  privileges to the resources will be based on the name assigned to the
     *  RM instance*/
    const char  *instName;
    /** The type of RM instance that will be created. */
    Rm_InstType  instType;
    /** Instance's multi-threaded semaphore object.  Two or more RM instances
     *  can have their service transactions to the Server serialized by
     *  providing a valid multi-threaded semaphore object.  The semaphore
     *  object will be used by the Osal_rmMtCsEnter and Osal_rmMtCsExit
     *  functions.  The multi-threaded semaphore associated with the provided
     *  object will be taken when a service transaction is requested.  The
     *  semaphore will be released when the service response is returned.  This
     *  will serialize the transactions of any RM instance initialized with the
     *  same multi-threaded sem object.  Multi-threaded semaphores will not be
     *  used if NULL is provided as the object */
    uint32_t    *mtSemObj;
    /** Instance-type specific configurations */
    union {
        /** #Rm_ServerCfg */
        Rm_ServerCfg         serverCfg;
        /** #Rm_ClientDelegateCfg */
        Rm_ClientDelegateCfg cdCfg;
        /** #Rm_ClientCfg */
        Rm_ClientCfg         clientCfg;
        /** #Rm_SharedClientCfg */
        Rm_SharedClientCfg   sharedClientCfg;
    } instCfg;
} Rm_InitCfg;

/**
 *  @b Description
 *  @n
 *      This function prints and returns the status for all resources managed by
 *      the RM instance network.  The allocate/free status as well as ownership
 *      status will be printed for every resource.  Also, the NameServer name
 *      entries will be displayed.
 *
 *      This function will return error if a Client handle is provided since
 *      Clients do not track any resource data structures
 *
 *  @param[in]  rmHandle
 *      Instance handle.
 *
 *  @param[in]  printResources
 *      Non-zero - Resource ownership details will be printed for all
 *                 tracked resources
 *      0        - Resource ownership details will not be printed.  Only
 *                 the number of allocated resource ranges will be 
 *                 returned.
 *
 *  @retval
 *      Success - Total number of allocated resource nodes owners.  Effectively
 *                returns the number of resource ranges still allocated.
 *  @retval
 *      Failure - #RM_ERROR_INVALID_RES_STATUS_INSTANCE
 */
int32_t Rm_resourceStatus(Rm_Handle rmHandle, int printResources);

/**
 *  @b Description
 *  @n
 *      This function prints the current status of a RM instance.  The
 *      following instance properties will be printed:
 *      a) instance name & type
 *      b) The instance's registered transports 
 *      c) All service transactions queued in the instance transaction
 *         queue and their states
 *
 *  @param[in]  rmHandle
 *      Instance handle.
 */
void Rm_instanceStatus(Rm_Handle rmHandle);

/**
 *  @b Description
 *  @n
 *      This function initializes a RM instance.  There are no restrictions
 *      on the amount of times this function can be called.  Each call will
 *      result in a new RM instance.  However, a network of RM instances
 *      can have only one RM Server.  If an application has multiple RM
 *      Servers the resources managed by each server must be mutually
 *      exclusive.  Additionally if an application has multiple RM shared
 *      servers the resources they manage must be mutually exclusive as well
 *
 *      If any errors are encountered during the initialization process
 *      the Rm_Handle returned will be NULL.
 *
 *  @param[in]  initCfg
 *      Pointer to the instance initialization structure.
 *
 *  @param[out] result
 *      Pointer to a signed int used to return any errors encountered during
 *      the instance initialization process.
 *
 *  @retval
 *      Success - Rm_Handle for instance and result = #RM_OK
 *  @retval
 *      Failure - NULL Rm_Handle and result < #RM_OK
 */
Rm_Handle Rm_init(const Rm_InitCfg *initCfg, int32_t *result);

/**
 *  @b Description
 *  @n
 *      This function deletes the specified RM instance.  All memory
 *      associated with the instance will be freed.
 *
 *  @param[in]  rmHandle
 *      Instance handle.
 *
 *  @param[in]  ignorePendingServices
 *      Non-zero - The instance will be deleted despite any services pending<br>
 *      0        - The instance will not be deleted due to at least one service
 *                 pending.
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_CANT_DELETE_WITH_OPEN_SERV_HNDL
 *  @retval
 *      Failure - #RM_ERROR_CANT_DELETE_WITH_REGD_TRANSPORT
 *  @retval
 *      Failure - #RM_ERROR_CANT_DELETE_PENDING_TRANSACTIONS 
 */
int32_t Rm_delete(Rm_Handle rmHandle, int ignorePendingServices);

/**
 *  @b Description
 *  @n
 *      The function is used to get the version information of RM.
 *
 *  @retval
 *      Version Information.
 */
uint32_t Rm_getVersion(void);

/**
 *  @b Description
 *  @n
 *      The function is used to get the version string for RM.
 *
 *  @retval
 *      Version String.
 */
const char* Rm_getVersionStr(void);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* RM_H_ */

