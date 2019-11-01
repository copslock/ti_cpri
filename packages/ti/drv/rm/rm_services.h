/**
 *   @file  rm_services.h
 *
 *   @brief
 *      This is the RM include file for services provided to components that
 *      register a RM instance
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

#ifndef RM_SERVICES_H_
#define RM_SERVICES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RM includes */
#include <ti/drv/rm/rm.h>

/**
@addtogroup RM_SERVICES_API
@{
*/

/** 
 * @brief RM service types
 */
typedef enum {
    /** RM resource allocate for initialization service */
    Rm_service_RESOURCE_ALLOCATE_INIT = 0,
    /** RM resource allocate for use service */
    Rm_service_RESOURCE_ALLOCATE_USE,
    /** RM resource free service */
    Rm_service_RESOURCE_FREE,
    /** RM resource status service - returns the reference count for specified
     *  resource. */
    Rm_service_RESOURCE_STATUS,
    /** RM NameServer map resource to name service */
    Rm_service_RESOURCE_MAP_TO_NAME,
    /** RM NameServer get resource by name service */
    Rm_service_RESOURCE_GET_BY_NAME,
    /** RM NameServer unmap resource from name service */
    Rm_service_RESOURCE_UNMAP_NAME,
    /** DO NOT USE: Last type */
    Rm_service_LAST
} Rm_ServiceType;

/**
 * @brief RM service response information used by RM to provide service
 *        request results back to the application components
 */
typedef struct {
    /** RM instance handle from which the service request that spawned this
     *  result originated.  Used by application to sort responses, received
     *  via callback function, from RM instances located on the same core. */
    Rm_Handle rmHandle;
    /** Service request state.  State values can be found in rm.h starting
     *  with #RM_SERVICE_PROCESSING, #RM_SERVICE_DENIED_BASE, and
     *  #RM_ERROR_LIBFDT_START */
    int32_t   serviceState;
    /** The service ID is returned to the application in order to match service
     *  responses received at a later time via the provided callback function
     *  because RM required a blocking operation in order to satisfy the
     *  resource request. <br> <br> The service ID will never have a value of
     *  zero. */
    uint32_t  serviceId;
    /** Affected resource name */
    char      resourceName[RM_NAME_MAX_CHARS];
    /** The resource base value allocated, freed, or mapped to NameServer
     *  name. */
    int32_t   resourceBase;
    /** The resource length starting at base allocated, freed, or mapped to
     *  NameServer name. */
    uint32_t  resourceLength;
/** resourceNumOwners is not valid unless >= 0 */
#define RM_RESOURCE_NUM_OWNERS_INVALID (-1)
    /** Current number of owners for the returned resource.  A value greater
     *  than one means the resource is being shared.  This value is only valid
     *  if the serviceState is RM_SERVICE_APPROVED or
     *  RM_SERVICE_APPROVED_STATIC. */
    int32_t   resourceNumOwners;
/** instAllocCount is not valid unless >= 0 */
#define RM_INST_ALLOC_COUNT_INVALID    (-1)
    /** Number of times the requesting instance has allocated the returned
     *  resource.  This value is only valid if the serviceState is
     *  RM_SERVICE_APPROVED or RM_SERVICE_APPROVED_STATIC */
    int32_t   instAllocCount;
} Rm_ServiceRespInfo;

/**
 * @brief RM service callback function
 */
typedef struct {
    /** Component callback function.  RM calls this function when a blocking
     *  resource service request is complete. The callback function supplied
     *  for this parameter must match the function pointer prototype. */
    void (*serviceCallback) (Rm_ServiceRespInfo *serviceResponse);
} Rm_ServiceCallback;

/**
 * @brief RM service request information
 */
typedef struct {
    /** The type of service requested */
    Rm_ServiceType      type;
    /** Pointer to an array containing the resource name affected by
     *  the request.  The resource name must match a resource node name
     *  defined in the GRL and global/static policy.  The request will be
     *  denied if the resource name does not match any resource node names
     *  in the policy */
    const char         *resourceName;
/** Informs RM to find the next available resource block of length
 *  resourceLength and alignment resourceAlignment for allocation.  This
 *  parameter is only valid for resource allocate service types. */
#define RM_RESOURCE_BASE_UNSPECIFIED (-1)  
    /** The base value of the resource affected by the service request.
     *  #RM_RESOURCE_BASE_UNSPECIFIED can be substituted. */
    int32_t             resourceBase;
    /** The resource length, starting from #resourceBase affected by the
     *  service request. */
    uint32_t            resourceLength;
/** Informs RM to find the next available resource block with length
 *  resourceLength and the alignment specified in 
 *  a) The resource node in the policy if it has the "allocation-alignment"
 *     property defined.
 *  b) The default alignment of 1 if no alignment is specified in the policy
 *     for the resource.
 *  This value is only valid if resourceBase is set to
 *  #RM_RESOURCE_BASE_UNSPECIFIED */
#define RM_RESOURCE_ALIGNMENT_UNSPECIFIED (-1)
    /** Alignment of the resource affected by the service request.  Only valid
     *  if resourceBase is set to #RM_RESOURCE_BASE_UNSPECIFIED.
     *  #RM_RESOURCE_ALIGNMENT_UNSPECIFIED can be substituted. */
    int32_t             resourceAlignment;
    /** The NameServer name associated, or to be associated, with a resource.
     *  The NameServer name has precedence over #resourceBase and
     *  #resourceLength for all resource modification service types as well as
     *  #Rm_service_RESOURCE_GET_BY_NAME.  If the NameServer name and the base
     *  and length are not NULL the resource information retrieved from the
     *  NameServer entry for the name will replace the values present in
     *  #resourceBase and #resourceLength */
    const char         *resourceNsName;
    /** Callback function used by RM to provide responses back to application
     *  components after a service request resulted in a blocking operation.
     *  If no callback function is provided the RM instance will block until
     *  the service response is ready. */
    Rm_ServiceCallback  callback;
} Rm_ServiceReqInfo;

/**
 * @brief RM service handle provided to application components for requesting
 *        services
 */
typedef struct {
    /** RM instance handle from which the service handle was allocated from. */
    void *rmHandle;
    /**
     *  @b Description
     *  @n
     *      Processes service requests from application components.  Responses
     *      are returned immediately if the service request could be satisfied
     *      without blocking.  If the service request requires a blocking
     *      operation, such as forwarding the service request to another
     *      instance for validation, the response will be received via the
     *      Rm_ServerCallback
     *
     *   @param[in] rmHandle
     *      RM instance handle specifies the instance that handles the service
     *      request.  The request's result, if policy checks are involved, will
     *      be based on the permissions assigned to the rmHandle's instance
     *      name within global/static policy.
     *
     *  @param[in]  serviceRequest
     *      Pointer to the service request structure
     *
     *  @param[out]  serviceResponse
     *      Pointer to a service response structure.
     */
    void (*Rm_serviceHandler)(void *rmHandle,
                              const Rm_ServiceReqInfo *serviceRequest,
                              Rm_ServiceRespInfo *serviceResponse);
} Rm_ServiceHandle;

/**
 *  @b Description
 *  @n
 *      This function returns a RM service handle to the application to
 *      provide services to software components (LLDs, BIOS, etc) that want to
 *      use RM for resource management. Only one service handle can be opened
 *      from each RM instance.
 *
 *  @param[in]  rmHandle
 *      RM instance handle from which the service handle will be opened
 *
 *  @param[out] result
 *      Pointer to a signed int used to return any errors encountered during
 *      the instance initialization process.
 *
 *  @retval
 *      Success - Rm_ServiceHandle and result = #RM_OK
 *  @retval
 *      Failure - NULL Rm_ServiceHandle and
 *                result = #RM_ERROR_SERVICE_HANDLE_MEM_ALLOC_FAILED
 */
Rm_ServiceHandle *Rm_serviceOpenHandle(Rm_Handle rmHandle, int32_t *result);

/**
 *  @b Description
 *  @n
 *      This function closes a RM instance's service handle
 *
 *  @param[in]  rmServiceHandle
 *      RM instance service handle to be closed.
 *
 *  @retval
 *      Success - #RM_OK
 *  @retval
 *      Failure - #RM_ERROR_SERVICE_HANDLE_ALREADY_CLOSED
 */
int32_t Rm_serviceCloseHandle(Rm_ServiceHandle *rmServiceHandle);

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* RM_SERVICES_H_ */

