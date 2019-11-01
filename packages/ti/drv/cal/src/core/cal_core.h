/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 */

/**
 *  \file cal_core.h
 *
 *  \brief CAL core interface.
 *  This file defines the commom interface for the each of the core
 *  modules present in CAL.
 */

#ifndef CAL_CORE_H_
#define CAL_CORE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/cal/cal.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Global handle representation for CORE before CORE has been opened. */
#define CAL_CORE_COMMON_HANDLE        (0xFFFFFFFFU)

/** \brief Typedef for Core driver instance handle returned by init function. */
typedef void *Cal_CoreInst;

/** \brief Typedef for Core driver handle returned by open function. */
typedef void *Cal_CoreHandle;

/**
 *  enum Cal_CoreName
 *  \brief Enumerations for the core names.
 */
typedef enum
{
    CAL_CORE_CAPT,
    /**< CAL core. */
    CAL_CORE_MAX
    /**< Should be the last value of this enumeration.
     *   Will be for validating the input parameters. */
} Cal_CoreName;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Cal_CoreProperty
 *  \brief Properties of a particular core.
 */
typedef struct
{
    Cal_CoreName name;
    /**< Core name. */
    Bool         isDropDataSupport;
    /**< Set this to TRUE if the hardware corresponding to the core supports
     *   drop data feature. This will be used by the driver to decide what
     *   to do when the application is not queueing data at the capture/display
     *   rate. */
} Cal_CoreProperty;

/**
 *  enum Cal_CoreFrame
 *  \brief Frame structure containing information about video buffer exchanged
 *  between driver and core.
 */
typedef struct
{
    void  *drvData;
    /**< Driver specific data - used by driver, core can't modify or use. */
    uint32_t streamId;
    /**< 0 - STREAM ID MAX used in single channel,
     *   or for anc-data in multi-channel. */
    uint32_t chId;
    /**< 0-15 maps to actual VPDMA-channel – used in multi-channel. */
    uint64_t addr[FVID2_MAX_PLANES];
    /**< Array of frame buffer pointers for given stream.
     *   Size of array shall be "numFrames"
     *   Each entry is equivalent to "addr" in #Fvid2_Frame. */
    uint32_t dropFrm;
    /**< If TRUE core should drop the frame. */
    void  *rtParams;
    /**< run time parameters that can be changed for each frames
     *   e.g height,width,position of frame etc  */

    /*
     * Below variables are output status from the core at the time of frame
     * done callback.
     */
    uint32_t fid;
    /**< Indicates whether this frame belong to top or bottom field.
     *   For valid values see #Fvid2_Fid. */
    uint32_t width;
    /**< Captured width in pixels. */
    uint32_t height;
    /**< Captured height in lines. */
    void *coreStats;
    /**< Some additional statistics information which core can provide for
     *   debugging. eg: HW errors observed. */
    uint32_t status;
    /**< Status of the frame, see #Fvid2_FrameStatus for the valid values.
            This will be update by the cores to indicate validity of frame */
} Cal_CoreFrame;

/**
 *  \brief Extern function to be implemented by driver to provide a new frame
 *  buffers from application to the core.
 */
typedef Cal_CoreFrame * (*Cal_CoreReqFrameCb)(void  *drvData,
                                              uint32_t streamId,
                                              uint32_t chId);

/**
 *  \brief Extern callback function to be implemented by driver to free
 *  up buffers to the application specified by the core.
 */
typedef int32_t (*Cal_CoreFrameDoneCb)(void                *drvData,
                                     const Cal_CoreFrame *frm);

typedef int32_t (*Cal_CoreFrameDoneNotifyCb)(void *drvData);

/**
 *  \brief Extern callback function to be implemented by driver to notify
 *      completion of reception of sub-frame
 */
typedef int32_t (*Cal_CoreSubFrameCbFxn)(void                 *drvData,
                                        const Cal_CoreFrame *frm,
                                        Fvid2_SubFrameInfo  *subFrmInfo);

/**
 *  enum Cal_CoreFrame
 *  \brief Frame structure containing information about video buffer exchanged
 *  between driver and core.
 */
typedef struct
{
    void                     *drvData;
    /**< Driver specific data which will be passed during callback functions. */
    Cal_CoreReqFrameCb        reqFrmCb;
    /**< Request frame callback function. This cannot be NULL. */
    Cal_CoreFrameDoneCb       frmDoneCb;
    /**< Frame complete callback function. This cannot be NULL. */
    Cal_CoreFrameDoneNotifyCb frmDoneNotifyCb;
    /**< Frame completion notification to upper layer, can be NULL
     *   depending on the driver */
} Cal_CoreOpenPrms;

/** \brief Typedef for core open function pointer. */
typedef Cal_CoreHandle (*Cal_CoreOpen)(Cal_CoreInst            instObj,
                                       const Cal_CoreOpenPrms *openPrms,
                                       const void             *coreOpenPrms,
                                       void                   *coreReturnPrms);

/** \brief Typedef for core close function pointer. */
typedef int32_t (*Cal_CoreClose)(Cal_CoreHandle handle);

/** \brief Typedef for core get property function pointer. */
typedef int32_t (*Cal_CoreGetProperty)(Cal_CoreInst      instObj,
                                     Cal_CoreProperty *property);

/**
 *  \brief Typedef for core set parameters function pointer.
 *  Each core will have its own set of params and will be defined in the
 *  respective core header file.
 */
typedef int32_t (*Cal_CoreSetParams)(Cal_CoreHandle handle,
                                   const void    *params,
                                   void          *subFrameRetPrms);

/**
 *  \brief Typedef for core get parameters function pointer.
 *  Each core will have its own set of params and will be defined in the
 *  respective core header file.
 */
typedef int32_t (*Cal_CoreGetParams)(Cal_CoreHandle handle, void *params);

/** \brief Typedef for core control functions. */
typedef int32_t (*Cal_CoreControl)(Cal_CoreHandle handle,
                                 uint32_t         cmd,
                                 void          *appArgs,
                                 void          *drvArgs);

/**
 *  \brief Typedef for core start function pointer.
 *  This will start the capture/display/m2m operation.
 */
typedef int32_t (*Cal_CoreStart)(Cal_CoreHandle handle);

/**
 *  \brief Typedef for core stop function pointer.
 *  This will stop the capture/display/m2mure operation.
 */
typedef int32_t (*Cal_CoreStop)(Cal_CoreHandle handle);

/**
 *  \brief Typedef for core process function pointer.
 *  This should be called by the driver when it gets the VSYNC interrupt from
 *  the hardware.
 */
typedef int32_t (*Cal_CoreProcessIsr)(Cal_CoreHandle handle, uint32_t chId);

/**
 *  \brief Typedef for core prog buffer function pointer.
 *  This should be called by the driver when it gets the buffer from app and if
 *  previous buffer is repeated.
 */
typedef int32_t (*Cal_CoreprogBuffer)(Cal_CoreHandle handle,
                                    Cal_CoreFrame *frm,
                                    uint32_t bypassLowLatencyCheck);

/**
 *  \brief Typedef for core to program multiple frame pointers.
 */
typedef int32_t (*Cal_CoreputFrames)(Cal_CoreHandle  handle,
                                   Cal_CoreFrame **frm);

/**
 *  \brief Typedef for core to get multiple frame pointers back.
 */
typedef int32_t (*Cal_CoregetFrames)(Cal_CoreHandle  handle,
                                   Cal_CoreFrame **frm);

/**
 *  \brief Typedef for core get Error Status function pointer.
 *  Each core will have its own set of params and will be defined in the
 *  respective core header file.
 */
typedef int32_t (*Cal_CoregetErrorStat)(Cal_CoreHandle handle,
                                      void          *retParams);

/**
 *  struct Cal_CoreOps
 *  \brief Structure to store core function pointers.
 */
typedef struct
{
    Cal_CoreGetProperty  getProperty;
    /**< Get property function pointer. */
    Cal_CoreOpen         open;
    /**< Open function pointer. */
    Cal_CoreClose        close;
    /**< Close function pointer. */
    Cal_CoreSetParams    setParams;
    /**< Set parameters function pointer. */
    Cal_CoreGetParams    getParams;
    /**< Get parameters function pointer. */
    Cal_CoreControl      control;
    /**< Core control function pointer. */
    Cal_CoreStart        start;
    /**< Core start function pointer. */
    Cal_CoreStop         stop;
    /**< Core stop function pointer. */
    Cal_CoreProcessIsr   proc;
    /**< Core Process ISR function pointer. */
    Cal_CoreprogBuffer   progBuffer;
    /**< Core prog Buffer function pointer. */
    Cal_CoreputFrames    putFrames;
    /**< Core put Frames function pointer. */
    Cal_CoregetFrames    getFrames;
    /**< Core get Frames function pointer. */
    Cal_CoregetErrorStat getErrorStat;
} Cal_CoreOps;

/**
 *  \brief Copies a FVID2 Frame to CAL Frame
 *         Currently Copies only Addresses
 *
 *  \param calFrm       [IN] Pointer to CAL Frame.
 *  \param fvid2Frm     [IN] Pointer to FVID2 Frame.
 *
 */
static inline void Cal_CorecopyFvid2ToCalFrame(Cal_CoreFrame *calFrm,
                                               const Fvid2_Frame   *fvid2Frm);

/**
 *  \brief Copies a CAL Frame to FVID2 Frame
 *         Currently Copies only buffer Addresses
 *
 *  \param fvid2Frm     [IN] Pointer to FVID2 Frame.
 *  \param calFrm       [IN] Pointer to CAL Frame.
 *
 */
static inline void Cal_CorecopyCalToFvid2Frame(Fvid2_Frame   *fvid2Frm,
                                               const Cal_CoreFrame *calFrm);

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static inline void Cal_CorecopyFvid2ToCalFrame(Cal_CoreFrame *calFrm,
                                               const Fvid2_Frame   *fvid2Frm)
{
    if((NULL != calFrm) && (NULL != fvid2Frm))
    {
        calFrm->addr[0U] = fvid2Frm->addr[0U];
        calFrm->addr[1U] = fvid2Frm->addr[1U];
        calFrm->addr[2U] = fvid2Frm->addr[2U];
        calFrm->addr[3U] = fvid2Frm->addr[3U];
        calFrm->addr[4U] = fvid2Frm->addr[4U];
        calFrm->addr[5U] = fvid2Frm->addr[5U];
    }
}

static inline void Cal_CorecopyCalToFvid2Frame(Fvid2_Frame   *fvid2Frm,
                                               const Cal_CoreFrame *calFrm)
{
    if((NULL != calFrm) && (NULL != fvid2Frm))
    {
        fvid2Frm->addr[0U] = calFrm->addr[0U];
        fvid2Frm->addr[1U] = calFrm->addr[1U];
        fvid2Frm->addr[2U] = calFrm->addr[2U];
        fvid2Frm->addr[3U] = calFrm->addr[3U];
        fvid2Frm->addr[4U] = calFrm->addr[4U];
        fvid2Frm->addr[5U] = calFrm->addr[5U];
    }
}

#ifdef __cplusplus
}
#endif

#endif /* CAL_CORE_H_ */

