/**
 *   @file  linuxutil.c
 *
 *   @brief
 *          contains device agnostic linux code
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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

#ifdef __LINUX_USER_SPACE
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <dirent.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "fw_test.h"
#include "linuxutil.h"

#include <ti/drv/qmss/qmss_drv.h>

#define MAX_NAME_LENGTH         64
#define MAX_FILE_NAME_LENGTH    128
#define MAX_LINE_LENGTH         16

/* Read a string out of a /proc or /sys entry */
static int uioutil_get_string (char *filename, char *str, int str_len)
{
    FILE *fpr = 0;
    int ret_val = -1;

    if (!filename || !str || !str_len) {
        goto close_n_exit;
    }

    fpr = fopen(filename, "r");
    if (!fpr) {
        printf("Error opening file %s (%s)",
               filename, strerror(errno));
        goto close_n_exit;
    }
    if (!fgets(str, str_len, fpr)) {
        printf("Error reading file %s (%s)",
               filename, strerror(errno));
        goto close_n_exit;
    }
    /* Terminate string at new line or carriage return if any */
    str[strcspn(str, "\n\r")] = '\0';

    ret_val = 0;

close_n_exit:
    if (fpr)
    {
        fclose(fpr);
    }
    return ret_val;
}

/* Read a big endian uint32_t out of a /proc/device-tree entry */
static int dt_get_uint32 (char *filename, uint32_t *val)
{
    FILE *fpr = 0;
    int ret_val = -1;
    unsigned char buffer[sizeof(uint32_t)];

    if (!filename || !val) {
        goto close_n_exit;
    }

    fpr = fopen(filename, "r");
    if (!fpr) {
        printf("Error opening file %s (%s)", filename, strerror(errno));
        goto close_n_exit;
    }
    if (fread (buffer, sizeof(uint32_t), 1, fpr) != 1) {
        printf("Error reading file %s (%s)", filename, strerror(errno));
        goto close_n_exit;
    }
    /* dt is always big endian, swizzle */
    *val = (buffer[0] << 24) |
           (buffer[1] << 16) |
           (buffer[2] <<  8) |
           (buffer[3]      );

    ret_val = 0;

close_n_exit:
    if (fpr)
    {
        fclose(fpr);
    }

    return ret_val;
}

/* Remove everything after and including "." from a name */
static char  *remove_postfix_from_device_name(char *name, int max_length)
{
    int index;

    index=0;

    /* strip anything after . in the name */
    while(name[index] != '\0')
    {
        if(name[index] == '.')
        {
            name[index] = '\0';
            index++;
            return(&name[0]);
        }
        if(index > max_length)
            break;
        index++;
    }
    return(name);
}
int openUioForQID (
    int inputQueueNum
)
{
    struct dirent *entry = 0;
    char          filename[MAX_FILE_NAME_LENGTH];
    char          devname[MAX_FILE_NAME_LENGTH];
    char          name[MAX_NAME_LENGTH];
    DIR          *dir = NULL;
    uint32_t      queueNum;
    int           ret_val = -1, fd_uio;
    char         *name_ptr;
    char         basename[MAX_NAME_LENGTH]="qpend";

    /* dig through /sys/class/uio to find devices with name
     * starting with basename */
    dir = opendir("/sys/class/uio");
    if (!dir) {
        printf("readdir /sys/class/uio");
        goto close_n_exit;
    }
    while ((entry = readdir(dir)) != NULL) {
        if (strstr (entry->d_name, "uio") == NULL) {
            continue;
        }
        snprintf(filename, MAX_FILE_NAME_LENGTH, "/sys/class/uio/%s/name", entry->d_name);

        if (uioutil_get_string(filename, name, MAX_NAME_LENGTH) < 0)
            goto close_n_exit;

        name_ptr = remove_postfix_from_device_name(name, MAX_NAME_LENGTH);

        if (!strncmp (basename, name_ptr, strlen(basename))) {
            /* Now find associated queue number in
             * /proc/device-tree/soc/<classname>/queue
             */
            snprintf(filename, MAX_FILE_NAME_LENGTH,
                     "/proc/device-tree/soc/%s/cfg-params/ti,qm-queue",
                     name_ptr);
            if (dt_get_uint32 (filename, &queueNum) < 0)
                goto close_n_exit;

            if(queueNum == inputQueueNum) {
                /* Now open the /dev/uio## associated with this qpend
                 * in order to block on interrupt */
                snprintf (devname, MAX_FILE_NAME_LENGTH, "/dev/%s",
                    entry->d_name);

                if ( (fd_uio = open (devname, (O_RDWR | O_SYNC))) < 0)
                {
                    printf ("Error open %s (%s)\n", devname, strerror(errno));
                    goto close_n_exit;
                }
		        return(fd_uio);
            }
        }
    }

close_n_exit:
    if (dir)
    {
        closedir(dir);
    }

    return ret_val;

}

/**
 *  @b Description
 *  @n
 *      Find a qpend capable pairing between a queue and device entry
 *
 *  1) Search /sys/class/uio/ * /name that matches basename
 *     this * defines which device shall be used for waiting for interrupts.
 *  2) Find this device in /proc/device-tree in order to find "queue" attribute
 *     placed in device tree.  This is the queue number assocaited with the qpend.
 *  3) Open the queue.  If RM allows it, use it, else move to next uio device.
 *  4) Open the /dev/uio## that was derived from /sys/class/uio.
 *
 *  @param[in]  basename
 *      Base name of device to find (eg qpend)
 *  @param[out] rxQueueHnd
 *      Queue handle of queue that is paired with the device
 *
 *  @retval
 *      fd to use to block on interrupt, or < 0 on error.
 */
int acquireUioRxQueue (
    const char *basename,
    Qmss_QueueHnd *rxQueueHnd
)
{
    struct dirent *entry = 0;
    char          filename[MAX_FILE_NAME_LENGTH];
    char          devname[MAX_FILE_NAME_LENGTH];
    char          name[MAX_NAME_LENGTH];
    int           ret_val = -1, fd_uio;
    DIR          *dir = NULL;
    char         *name_ptr;
    uint32_t      queueNum;
    uint8_t       isAllocated;
    Qmss_QueueHnd queHnd;

    /* dig through /sys/class/uio to find devices with name
     * starting with basename */
    dir = opendir("/sys/class/uio");
    if (!dir) {
        printf("readdir /sys/class/uio");
        goto close_n_exit;
    }
    while ((entry = readdir(dir)) != NULL) {
        if (strstr (entry->d_name, "uio") == NULL) {
            continue;
        }

        snprintf(filename, MAX_FILE_NAME_LENGTH, "/sys/class/uio/%s/name", entry->d_name);

        if (uioutil_get_string(filename, name, MAX_NAME_LENGTH) < 0)
            goto close_n_exit;

        name_ptr = remove_postfix_from_device_name(name, MAX_NAME_LENGTH);

        if (!strncmp (basename, name_ptr, strlen(basename))) {
            /* Now find associated queue number in
             * /proc/device-tree/soc/<classname>/queue
             */
            snprintf(filename, MAX_FILE_NAME_LENGTH,
                     "/proc/device-tree/soc/%s/cfg-params/ti,qm-queue",
                     name_ptr);
            if (dt_get_uint32 (filename, &queueNum) < 0)
                goto close_n_exit;
            /* Try opening the queue */
            if ((queHnd = Qmss_queueOpen (QMSS_PARAM_NOT_SPECIFIED, queueNum, &isAllocated)) >= 0)
            {
                if (isAllocated == 1) /* I'm first to get the queue */
                {
                    /* Now open the /dev/uio## associated with this qpend
                     * in order to block on interrupt */
                    snprintf (devname, MAX_FILE_NAME_LENGTH, "/dev/%s",
                              entry->d_name);

                    if ( (fd_uio = open (devname, (O_RDWR | O_SYNC))) < 0)
                    {
                        printf ("Error open %s (%s)\n", devname, strerror(errno));
                        goto close_n_exit;
                    }

                    /* It worked! */
                    *rxQueueHnd = queHnd;
                    ret_val = fd_uio;
                    printf ("Successfully opened RX QPEND queue %d\n", queueNum);
                    printf ("Successfully opened UIO dev %s\n", devname);
                    goto close_n_exit;
                }
                else
                {
                    /* Somebody got this queue already */
                    if (Qmss_queueClose (queHnd) < QMSS_SOK)
                    {
                        printf ("Failed to close already owned queue %d\n", queHnd);
                        goto close_n_exit;
                    }
                    /* else fall through and try another device */
                }
            }
            /* Else, fall through and try another uio device to see if its
             * underlying queue is available */
        }
    }

close_n_exit:
    if (dir)
    {
        closedir(dir);
    }

    return ret_val;
}

/**
 *  @b Description
 *  @n
 *      Inverse of acquireUioRxQueue
 *
 *  Closes fd and closes rxQueHnd.
 *
 *  @param[in]  fd
 *      fd returned by acquireUioRxQueue
 *  @param[in]  rxQueHnd
 *      Queue handle returned by acquireUioRxQueue
 *
 *  @retval
 *      0: success
 *      < 0: failed
 */
int releaseUioRxQueue (int fd, Qmss_QueueHnd rxQueHnd)
{
    Qmss_Result result;
    int retval = 0;

    /* Close the queues */
    if ((result = Qmss_queueClose (rxQueHnd)) < QMSS_SOK)
    {
        retval = -1;
        printf ("Fail closing Rx queue %d error code : %d\n",
                (uint32_t)rxQueHnd, result);
    }

    if (close(fd) < 0)
    {
        printf ("Failed to close uio fd: %s\n", strerror(errno));
        retval = -2;
    }

    return retval;
}

/**
 *  @b Description
 *  @n
 *      Waits for an interrupt using read().
 *
 *  @param[in]  fd
 *      fd returned by acquireUioRxQueue
 *
 *  @retval
 *      > 0 : # of interrupts since last call
 *      <= 0: failed
 */
int waitForInterrupt (int fd)
{
    uint32_t count;
    uint32_t arm = 1;
    int retval = -1;
    /* Enable the interrupt */
    if (write(fd, &arm, sizeof(arm)) != sizeof(arm))
    {
        printf ("write() to enable interrupt failed\n");
    }
    /* Note its possible to use select() as well.  This facilitates
     * waiting on multiple qpends if required */
    else if ( (read (fd, &count, sizeof(count))) != sizeof(count))
    {
        printf ("read() on block for interrupt failed\n");
    }
    else
    {
        retval = (int)count;
    }

    return retval;
}

/**
 *  @b Description
 *  @n
 *      Runs taskFcn as a thread with argument taskArgs
 *
 *  @param[in]  taskFcn
 *      Function to run as thread
 *  @param[in]  taskArgs
 *      Will be given to taskFcn as its argument by thread library
 *
 *  @retval
 *      Thread handle.  Must be freed by calling waitReceiveTask
 */
void *makeReceiveTask (void *(*taskFcn)(void *), void *taskArgs)
{
    pthread_t *handle = malloc (sizeof(pthread_t));
    if (! handle)
    {
        return NULL;
    }
    if (fw_task_create (taskFcn, taskArgs, handle) == 0)
    {
        return handle;
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Blocks until thread finishes.  Frees thread handle when done.
 *
 *  @param[in]  handle
 *      Thread handle returned by makeReceiveTask.
 *
 *  @retval
 *      None
 */
void waitReceiveTask (void *handle)
{
    fw_task_wait (handle);
    free (handle);
}
#endif
