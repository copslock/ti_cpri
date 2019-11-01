/*
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 */

#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include "sockutils.h"

#define error_msg printf

typedef struct sock_data {
    struct sockaddr_un addr;
    fd_set  readfds;
    int fd;
} sock_data_t;

int check_and_create_path (char *path)
{
    char *d = path;
    if (!d)
        return -1;

    while ((d = strchr(d + 1, '/'))) {
        *d = 0;
        if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {
            if (errno != EEXIST) {
                *d = '/';
                error_msg("can't create path %s (error: %s)",
                        path, strerror(errno));
                return -1;
            }
        }
        *d = '/';
    }
    return 0;
}

sock_h sock_open (sock_name_t *sock_name)
{
    sock_data_t *sd = 0;
    int retval = 0;

    if (!sock_name) {
        return 0;
    }

    sd = calloc (1, sizeof(sock_data_t));

    if (sock_name->type == sock_addr_e) {
        memcpy (&sd->addr, sock_name->s.addr, sizeof(struct sockaddr_un));
    } else {
        if (check_and_create_path(sock_name->s.name) < 0) {
            goto check_n_return;
        }
        sd->addr.sun_family = AF_UNIX;
        strncpy(sd->addr.sun_path, sock_name->s.name, UNIX_PATH_MAX);
    }

    sd->fd =  socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sd->fd < 0) {
        error_msg("can't open socket %s (error: %s)",
                  sd->addr.sun_path, strerror(errno));
        goto check_n_return;
    }

    unlink(sd->addr.sun_path);
    if (bind(sd->fd, (struct sockaddr *) &sd->addr, sizeof(struct sockaddr_un)) < 0) {
        error_msg("can't bind socket %s (error: %s)",
                  sd->addr.sun_path, strerror(errno));
        goto check_n_return;
    }

    FD_ZERO(&sd->readfds);
    FD_SET(sd->fd, &sd->readfds);

    retval = (int) sd;

check_n_return:
    if (!retval) {
        sock_close ((sock_h) &sd);
    }

    return ((sock_h) retval);
}

int sock_close (sock_h handle)
{
    sock_data_t *sd = (sock_data_t *) handle;

    if (!sd) {
        return -1;
    }

    if (sd->fd)
        close (sd->fd);
    free (sd);

    return 0;    
}

int sock_send (sock_h handle, const char *data, int length,
            sock_name_t *to)
{
    int fd;
    sock_data_t *sd = (sock_data_t *) handle;
    struct sockaddr_un to_addr;

    if (!to) {
        return -1;
    }

    if (to->type == sock_addr_e) {
        memcpy (&to_addr, to->s.addr, sizeof(struct sockaddr_un));
    } else {
        to_addr.sun_family = AF_UNIX;
        strncpy(to_addr.sun_path, to->s.name, UNIX_PATH_MAX);
    }

    if (sd) {
        fd = sd->fd;
    } else {
        fd =  socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd < 0) {
            error_msg("can't open socket %s (error: %s)",
                      to_addr.sun_path, strerror(errno));
            return -1;
        }
    }

    if (sendto (fd, data, length, 0, (struct sockaddr *) &to_addr,
                sizeof(struct sockaddr_un)) < 0) {
        error_msg("can't send data to %s (error: %s)",
                to_addr.sun_path, strerror(errno));
        return -1;

    }

    return 0;
}

int sock_wait (sock_h handle, int *size, struct timeval *timeout, int extern_fd)
{
    sock_data_t *sd = (sock_data_t *) handle;
    int retval;
    fd_set fds;

    if (!sd) {
        error_msg("invalid hanlde");
        return -1;
    }

    fds = sd->readfds;

    if (extern_fd != -1) {
        FD_SET(extern_fd, &fds);
    }

    retval = select(FD_SETSIZE, &fds, NULL, NULL, timeout);
    if (retval == -1) {
        error_msg("select failed for %s (error: %s)",
                sd->addr.sun_path, strerror(errno));
        return -1;
    }

    if ((extern_fd != -1) && (FD_ISSET(extern_fd, &fds))) {
        return 1;
    }

    if (!FD_ISSET(sd->fd, &fds)) {
        /* Wait timedout */
        return -2;
    }

    if (!retval) {
        return 0;
    }

    if (size != 0) {
        retval = ioctl(sd->fd, FIONREAD, size);
        if (retval == -1) {
            error_msg("can't read datagram size for %s (error: %s)",
                    sd->addr.sun_path, strerror(errno));
            return -1;
        }
    }

    return 0;
}

int sock_recv (sock_h handle, char *data, int length, sock_name_t *from)
{
    int size;
    sock_data_t *sd = (sock_data_t *) handle;
    socklen_t from_length = 0;

    if (!sd) {
        error_msg("invalid hanlde");
        return -1;
    }

    if (from) {
        if((from->type = sock_addr_e) && (from->s.addr))
            from_length = sizeof(struct sockaddr_un);
        else {
            error_msg("invalid from parameter");
            return -1;
        }
    }

    size = recvfrom(sd->fd, data, length, 0, (struct sockaddr *)((from_length) ? from->s.addr : NULL), &from_length);
    if (size < 1) {
        error_msg("can't read datagram from socket for %s (error: %s), size %d",
                sd->addr.sun_path, strerror(errno), size);
        return -1;
    }

    return size;
    
}

