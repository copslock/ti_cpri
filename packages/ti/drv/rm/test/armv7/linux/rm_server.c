/*
 * Copyright (C) 2013-2015 Texas Instruments Incorporated - http://www.ti.com/
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
 */

/* 
 * Shut off: remark #880-D: parameter "appTransport" was never referenced
 *
 * This is better than removing the argument since removal would break
 * backwards compatibility
 */
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
 * warning: unused parameter ‘appTransport’ [-Wunused-parameter]
 */
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/* Standard includes */
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <signal.h>
#include <fcntl.h>
#include <getopt.h>
#include <unistd.h>
#include <grp.h>

#include <libdaemon/daemon.h>

/* Socket Includes */
#include "serverlogutil.h"
#include "sockutils.h"

/* RM includes */
#include <ti/drv/rm/rm_server_if.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>

#define RMSERVER_DAEMON_PID_FILE_NAME "/var/run/rmServer/pid"

#define MAX_LOG_LEVEL LOG_DEBUG

/* logging errors */
#define error_msg(...) rmsrv_log(LOG_ERR, __func__, __FILE__, __LINE__, __VA_ARGS__);
/* logging warnings */
#define warning_msg(...) rmsrv_log(LOG_WARNING, __func__, __FILE__, __LINE__, __VA_ARGS__);
/* logging information */
#define info_msg(...) rmsrv_log(LOG_INFO, __func__, __FILE__, __LINE__, __VA_ARGS__);
/* logging debug information */
#define debug_msg(...) rmsrv_log(LOG_DEBUG, __func__, __FILE__, __LINE__, __VA_ARGS__);

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)            \
    if (resultVal != checkVal) {                                          \
        char errorMsgToPrint[] = printMsg;                                \
        error_msg("%s : ", rmInstName);                                      \
        error_msg("%s with error code : %d\n", errorMsgToPrint, resultVal);  \
        exit(EXIT_FAILURE);                                               \
    }

#define LOG_APPEND(x)                                      \
    do {                                                   \
        int len;                                           \
        len = MAX_PRE_LOG_BUF_LEN - strlen(rmsrv_log_buf); \
        if (len > 0) {                                     \
            strncat(rmsrv_log_buf, x, len);                \
        }                                                  \
        else {                                             \
            return;                                        \
        }                                                  \
    } while(0)

/* RM registered transport mapping structure */
typedef struct trans_map_entry_s {
    /* Registered RM transport handle */
    Rm_TransportHandle        trans_handle;
    /* Remote socket tied to the transport handle */
    sock_name_t              *remote_sock;
    /* Next entry in the transport map */
    struct trans_map_entry_s *n;
} trans_map_entry_t;

/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* RM Server instance name (must match with RM Global Resource List (GRL) and policies */
char           server_name[RM_NAME_MAX_CHARS] = "RM_Server";

Rm_Handle      server_h;

sock_h         server_sock = NULL;

rmserver_cfg_t rmsrv_cfg;

char           rmsrv_log_buf[MAX_PRE_LOG_BUF_LEN];

/**********************************************************************
 ********************** External Variables ****************************
 **********************************************************************/

extern int   optind;
extern char *optarg;

/**********************************************************************
 ************************** Server Functions **************************
 **********************************************************************/

/* RM Server logging utility (need to move it to another file) */
void rmsrv_log(const int loglevel, const char* functionName, const char* fileName, const int lineNo, const char* format, ...)
{
    int         len;
    char        lineno_a[32];
    va_list     args;
    struct stat fbuf;

    if (loglevel <= MAX_LOG_LEVEL) {
        snprintf(lineno_a, sizeof(lineno_a), "%d", lineNo);

        rmsrv_log_buf[0] = 0;

        LOG_APPEND(fileName);
        LOG_APPEND(":");
        LOG_APPEND(lineno_a);
        LOG_APPEND(":");
        LOG_APPEND(functionName);
        LOG_APPEND(":");

        len = MAX_PRE_LOG_BUF_LEN - strlen(rmsrv_log_buf);
        if (len <= 0) {
            return;
        }

        va_start(args, format);
        vsnprintf(&rmsrv_log_buf[strlen(rmsrv_log_buf)], len, format, args);
        va_end(args);

        /* logfile reset if going over max_len */
        if (fstat(fileno(rmsrv_cfg.logfile_p), &fbuf) == 0) {
            if ((fbuf.st_size + ((int)strlen(rmsrv_log_buf))) > rmsrv_cfg.logfile_max_len) {
                freopen(RMSERVER_DAEMON_LOG_FILE_NAME, "w+", rmsrv_cfg.logfile_p);
            }
        }

        fprintf(rmsrv_cfg.logfile_p, "%s", rmsrv_log_buf);
        fflush(rmsrv_cfg.logfile_p);
    }
}

Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rm_pkt = NULL;

    rm_pkt = calloc(1, sizeof(*rm_pkt));
    if (!rm_pkt) {
        error_msg("Failed to malloc RM packet (err: %s)\n",
                  strerror(errno));
        return (NULL);
    }
    rm_pkt->pktLenBytes = pktSize;
    *pktHandle = rm_pkt;

    return(rm_pkt);
}

void transportFree(Rm_Packet *rm_pkt)
{
    if (rm_pkt) {
        free(rm_pkt);
    }
}

int32_t transportSend (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    sock_name_t *client_sock_name = (sock_name_t *)appTransport;
    Rm_Packet   *rm_pkt = (Rm_Packet *)pktHandle;
    
    if (sock_send(server_sock, (char *)rm_pkt, (int) rm_pkt->pktLenBytes, client_sock_name)) {
        error_msg("Failed to send RM packet\n");
    }
    else {
        /* Print resources after sending response */
        Rm_resourceStatus(server_h, 1);
    }

    transportFree(rm_pkt);
    return (0);
}

int rm_server_run(void *grl, void *policy, void *lin_dtb, int is_daemon,
                  char *group_name)
{
    Rm_InitCfg          rm_init_cfg;
    Rm_TransportCfg     rm_trans_cfg;
    int32_t             rm_result;
    trans_map_entry_t  *trans_map = NULL;
    trans_map_entry_t  *new_map_entry;
    trans_map_entry_t  *map_index;
    int                 retval;
    int                 length = 0;
    sock_name_t         serv_sock_name;
    struct group       *group_data;
    int                 group_valid = 0;
    gid_t               serv_group_id = 0;
    sock_name_t         client_sock_addr;
    Rm_Packet          *rm_pkt = NULL;
    char                pkt_src[RM_NAME_MAX_CHARS];
    struct sockaddr_un  client_addr; 
    int                 signal_fd = -1;
    char                rm_socket_name[] = RM_SERVER_SOCKET_NAME;

    rmsrv_cfg.logfile_p = fopen(RMSERVER_DAEMON_LOG_FILE_NAME, "w+");
    if (!rmsrv_cfg.logfile_p) {
        printf("Error in opening log file %s (%s)", RMSERVER_DAEMON_LOG_FILE_NAME, strerror(errno));
    }

    debug_msg("Starting RM server\n");
    
    /* Create the Server instance */
    memset(&rm_init_cfg, 0, sizeof(rm_init_cfg));
    rm_init_cfg.instName = server_name;
    rm_init_cfg.instType = Rm_instType_SERVER;
    rm_init_cfg.instCfg.serverCfg.globalResourceList = grl;
    rm_init_cfg.instCfg.serverCfg.linuxDtb = lin_dtb;
    rm_init_cfg.instCfg.serverCfg.globalPolicy = policy;
    server_h = Rm_init(&rm_init_cfg, &rm_result);
    ERROR_CHECK(RM_OK, rm_result, server_name, "Initialization failed\n");

    debug_msg("RM Server initialized with name: %s\n", server_name);

    Rm_resourceStatus(server_h, 1);

    if (group_name) {
        /* Get the group ID */
        errno = 0;
        group_data = getgrnam(group_name);
        if (group_data) {
            group_valid = 1;
            serv_group_id = group_data->gr_gid;
        } else {
            if (errno) {
                error_msg("Received error: \"%s\" when attempting to retrieve "
                          "group database information for group: %s\n",
                          strerror(errno), group_name);
                return(-1);
            } else {
                error_msg("Group database information does not exist for "
                          "group with name %s - "
                          "Not setting group permissions\n", group_name);
            }
        }
    }
    serv_sock_name.type = sock_name_e;
    serv_sock_name.s.name = rm_socket_name;
    server_sock = sock_open(&serv_sock_name, group_valid, serv_group_id);
    if (!server_sock) {
        error_msg("Error when opening socket %s\n", rm_socket_name);
        return -1;
    }

    if (is_daemon){
        signal_fd = daemon_signal_fd();
    }

    while(1) {
        info_msg("Waiting for messages from Clients\n");
        retval = sock_wait(server_sock, &length, NULL, signal_fd);
        if (retval < 0) {
            error_msg("Error in reading from socket\n");
            goto loop_continue;
        }

        if (length < ((int)sizeof(rm_pkt))) {
            error_msg("invalid RM message length %d\n", length);
            goto loop_continue;
        }
        rm_pkt = calloc(1, length);
        if (!rm_pkt) {
            error_msg("can't malloc for recv'd RM message (err: %s)\n",
                      strerror(errno));
            goto loop_continue;
        }

        client_sock_addr.type = sock_addr_e;
        client_sock_addr.s.addr = &client_addr;
        retval = sock_recv(server_sock, (char *)rm_pkt, length, &client_sock_addr);
        if (retval != length) {
            error_msg("recv RM pkt failed from socket, received = %d, expected = %d\n",
                      retval, length);
            goto loop_continue;
        }

        info_msg("Received RM pkt of size %d bytes from socket %s\n", length, client_sock_addr.s.addr->sun_path);
        if (Rm_receiveGetPktSrcName(rm_pkt, &pkt_src[0], RM_NAME_MAX_CHARS) == RM_OK) {
            info_msg("    RM pkt originated from %s instance\n", &pkt_src[0]);
        }
        if (Rm_receiveGetPktServiceSrcName(rm_pkt, &pkt_src[0], RM_NAME_MAX_CHARS) == RM_OK) {
            info_msg("    Service request within RM pkt originated from %s instance\n", &pkt_src[0]);
        }

        map_index = trans_map;
        while(map_index != NULL) {
            if (strncmp(map_index->remote_sock->s.addr->sun_path,
                        client_addr.sun_path, 
                        sizeof(client_addr.sun_path)) == 0) {
                break;
            }
            map_index = map_index->n;
        }

        if (!map_index) {
            new_map_entry = calloc(1, sizeof(*new_map_entry));
            new_map_entry->remote_sock = calloc(1, sizeof(sock_name_t));
            new_map_entry->remote_sock->s.addr = calloc(1, sizeof(struct sockaddr_un));
            new_map_entry->remote_sock->type = sock_addr_e;
            memcpy(new_map_entry->remote_sock->s.addr, &client_addr, sizeof(struct sockaddr_un));
                        
            /* Register the Client with the Server instance */
            rm_trans_cfg.rmHandle = server_h;
            rm_trans_cfg.appTransportHandle = (Rm_AppTransportHandle)new_map_entry->remote_sock;
            rm_trans_cfg.remoteInstType = Rm_instType_CLIENT;
            rm_trans_cfg.transportCallouts.rmAllocPkt = transportAlloc;
            rm_trans_cfg.transportCallouts.rmSendPkt = transportSend;
            new_map_entry->trans_handle = Rm_transportRegister(&rm_trans_cfg, &rm_result);

            new_map_entry->n = NULL;

            if (trans_map == NULL) {
                trans_map = new_map_entry;
            }
            else {
                map_index = trans_map;

                while(map_index->n != NULL) {
                    map_index = map_index->n;
                }
                map_index->n = new_map_entry;
            }

            map_index = new_map_entry;
        }

        /* Provide packet to RM Server for processing */       
        if ((rm_result = Rm_receivePacket(map_index->trans_handle, rm_pkt))) {
            error_msg("RM failed to process received packet: %d\n", rm_result);
        }
        
loop_continue:
        /* Cleanups */
        length = 0;
        transportFree(rm_pkt);
        memset(&client_sock_addr, 0, sizeof(sock_name_t));
        memset(&client_addr, 0, sizeof(struct sockaddr_un));
    }   
}

char *get_pid_file_name(void) {
    static char pid_file_name[] = RMSERVER_DAEMON_PID_FILE_NAME;
    return pid_file_name;
}

static void print_usage(char *appname)
{
    printf ("Usage: %s [OPTION]... [GRL] [POLICY]\n", appname);
    printf ("Run a resource manager server with the specified [GRL] and [POLICY]\n"
            "[GRL] and [POLICY] must be device tree blob (DTB) files\n"
            "Example: rmserver grl.dtb policy.dtb\n\n"
            "Configuration:\n"
            "  -s, --logsize MAXLOGSIZE  MAXLOGSIZE bytes will be written to the\n"
            "                            log file before the log is reset.  On\n"
            "                            reset, the log file will be wiped.\n"
            "\n"
            "                            The default for MAXLOGSIZE is 8MB if\n"
            "                            logsize is not specified\n"
            "Optional Input:\n"
            "  -l, --lindtb [LINUX_DTB]  Optionally, provide a Linux DTB file\n"
            "                            that RM will use to reserve resources\n"
            "                            for Linux.  The GRL must have the\n"
            "                            proper Linux DTB resource mappings\n"
            "                            for this feature to work\n"
            "  -g, --group [GROUP_NAME]  Group permission to assign to RM\n"
            "                            Server socket\n"
            "Miscellaneous:\n"
            "  -n, --nodaemon            do not daemonize, run in foreground\n"
            "  -k, --kill                kill the existing daemon\n"
            "  -h, --help                print this message\n"
            "\n"
            "rmserver will run as a daemon by default.\n\n");
}

int main(int argc, char *argv[])
{
    int          opt;
    int          daemonize = 1, kill = 0;
    int          fd;
    struct stat  file_stat;
    pid_t        pid;
    char        *grl_file = NULL;
    char        *policy_file = NULL;
    char        *lin_dtb_file = NULL;
    void        *grl = NULL;
    void        *policy = NULL;
    void        *lin_dtb = NULL;
    char        *group_name = NULL;
    
    const struct option longopts[] =
    {
        {"nodaemon", no_argument,       0, 'n'},
        {"kill",     no_argument,       0, 'k'},
        {"help",     no_argument,       0, 'h'},
        {"lindtb",   required_argument, 0, 'l'},
        {"logsize",  required_argument, 0, 's'},
        {"group",    required_argument, 0, 'g'},
        {0,          0,                 0,   0},
    };

    rmsrv_cfg.logfile_max_len = DEFAULT_LOG_LEN;

    while((opt = getopt_long(argc, argv, "nkhl:s:g:", longopts, NULL)) != -1) {
        switch (opt) {
            case 'n':
                daemonize = 0;
                break;
            case 'k':
                kill = 1;
                break;
            case 'l':
                lin_dtb_file = optarg;
                break;
            case 's':
                rmsrv_cfg.logfile_max_len = strtol(optarg, NULL, 0);
                break;
            case 'g':
                group_name = optarg;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                exit(EXIT_SUCCESS);
        }
    }

    if (!kill) {
        /* GRL and Policy must always be provided */
        if (optind == (argc - 2)) {
            /* First must be GRL */
            grl_file = argv[optind++];
            /* Second must be policy */
            policy_file = argv[optind++];  
        }
        else {
            printf("GRL or policy not provided\n\n");
            print_usage(argv[0]);
            exit(EXIT_FAILURE);
        }

        /* mmap the GRL */
        fd = open(grl_file, O_RDONLY);
        if (fd == -1) {
            printf("Error opening GRL: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        /* Obtain file size */
        if (fstat(fd, &file_stat) == -1) {
            printf("Error getting GRL size\n");
            exit(EXIT_FAILURE);
        }
        grl = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (grl == MAP_FAILED) {
            printf("mmap of GRL failed\n");
            exit(EXIT_FAILURE);
        }

        /* mmap the Global Policy */
        fd = open(policy_file, O_RDONLY);
        if (fd == -1) {
            printf("Error opening Global Policy: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        /* Obtain file size */
        if (fstat(fd, &file_stat) == -1) {
            printf("Error getting Global Policy size\n");
            exit(EXIT_FAILURE);
        }
        policy = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (policy == MAP_FAILED) {
            printf("mmap of Global Policy failed\n");
            exit(EXIT_FAILURE);
        }

        if (lin_dtb_file) {
            /* mmap the Linux DTB if it was provided */
            fd = open(lin_dtb_file, O_RDONLY);
            if (fd == -1) {
                printf("Error opening Linux DTB: %s\n", strerror(errno));
                exit(EXIT_FAILURE);
            }
            /* Obtain file size */
            if (fstat(fd, &file_stat) == -1) {
                printf("Error getting Linux DTB size\n");
                exit(EXIT_FAILURE);
            }
            lin_dtb = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
            if (lin_dtb == MAP_FAILED) {
                printf("mmap of Linux DTB failed\n");
                exit(EXIT_FAILURE);
            }
        }        
    }

    if (daemonize) {
        if (kill) {
            printf("Killing %s\n", argv[0]);
        }
        else {
            printf("Starting %s\n", argv[0]);
        }

        /* Reset signal handlers */
        if (daemon_reset_sigs(-1) < 0) {
            printf("Failed to reset all signal handlers: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }

        /* Unblock signals */
        if (daemon_unblock_sigs(-1) < 0) {
            printf("Failed to unblock all signals: %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }

        if (check_and_create_path(get_pid_file_name(), 0, 0) < 0) {
            printf("Failed to create pid file path: %s\n", get_pid_file_name());
            exit(EXIT_FAILURE);
        }

        /* set daemon id string */
        daemon_log_ident = daemon_ident_from_argv0(argv[0]);
        daemon_pid_file_proc = (daemon_pid_file_proc_t) get_pid_file_name;

        if (kill) {
            if (daemon_pid_file_kill_wait(SIGTERM, 5) < 0) {
                printf("Failed to kill daemon: %s\n", strerror(errno));
                exit(EXIT_FAILURE);
            }
            daemon_pid_file_remove();
            exit(EXIT_SUCCESS);
        }

        /* Single instance */
        if ((pid = daemon_pid_file_is_running()) >= 0) {
            printf("Daemon already running on PID file %u\n", pid);
            exit(EXIT_FAILURE);
        }

        if (daemon_retval_init() < 0) {
            printf("Failed to create pipe.\n");
            exit(EXIT_FAILURE);
        }

        /* Do the fork */
        if ((pid = daemon_fork()) < 0) {
            daemon_retval_done();
            printf("Error in daemon fork %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        } 
        else if (pid) { /* The parent */
            int ret;

            /* Wait for 20 seconds for the return value passed from the daemon process */
            if ((ret = daemon_retval_wait(20)) < 0) {
                printf("Could not receive return value from daemon process: %s\n", strerror(errno));
                return -1;
            }

            printf("Daemon returned %i as return value.\n", ret);
            return(ret);
        }

        /* Close FDs */
        if (daemon_close_all(-1) < 0) {
            printf("Failed to close all file descriptors: %s\n", strerror(errno));

            /* Send the error condition to the parent process */
            daemon_retval_send(1);
            goto close_n_exit;
        }

        /* Create the PID file */
        if (daemon_pid_file_create() < 0) {
            printf("Could not create PID file (%s).\n", strerror(errno));
            daemon_retval_send(2);
            goto close_n_exit;
        }

        /* Initialize signal handling */
        if (daemon_signal_init(SIGINT, SIGTERM, SIGQUIT, SIGHUP, 0) < 0) {
            printf("Could not register signal handlers (%s).\n", strerror(errno));
            daemon_retval_send(3);
            goto close_n_exit;
        }

        /* Send OK to parent process */
        daemon_retval_send(0);
    }
    else {
        if (kill) {
            printf("Can't kill undaemonized RM server\n");
            exit(EXIT_FAILURE);
        }
    }
    
    rm_server_run(grl, policy, lin_dtb, daemonize, group_name);

close_n_exit:
    printf("Exiting %s daemon\n", argv[0]);
    if (daemonize) {
        daemon_retval_send(255);
        daemon_signal_done();
        daemon_pid_file_remove();
    }

    return(0);
}

