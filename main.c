/*
 * CAN FD to Serial for KLIPPER firmware
 * Copyright (C) 2020  Arsi <arsi@arsi.sk>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 */

/* 
 * File:   main.c
 * Author: arsi
 *
 * Created on Streda, 2020, augusta 12, 11:37
 */

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <pty.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>
#include "klipper/generic/can_fd_support.h"
#include <sys/time.h>

// cd do klipper dir and:
//      ~/klippy-env/bin/python ./klippy/console.py /tmp/ttyCAN0MCU2 250000

static char portName[30];
static char portNameUpper[30];
static char ttyName[30];
static char mcuNr[20];


#define NUM_CAN_FILTERS 1

//original Klipper message defs
#define MESSAGE_POS_LEN 0
#define MESSAGE_POS_SEQ 1
#define MESSAGE_TRAILER_SIZE 3
#define MESSAGE_TRAILER_CRC  3
#define MESSAGE_TRAILER_SYNC 1
#define MESSAGE_SYNC 0x7E
#define MESSAGE_MIN 5
#define MESSAGE_MAX 64
#define MESSAGE_SEQ_MASK 0x0f
#define MESSAGE_DEST 0x10

static int sock;
pthread_t PxTh; //ping thread
pthread_t RxTh; //rx thread
pthread_t TxTh; //tx thread
pthread_t IxTh; // Inotify thread
static int Inotify;
static int threadexit = 0;
static int threadexitTx = 0;
static int threadexitIx = 0;
static int threadexitPx = 0;
static int running;
int watch;
int fd, sfd, res;
struct termios ti;

uint8_t need_sync;
int input_pos;
uint32_t bytes_read, bytes_invalid;
static uint8_t mcuAddress;
uint8_t input_buf[100];


static uint8_t encodeBuffer[100];
#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define EVENT_BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )
volatile int active;
char ev_buf[EVENT_BUF_LEN];

/**
 * Original Klipper CRC function
 * @param buf
 * @param len
 * @return 
 */
static uint16_t crc16_ccitt(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xffff;
    while (len--) {
        uint8_t data = *buf++;
        data ^= crc & 0xff;
        data ^= data << 4;
        crc = ((((uint16_t) data << 8) | (crc >> 8)) ^ (uint8_t) (data >> 4)
                ^ ((uint16_t) data << 3));
    }
    return crc;
}

/**
 * Restore received CAN MCU message to original Kliper format
 * @param buf
 * @param msglen
 * @param next_sequence
 */
void command_add_frame(uint8_t *buf, uint_fast8_t msglen, uint8_t next_sequence) {
    buf[MESSAGE_POS_LEN] = msglen;
    buf[MESSAGE_POS_SEQ] = next_sequence;
    uint16_t crc = crc16_ccitt(buf, msglen - MESSAGE_TRAILER_SIZE);
    buf[msglen - MESSAGE_TRAILER_CRC + 0] = crc >> 8;
    buf[msglen - MESSAGE_TRAILER_CRC + 1] = crc;
    buf[msglen - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
    buf[msglen - MESSAGE_TRAILER_SYNC + 1] = MESSAGE_SYNC;
}

/**
 * Open virtual serial port
 * @return 
 */
static int CanVport() {
    memset(&ti, 0, sizeof (ti));
    res = openpty(&fd, &sfd, NULL, &ti, NULL);
    if (res) {
        fprintf(stderr, "Error: openpty %d\n", res);
        return -1;
    }
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) {
        fprintf(stderr, "Error: fcntl getfl %d\n", flags);
        return -1;
    }
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    fcntl(sfd, F_SETFD, FD_CLOEXEC);
    char *tname = ttyname(sfd);
    printf("New ptyname %s linked to %s\n", tname, ttyName);

    // Create symlink to tty
    unlink(ttyName);

    res = symlink(tname, ttyName);
    if (res) {
        fprintf(stderr, "Error: symlink %d\n", res);
        return -1;
    }
    res = chmod(tname, 0660);
    if (res) {
        fprintf(stderr, "Error: chmod %d\n", res);
        return -1;
    }
    watch = inotify_add_watch(Inotify, ttyName, IN_OPEN | IN_CLOSE);

}

/**
 * Close virtual serial port
 */
static void CanVportClose() {
    int res;

    inotify_rm_watch(Inotify, watch);
    res = unlink(ttyName);
    if (res == 0) {
        printf("Unlink from %s\n", ttyName);
    }
}

/**
 * Original Klipper MCU code to find message in serial stream
 * Verify a buffer starts with a valid mcu message
 * @param need_sync
 * @param buf
 * @param buf_len
 * @return 
 */
static int check_message(uint8_t *need_sync, uint8_t *buf, int buf_len) {
    if (buf_len < MESSAGE_MIN)
        // Need more data
        return 0;
    if (*need_sync)
        goto error;
    uint8_t msglen = buf[MESSAGE_POS_LEN];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX)
        goto error;
    uint8_t msgseq = buf[MESSAGE_POS_SEQ];
    if ((msgseq & ~MESSAGE_SEQ_MASK) != MESSAGE_DEST)
        goto error;
    if (buf_len < msglen)
        // Need more data
        return 0;
    if (buf[msglen - MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC)
        goto error;
    uint16_t msgcrc = ((buf[msglen - MESSAGE_TRAILER_CRC] << 8)
            | (uint8_t) buf[msglen - MESSAGE_TRAILER_CRC + 1]);
    uint16_t crc = crc16_ccitt(buf, msglen - MESSAGE_TRAILER_SIZE);
    if (crc != msgcrc)
        goto error;
    return msglen;

error:
    ;
    // Discard bytes until next SYNC found
    uint8_t *next_sync = memchr(buf, MESSAGE_SYNC, buf_len);
    if (next_sync) {
        *need_sync = 0;
        return -(next_sync - buf + 1);
    }
    *need_sync = 1;
    return -buf_len;
}

/**
 * Copy klipper request message to CAN frame and send it
 * 
 */
static void handle_message(void) {
    can_fd_frame mcuCanFrame;
    struct canfd_frame frameOut;

    mcuCanFrame.can_id.id.msglen = input_buf[0] - 2 - 3; //message length - header and footer
    mcuCanFrame.can_id.id.destination_address = mcuAddress;
    mcuCanFrame.can_id.id.source_address = 0; //0=master
    mcuCanFrame.can_id.id.sequence = input_buf[1]; //copy message sequence
    mcuCanFrame.can_id.id.msg_type = can_fd_msg_type_serial_data;
    memcpy(frameOut.data, input_buf + 2, mcuCanFrame.can_id.id.msglen); //copy pure data to CAN frame
    frameOut.can_id = mcuCanFrame.can_id.value | CAN_EFF_FLAG; //set CAN id
    frameOut.len = mcuCanFrame.can_id.id.msglen;
    //send it to MCU
    if (write(sock, &frameOut, sizeof (struct canfd_frame)) != sizeof (struct canfd_frame)) {
        perror("Error sending command via CAN FD!");
    }

}

/**
 * Original Klipper MCU code to extract serial frame from serial stream
 * This is necessary because console.py sent a few chaotic bytes after connecting
 * and we need to find the beginning of the packet
 */
static void input_event(void) {
    //read  serial data from klipper
    int ret = read(fd, &input_buf[input_pos]
            , sizeof (input_buf) - input_pos);
    if (ret <= 0) {
        return;
    }
    input_pos += ret;
    for (;;) {
        ret = check_message(&need_sync, input_buf, input_pos);
        if (!ret)
            // Need more data
            return;
        if (ret > 0) {
            // Received a valid message
            //Extract pure data and send the message to MCU over CAN
            handle_message();
            bytes_read += ret;

        } else {
            // Skip bad data at beginning of input
            ret = -ret;
            bytes_invalid += ret;
        }
        input_pos -= ret;
        if (input_pos)
            memmove(input_buf, &input_buf[ret], input_pos);
    }
}

/**
 * Ping MCU thread
 * @param ptr
 * @return 
 */
void *CanPxThread(void *ptr) {
    can_fd_frame mcuCanPingFrame;
    //ping MCU every 5s
    struct canfd_frame frame;
    while (threadexitPx == 0) {
        sleep(1);
        if (threadexitPx != 0) {
            break;
        }
        sleep(1);
        if (threadexitPx != 0) {
            break;
        }
        sleep(1);
        if (threadexitPx != 0) {
            break;
        }
        sleep(1);
        if (threadexitPx != 0) {
            break;
        }
        sleep(1);
        if (threadexitPx != 0) {
            break;
        }
        mcuCanPingFrame.can_id.value = 0;
        mcuCanPingFrame.can_id.id.destination_address = mcuAddress;
        mcuCanPingFrame.can_id.id.source_address = 0;
        mcuCanPingFrame.can_id.id.msg_type = can_fd_msg_type_ping;
        mcuCanPingFrame.can_id.id.msglen = 0;
        frame.can_id = mcuCanPingFrame.can_id.value | CAN_EFF_FLAG;
        frame.len = 0;
        if (write(sock, &frame, sizeof (struct can_frame)) != sizeof (struct can_frame)) {
            perror("Error sending command via CAN FD!");
        }
    }

}

/**
 * Handle virtual serial port open/close
 * @param ptr
 * @return 
 */
void *CanIxThread(void *ptr) {
    while (threadexitIx == 0) {
        ssize_t ev = read(Inotify, ev_buf, EVENT_BUF_LEN);
        if (ev > 0) {
            for (char *p = ev_buf; p < ev_buf + ev;) {
                struct inotify_event *event = (struct inotify_event *) p;
                if (watch == event->wd) {
                    if (event->mask & IN_OPEN) {
                        printf("Port Open\n");
                        sleep(1);
                        active = 1;
                        // Send reset to MCU
                    } else if (event->mask & IN_CLOSE) {
                        printf("Port Close \n");
                        active = 0;
                    }
                    break;
                }
                p += EVENT_SIZE + event->len;
            }
        }
    }
}

/**
 * Handle incomming serial stream from Klipper
 * @param ptr
 * @return 
 */
void *CanTxThread(void *ptr) {
    while (threadexitTx == 0) {
        if (active == 1) {
            input_event();
        }
    }

}

/**
 * Handle received messages from MCU
 * @param ptr
 * @return 
 */
void *CanRxThread(void *ptr) {
    can_fd_frame mcuCanRxFrame;

    int nbytes;
    while (threadexit == 0) {
        struct canfd_frame frame;
        nbytes = read(sock, &frame, sizeof (struct canfd_frame));
        if (nbytes>-1) {
            //copy id from CAN frame
            mcuCanRxFrame.can_id.value = frame.can_id;
            //We have the CAN filter turned on, but for sure
            if (mcuCanRxFrame.can_id.id.msg_type == can_fd_msg_type_serial_data && mcuCanRxFrame.can_id.id.source_address == mcuAddress) {
                //Handle serial data from MCU
                if (active) {
                    //copy data from CAN frame at offset 2 and length from can id
                    memcpy(&encodeBuffer[2], frame.data, mcuCanRxFrame.can_id.id.msglen);
                    //Encode pure response data to Klipper format <len><sequence><data><crc><crc><end>
                    command_add_frame(encodeBuffer, mcuCanRxFrame.can_id.id.msglen + 2 + 3, mcuCanRxFrame.can_id.id.sequence);
                    //send data to Klipper
                    if (write(fd, encodeBuffer, mcuCanRxFrame.can_id.id.msglen + 2 + 3) == 0) {
                        perror("Error sending command via /tmp!");
                    }
                }
                //We have the CAN filter turned on, but for sure    
            } else if (mcuCanRxFrame.can_id.id.msg_type == can_fd_msg_type_announce && mcuCanRxFrame.can_id.id.source_address == mcuAddress) {
                //We will notify the MCU that we have joined
                //MCU Changes the flashing LED to steady light
                mcuCanRxFrame.can_id.value = 0;
                mcuCanRxFrame.can_id.id.destination_address = mcuAddress;
                mcuCanRxFrame.can_id.id.source_address = 0;
                mcuCanRxFrame.can_id.id.msg_type = can_fd_msg_type_acknowledgeAnnounce;
                mcuCanRxFrame.can_id.id.msglen = 0;
                frame.can_id = mcuCanRxFrame.can_id.value | CAN_EFF_FLAG;
                frame.len = 0;
                if (write(sock, &frame, sizeof (struct can_frame)) != sizeof (struct can_frame)) {
                    perror("Error sending command via CAN FD!");
                }
            }
        }

    }

}

/**
 * Init Can socket
 * @return 
 */
int CanSockInit(void) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter *rfilter;

    /* open socket */
    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        return ENOTSOCK;
    }

    addr.can_family = AF_CAN;

    strcpy(ifr.ifr_name, portName); //copy name of CAN port
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        return SIOCGIFINDEX;
    }
    addr.can_ifindex = ifr.ifr_ifindex;


    //Set filter We only want packets from one MCU
    rfilter = malloc(sizeof (struct can_filter) * NUM_CAN_FILTERS);
    if (!rfilter) {
        return ENOMEM;
    }
    rfilter[0].can_id = mcuAddress << 4 & 0xf0; //see can_fd_frame
    rfilter[0].can_mask = mcuAddress << 4 & 0xf0;

    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER,
            rfilter, NUM_CAN_FILTERS * sizeof (struct can_filter));
    free(rfilter);

    //set timeout
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*) &tv, sizeof tv);


    //enable TX blocking mode when linux can TX buffer is full
    //https://rtime.felk.cvut.cz/can/socketcan-qdisc-final.pdf
    int sndbuf = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof (sndbuf)) < 0)perror("setsockopt");
    perror("setsockopt");

    //Enable CAN FD
    int enable = 1;
    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof (enable));


    //bind
    if (bind(sock, (struct sockaddr *) &addr, sizeof (addr)) < 0) {
        perror("bind");
        return EIO;
    }

    // Inotify for port open/close
    Inotify = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (Inotify == -1) {
        fprintf(stderr, "unable to create inotify fd\n");
        return EINVAL;
    }

    // Create threads
    pthread_create(&RxTh, NULL, CanRxThread, (void *) &RxTh);
    pthread_create(&IxTh, NULL, CanIxThread, (void *) &IxTh);
    pthread_create(&PxTh, NULL, CanPxThread, (void *) &PxTh);
    pthread_create(&TxTh, NULL, CanTxThread, NULL);
    return 0;
}

/**
 * Free fesources
 */
void CanSockClose() {
    threadexitIx = 1;
    threadexit = 1;
    threadexitTx = 1;
    threadexitPx = 1;
    close(sock);
    CanVportClose();
    pthread_join(RxTh, NULL);
    pthread_join(TxTh, NULL);
    pthread_join(IxTh, NULL);
    pthread_join(PxTh, NULL);
}

/**
 * cleanup_handler
 * @param signo
 */
static void cleanup_handler(int signo) {
    if (signo == SIGINT) {
        printf("Received SIGINT\n");
        running = 0;

        CanSockClose();
    }
}

/**
 * Get index of substring
 * @param text
 * @param find
 * @param offset
 * @return 
 */
int strpos(char *text, char *find, int offset) {
    char haystack[strlen(text)];
    strncpy(haystack, text + offset, strlen(text) - offset);
    char *p = strstr(haystack, find);
    if (p)
        return p - haystack + offset;
    return -1;
}

/**
 * Print help
 */
void printHelp(void) {
    printf("****************************************************************\n\r");
    printf("*********************** CAN FD to Serial ***********************\n\r");
    printf("****************************************************************\n\r");
    printf("************************* (c)2020 Arsi *************************\n\r");
    printf("****************************************************************\n\r");
    printf("*                    Usage: canFdToSerial canxmcuxfd - CAN FD  *\n\r");
    printf("*                    Usage: canFdToSerial canxmcuxsd - CAN 2.0 *\n\r");
    printf("*                  Example: canFdToSerial can0mcu1fd           *\n\r");
    printf("* Use from systemd service: canfdtoserial@can0mcu1fd           *\n\r");
    printf("*               Currently only CAN FD is supported!            *\n\r");
    printf("****************************************************************\n\r");
}

/**
 * Main
 * @param argc
 * @param argv
 * @return 
 */
int main(int argc, char** argv) {
    if (argc == 1 || argc > 2) {
        printf("Incorrectly entered parameters!\n\r");
        printHelp();
        return 1;
    } else {
        int pos = strpos(argv[1], "mcu", 0);
        if (pos < 4) {
            printf("Incorrectly entered parameters!\n\r");
            printHelp();
            return 1;
        }
        int posFd = strpos(argv[1], "fd", 0);
        if (posFd < 8) {
            printf("Incorrectly entered parameters!\n\r");
            printHelp();
            return 1;
        }
        //        portName = malloc(pos);
        //        mcuNr = malloc(posFd - pos - 3);
        strncpy(portName, argv[1], pos);
        strncpy(portNameUpper, portName, sizeof (portName));
        strncpy(mcuNr, argv[1] + pos + 3, posFd - pos - 3);
        mcuAddress = atoi(mcuNr);
        char *s = portNameUpper;
        while (*s) {
            *s = toupper((unsigned char) *s);
            s++;
        }
        sprintf(ttyName, "/tmp/tty%sMCU%u", portNameUpper, mcuAddress);
        printf("****************************************************************\n\r");
        printf("*********************** CAN FD to Serial ***********************\n\r");
        printf("****************************************************************\n\r");
        printf("************************* (c)2020 Arsi *************************\n\r");
        printf("****************************************************************\n\r");
        printf("     CAN port: %s\n\r", portName);
        printf("  Virtual tty: %s\n\r", ttyName);
        printf("  MCU address: %u\n\r", mcuAddress);
        printf("****************************************************************\n\r");
        static char ipDown[100];
        static char ipUp[200];
        static char lock[200];
        //configure the CAN interface only at the first start on the given port
        sprintf(lock, "/run/lock/%s.lock", portName); //use tmfs
        if (access(lock, F_OK) != -1) {
            printf("CAN interface init skiped, the previous instance configured it..\n\r");
        } else {
            printf("CAN interface init..\n\r");
            sprintf(ipDown, "ip link set %s down", portName);
            sprintf(ipUp, "ip link set %s up type can bitrate 1000000   dbitrate 1000000 restart-ms 1000 berr-reporting on fd on", portName);
            printf("%s\n\r", ipDown);
            system(ipDown);
            printf("%s\n\r", ipUp);
            system(ipUp);
            int fd2 = open(lock, O_RDWR | O_CREAT, 0770); //create init lock for this CAN port
            if (fd2 != -1) {
                close(fd2);
            }
        }
    }
    if (signal(SIGINT, cleanup_handler) == SIG_ERR) {
        fprintf(stderr, "Can't catch SIGINT\n");
    } else {
        running = 1;
    }
    CanSockInit();
    CanVport();
    while (running) {
        usleep(3000000); /* Delay 3s before next loop */
    }
    return (EXIT_SUCCESS);
}

