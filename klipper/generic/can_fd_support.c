/*
 * CAN FD to Serial for KLIPPER firmware
 * Copyright (C) 2020  Arsi <arsi@arsi.sk>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 */

#include "can_fd_support.h"
#include "autoconf.h" //
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_byte
#include "board/irq.h" // irq_poll
#include "board/misc.h" // console_sendf
#include "command.h" // DECL_CONSTANT_STR
#include "sched.h" // DECL_INIT
#include <string.h>
#include <sys/unistd.h>
#include "gpio.h"
#include "generic/misc.h"
#include "board/pgm.h"
#include <stdarg.h>

static uint8_t next_sequence = MESSAGE_DEST;


/****************************************************************
 * Binary message parsing for CAN FD. Look at the original Command.c
 * Klipper's original command API is written for a serial stream.
 * This means that it adds a message header and a trailer to each command.
 * <2 byte header><command data><3 byte trailer>
 * CAN FD uses packet transmission, which means that one command is one data packet.
 * And we send the parameters from the original header (message length and sequence number) in the CAN ID
 * <CAN ID><command data>
 * This section contains modified functions to support CAN FD
 * 
 * 
 ****************************************************************/

// Encode an integer as a variable length quantity (vlq)

static uint8_t *encode_int(uint8_t *p, uint32_t v) {
    int32_t sv = v;
    if (sv < (3L << 5) && sv >= -(1L << 5)) goto f4;
    if (sv < (3L << 12) && sv >= -(1L << 12)) goto f3;
    if (sv < (3L << 19) && sv >= -(1L << 19)) goto f2;
    if (sv < (3L << 26) && sv >= -(1L << 26)) goto f1;
    *p++ = (v >> 28) | 0x80;
f1:
    *p++ = ((v >> 21) & 0x7f) | 0x80;
f2:
    *p++ = ((v >> 14) & 0x7f) | 0x80;
f3:
    *p++ = ((v >> 7) & 0x7f) | 0x80;
f4:
    *p++ = v & 0x7f;
    return p;
}

uint_fast8_t can_command_encodef(uint8_t *buf, const struct command_encoder *ce, va_list args) {
    uint_fast8_t max_size = READP(ce->max_size);
    if (max_size == 5)
        // Ack/Nak message
        return 0;
    uint8_t *p = &buf[0];
    uint8_t *maxend = &p[max_size];
    uint_fast8_t num_params = READP(ce->num_params);
    const uint8_t *param_types = READP(ce->param_types);
    *p++ = READP(ce->msg_id);
    while (num_params--) {
        if (p > maxend)
            goto error;
        uint_fast8_t t = READP(*param_types);
        param_types++;
        uint32_t v;
        switch (t) {
            case PT_uint32:
            case PT_int32:
            case PT_uint16:
            case PT_int16:
            case PT_byte:
                if (sizeof (v) > sizeof (int) && t >= PT_uint16)
                    if (t == PT_int16)
                        v = (int32_t) va_arg(args, int);
                    else
                        v = va_arg(args, unsigned int);
                else
                    v = va_arg(args, uint32_t);
                p = encode_int(p, v);
                break;
            case PT_string:
            {
                uint8_t *s = va_arg(args, uint8_t*), *lenp = p++;
                while (*s && p < maxend)
                    *p++ = *s++;
                *lenp = p - lenp - 1;
                break;
            }
            case PT_progmem_buffer:
            case PT_buffer:
            {
                v = va_arg(args, int);
                if (v > maxend - p)
                    v = maxend - p;
                *p++ = v;
                uint8_t *s = va_arg(args, uint8_t*);
                if (t == PT_progmem_buffer)
                    memcpy_P(p, s, v);
                else
                    memcpy(p, s, v);
                p += v;
                break;
            }
            default:
                goto error;
        }
    }
    return p - buf;
error:
    shutdown("Message encode error");
}

// Add header and trailer bytes to a message block

void can_command_add_frame(can_fd_frame *can_frame, uint_fast8_t msglen) {
    can_frame->can_id.id.msglen = msglen;
    can_frame->can_id.id.sequence = next_sequence;
}

// Encode a message and then add a message block frame around it

uint_fast8_t can_command_encode_and_frame(can_fd_frame *can_frame, const struct command_encoder *ce, va_list args) {
    uint_fast8_t msglen = can_command_encodef(can_frame->data, ce, args);
    can_command_add_frame(can_frame, msglen);
    return msglen;
}


// Find the command handler associated with a command

static const struct command_parser * command_lookup_parser(uint_fast8_t cmdid) {
    if (!cmdid || cmdid >= READP(command_index_size))
        shutdown("Invalid command");
    return &command_index[cmdid];
}


// Dispatch all the commands found in a message block

void can_command_dispatch(uint8_t *buf, uint_fast8_t msglen) {
    uint8_t *p = &buf[0];
    uint8_t *msgend = &buf[msglen ];
    while (p < msgend) {
        uint_fast8_t cmdid = *p++;
        const struct command_parser *cp = command_lookup_parser(cmdid);
        uint32_t args[READP(cp->num_args)];
        p = command_parsef(p, msgend, cp, args);
        if (sched_is_shutdown() && !(READP(cp->flags) & HF_IN_SHUTDOWN)) {
            sched_report_shutdown();
            continue;
        }
        irq_poll();
        void (*func)(uint32_t*) = READP(cp->func);
        func(args);
    }
}

// Empty message (for ack/nak transmission)
const struct command_encoder encode_acknak_can PROGMEM = {
    .max_size = MESSAGE_MIN,
};

enum {
    CF_NEED_SYNC = 1 << 0, CF_NEED_VALID = 1 << 1
};
// Find the next complete message block
//

/**
 * Find the next complete message block
 * CAN FD only check the message
 * @param buf
 * @param buf_len
 * @param pop_count
 * @return 
 */
int_fast8_t can_command_find_block(uint8_t *buf, uint_fast8_t buf_len, uint_fast8_t sequence, uint_fast8_t *pop_count) {
    *pop_count = buf_len;
    next_sequence = ((sequence + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
    return 1;

nak:
    command_sendf(&encode_acknak_can);
    return -1;
}

// 

/**
 * Find a message block and then dispatch all the commands in it
 * CAN FD version
 * @param buf
 * @param buf_len
 * @param sequence
 * @param pop_count
 * @return 
 */
int_fast8_t can_command_find_and_dispatch(can_fd_frame *can_frame, uint_fast8_t *pop_count) {
    int_fast8_t ret = can_command_find_block(can_frame->data, can_frame->can_id.id.msglen, can_frame->can_id.id.sequence, pop_count);
    if (ret > 0) {
        can_command_dispatch(can_frame->data, *pop_count);
        command_send_ack(); //send emty CAN message as ACK
    }
    return ret;
}


