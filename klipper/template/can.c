/*
 * CAN FD to Serial for KLIPPER firmware
 * Copyright (C) 2020  Arsi <arsi@arsi.sk>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 * CAN driver implementation template
 */


#include "sched.h" // DECL_INIT
#include "board-generic/board/can_fd_support.h"
#include "can_fd_support.h"


static const uint8_t fdlen[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, /* 0 - 8 */
    12, 12, 12, 12, /* 9 - 12 */
    16, 16, 16, 16, /* 13 - 16 */
    20, 20, 20, 20, /* 17 - 20 */
    24, 24, 24, 24, /* 21 - 24 */
    32, 32, 32, 32, 32, 32, 32, 32, /* 25 - 32 */
    48, 48, 48, 48, 48, 48, 48, 48, /* 33 - 40 */
    48, 48, 48, 48, 48, 48, 48, 48, /* 41 - 48 */
    64, 64, 64, 64, 64, 64, 64, 64, /* 49 - 56 */
    64, 64, 64, 64, 64, 64, 64, 64}; /* 57 - 64 */

/**
 * Message length to CAN message length
 * @param len
 * @return 
 */
uint8_t can_len2len(uint8_t len) {
    if (len > 64)
        return 0xF;
    return fdlen[len];
}

static volatile bool init_ok = false;

#define can_new_message_in_rx_fifo_interrupt 1

/**
 * CAN irq handler
 */
void CAN0_HandlerIn(void) {
    //There will be CAN interrupt handling specific to the processor
    if (can_new_message_in_rx_fifo_interrupt) {
        sched_wake_tasks(); //IRQ wakes up the processor and you need to tell the scheduler to run tasks
        //handle interrupt
    }
}

DECL_ARMCM_IRQ(CAN0_HandlerIn, CAN0_IRQn);

/**
 * Initialization of the CAN subsystem for a given processor
 */
void can_init(void) {
    //init CAN subsystem
    //enable CAN filter - see CanFdToSerial code
    NVIC_EnableIRQ(CAN0_IRQn); //Enable CAN IRQ
}

DECL_INIT(can_init);

/**
 * Send message, called from klipper MCU code
 * @param ce
 * @param args
 */
void console_sendf(const struct command_encoder *ce, va_list args) {
    can_fd_frame outgoing_can_frame;
    outgoing_can_frame.can_id.id.destination_address = 0; //RPI address
    outgoing_can_frame.can_id.id.source_address = mcu_can_address; //MCU address
    can_command_encode_and_frame(&outgoing_can_frame, ce, args); //send only real message bytes
    outgoing_can_frame.can_id.id.msg_type = can_fd_msg_type_serial_data;
    //CAN_STACK_FLAG - device specific FLAG for CAN FD, it may not be necessary
    send_message_to_can(&mcan, outgoing_can_frame.can_id.value | CAN_STACK_FLAG, can_len2len(outgoing_can_frame.can_id.id.msglen), outgoing_can_frame.data);
}

/**
 * CAN RX task
 */
void can_rx_task(void) {
    uint_fast8_t pop_count;
    struct can_msg msg;
    can_fd_frame incomming_can_frame;

    //read message from CAN specific to the processor
    //in example to msg variable

    incomming_can_frame.can_id.value = msg.id;
    if (incomming_can_frame.can_id.id.msg_type == can_fd_msg_type_serial_data) {
        memcpy(incomming_can_frame.data, msg.data, incomming_can_frame.can_id.id.msglen); //copy msg data
        can_command_find_and_dispatch(&incomming_can_frame, &pop_count); //dispatch data to Klipper MCU code
    } else if (incomming_can_frame.can_id.id.msg_type == can_fd_msg_type_acknowledgeAnnounce) {
        // Sendet from  CanFdToSerial after it receives the can_fd_msg_type_announce
        // Change the status of the LED CanFdToSerial is connected
        init_ok = true;
    } else if (incomming_can_frame.can_id.id.msg_type == can_fd_msg_type_ping) {
        // Sendet from  CanFdToSerial every 5s
        if (!received_in_6s_window) {
            init_ok = false;
            // Change the status of the LED
            //CanFdToSerial is disconnected
        }
    }

    if (!init_ok) {
        if (run_every_second) {
            // Change the status of the LED
            //waitin for CanFdToSerial
            incomming_can_frame.can_id.id.msg_type = can_fd_msg_type_announce;
            incomming_can_frame.can_id.id.msglen = 0;
            incomming_can_frame.can_id.id.destination_address = 0;
            incomming_can_frame.can_id.id.source_address = mcu_can_address;
            send_message_to_can(&mcan, incomming_can_frame.can_id.value | CAN_STACK_FLAG, can_len2len(incomming_can_frame.can_id.id.msglen), incomming_can_frame.data);
        }

    }

}
DECL_TASK(can_rx_task);