/*
 * CAN FD to Serial for KLIPPER firmware
 * Copyright (C) 2020  Arsi <arsi@arsi.sk>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 * 
 */

/* 
 * File:   can_fd_support.h
 * Author: arsi
 *
 * Created on Utorok, 2020, augusta 11, 9:06
 */

#ifndef CAN_FD_SUPPORT_H
#define CAN_FD_SUPPORT_H

#include <stdint.h> // uint32_t
#include <stdbool.h> // uint32_t
#include <stdarg.h>
#include "command.h" // DECL_CONSTANT_STR


#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        uint8_t msglen;
        uint8_t sequence;
        uint8_t data[64 + 3]; //data + word CRC + byte MESSAGE_SYNC
    } can_fd_command_buffer_emu;

    enum can_fd_msg_type {
        can_fd_msg_type_serial_data = 0b000, //MCU protocol data
        can_fd_msg_type_announce = 0b010, //After switching on, the MCU sends an announce until confirmation and the green LED flashes
        can_fd_msg_type_acknowledgeAnnounce = 0b011, //Confirmation from the serial bridge, the MCU lights up the green LED.
        can_fd_msg_type_ping = 0b100, //The serial bridge pings the MCU once every X seconds,
        //if the MCU does not receive a ping at that time, it returns to the announce step
        //This is the detection of an unconnected serial port.
        //The connection to the MCU is also controlled by Klipper,
        //but if Klipper is turned off, it will be good to know that the transmission between the MCU and the Serial Port is online.
        can_fd_msg_type_update_firmware = 0b101, //Command to start the bootloader, the command sends the firmware update tool
        can_fd_msg_type_multipart_serial_data = 0b110, //Multipart MCU protocol data, countdown counter in msg_part_counter, up to 4 parts (256 bytes) - same sequence
        can_fd_msg_type_unused = 0b111, //unused

    };

    typedef struct {

        union {

            struct can_fd_frame_id { //message ID = 29-bits
                uint32_t destination_address : 4; // 0-15
                uint32_t source_address : 4; //1-byte - 0-15
                uint32_t msglen : 8; //2byte - current message size,
                //CAN FD only supports specified message lengths, so we need to know the real length.
                uint32_t sequence : 8; //3-byte - Klipper message sequence
                uint32_t msg_type : 3; //can_fd_msg_type
                uint32_t msg_part_counter : 2; //Multipart MCU protocol data, countdown counter up to 4 pars (256 bytes)
                //Ready for the future, not currently implemented
                uint32_t can_flag1 : 1; //CAN stack flags
                uint32_t can_flag2 : 1; //CAN stack flags
                uint32_t can_flag3 : 1; //4byte - CAN stack flags
                //The CAN protocol has an integrated CRC, so it is not necessary to transmit it. 
                //It will be added to the serial stream by the CANtoSerial application on the RPI side
            } id;

            uint32_t value;
        } can_id;
        uint8_t data[64];
    } can_fd_frame;

    typedef struct {
        union {
            struct can_frame_id { //message ID = 11-bits
                uint32_t response : 1; // 0- Request from Klipper, 1-Response from MCU
                uint32_t muc_id : 3; //0-7 mcu_id
                uint32_t msg_segment : 3; //0-7 MsgSize/8=number of segments, segment with the highest number 
                //contains bytes 0-7 and so on, segment 0 contains the remaining number of bytes, 
                //their sum calculates the real length of the message
                uint32_t sequence : 4; //from klipper doc The sequence byte contains a 4 bit sequence number
                //in the low-order bits and the high-order bits always contain 0x10 (the high-order bits are reserved for future use). 
            } id;
            //The CAN protocol has an integrated CRC, so it is not necessary to transmit it. 
            //It will be added to the serial stream by the CANtoSerial application on the RPI side

            uint32_t value;
        } can_id;
        uint8_t data[8];
    } can_frame;

    uint_fast8_t can_command_encodef(uint8_t *buf, const struct command_encoder *ce, va_list args);

    void can_command_dispatch(uint8_t *buf, uint_fast8_t msglen);

    int_fast8_t can_command_find_block(uint8_t *buf, uint_fast8_t buf_len, uint_fast8_t sequence, uint_fast8_t *pop_count);

    //entry point for incomming messages
    int_fast8_t can_command_find_and_dispatch(can_fd_frame *can_frame, uint_fast8_t *pop_count);

    void can_command_add_frame(can_fd_frame *can_frame, uint_fast8_t msglen);

    //entry point for outgoing messages
    uint_fast8_t can_command_encode_and_frame(can_fd_frame *can_frame, const struct command_encoder *ce, va_list args);


#ifdef __cplusplus
}
#endif

#endif /* CAN_FD_SUPPORT_H */


