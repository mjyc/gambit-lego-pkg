/****************************************************************************************
GAME-PLAYING ROBOT ARM
INTEL LABS SEATTLE

DYNAMIXEL CONTROL LIBRARY
B. Mayton <bmayton@bdm.cc>
March 2010

dynamixel/packet.h

This file defines classes that represent control packets that can be sent to the
Dynamixel units that comprise the wrist and gripper DOFs on the arm, and the status
packets that can be received in response.
****************************************************************************************/

#ifndef __dynamixel_packet_h__
#define __dynamixel_packet_h__

#include <stdint.h>
#include <string.h>
#include <assert.h>

namespace dynamixel {

// Register addresses (RAM area)
static const uint8_t REG_TORQUE_ENABLE = 24;
static const uint8_t REG_LED = 25;
static const uint8_t REG_CW_COMPLIANCE_MARGIN = 26;
static const uint8_t REG_CCW_COMPLIANCE_MARGIN = 27;
static const uint8_t REG_CW_COMPLIANCE_SLOPE = 28;
static const uint8_t REG_CCW_COMPLIANCE_SLOPE = 29;
static const uint8_t REG_GOAL_POSITION = 30;
static const uint8_t REG_MOVING_SPEED = 32;
static const uint8_t REG_TORQUE_LIMIT = 34;
static const uint8_t REG_PRESENT_POSITION = 36;
static const uint8_t REG_PRESENT_SPEED = 38;
static const uint8_t REG_PRESENT_LOAD = 40;
static const uint8_t REG_PRESENT_VOLTAGE = 42;
static const uint8_t REG_PRESENT_TEMPERATURE = 43;
static const uint8_t REG_REGISTERED_INSTR = 44;
static const uint8_t REG_MOVING = 46;
static const uint8_t REG_LOCK = 47;
static const uint8_t REG_PUNCH = 48;

/**
 * Packet to send to the Dynamixel units. See page 16 of the RX-28 manual.
 */
class InstructionPacket {
    public:

    static const unsigned int MAX_PARAMETERS = 253;
    static const unsigned int MAX_LENGTH = 259;
    static const uint8_t ID_BROADCAST = 0xFE;

    // Possible instructions
    static const uint8_t CMD_PING = 0x01;
    static const uint8_t CMD_READ = 0x02;
    static const uint8_t CMD_WRITE = 0x03;
    static const uint8_t CMD_REG_WRITE = 0x04;
    static const uint8_t CMD_ACTION = 0x05;
    static const uint8_t CMD_RESET = 0x06;
    static const uint8_t CMD_SYNC_WRITE = 0x83;

    // Exceptions
    static const int EXC_PARAMETERS_FULL = -1;

    InstructionPacket(uint8_t dest_id, uint8_t instr);
    ~InstructionPacket();

    size_t get_binary(uint8_t *buf);
    void debug_print();
    
    inline void add_byte(uint8_t val) {
        if(num_parameters == MAX_PARAMETERS)
            throw EXC_PARAMETERS_FULL;
        parameters[num_parameters++] = val;
    }

    inline void add_short(uint16_t val) {
        if(num_parameters == MAX_PARAMETERS-1)
            throw EXC_PARAMETERS_FULL;
        parameters[num_parameters++] = val & 0xFF;
        parameters[num_parameters++] = (val >> 8) & 0xFF;
    }

    protected:
    unsigned int num_parameters;
    uint8_t id;
    uint8_t instruction;
    uint8_t *parameters;
};

/**
 * Packets received from the Dynamixel units.  See page 18 of the RX-28 manual.
 */
class StatusPacket {
    public:
    
    // Error field bits
    static const uint8_t ERR_INSTRUCTION = 1<<6;
    static const uint8_t ERR_OVERLOAD = 1<<5;
    static const uint8_t ERR_CHECKSUM = 1<<4;
    static const uint8_t ERR_RANGE = 1<<3;
    static const uint8_t ERR_OVERHEAT = 1<<2;
    static const uint8_t ERR_ANGLE_LIMIT = 1<<1;
    static const uint8_t ERR_VOLTAGE = 1<<0;

    // Exceptions (can occur on decode)
    static const int EXC_MALFORMED_PACKET = -1;
    static const int EXC_BAD_CHECKSUM = -2;


    StatusPacket(uint8_t *buf, size_t length);
    ~StatusPacket();

    inline unsigned int get_num_parameters() {
        return num_parameters;
    }

    inline uint16_t read_short() {
        assert(param_ptr + 2 <= num_parameters && parameters != NULL);
        uint16_t value = parameters[param_ptr] | (parameters[param_ptr + 1] << 8);
        param_ptr += 2;
        return value;
    }

    inline uint8_t read_byte() {
        assert(param_ptr + 1 <= num_parameters && parameters != NULL);
        uint8_t value = parameters[param_ptr];
        param_ptr += 1;
        return value;
    }

    void debug_error();

    inline uint8_t get_error() { return error; }

    protected:
    unsigned int num_parameters;
    uint8_t *parameters;
    uint8_t error;
    unsigned int param_ptr;

    bool validate_checksum(uint8_t *buf);
};

}

#endif // __dynamixel_packet_h__
