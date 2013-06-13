/****************************************************************************************
GAME-PLAYING ROBOT ARM
INTEL LABS SEATTLE

DYNAMIXEL CONTROL LIBRARY
B. Mayton <bmayton@bdm.cc>
March 2010

packet.cpp

This file defines classes that represent control packets that can be sent to the
Dynamixel units that comprise the wrist and gripper DOFs on the arm, and the status
packets that can be received in response.
****************************************************************************************/

#include <dynamixel/packet.h>
#include <stdio.h>

namespace dynamixel {

InstructionPacket::InstructionPacket(uint8_t dest_id, uint8_t instr) {
    num_parameters = 0;
    id = dest_id;
    instruction = instr;
    parameters = new uint8_t[MAX_PARAMETERS];
}

InstructionPacket::~InstructionPacket() {
    delete[] parameters;
}

/** 
 * Copies the binary representation of the packet into a buffer.
 * @param buf pointer to the buffer where data will be copied; must be large enough to
 *        contain the packet
 * @return the number of bytes copied
 */
size_t InstructionPacket::get_binary(uint8_t *buf) {
    // Start sequence
    buf[0] = 0xFF;
    buf[1] = 0xFF;

    // Header
    buf[2] = id;
    buf[3] = num_parameters + 2;
    buf[4] = instruction;

    // Parameters
    memcpy(buf+5, parameters, num_parameters);

    // Checksum
    uint8_t checksum = 0;
    for(unsigned int i=0; i<3+num_parameters; i++)
        checksum += buf[i+2];
    buf[5+num_parameters] = ~checksum;

    return num_parameters + 6;
}

/**
 * Prints out the contents of the packet + checksum for debugging purposes.
 */
void InstructionPacket::debug_print() {
    uint8_t *buf;
    buf = new uint8_t[MAX_PARAMETERS + 10];
    
    size_t length = get_binary(buf);
    
    for(unsigned int i=0; i<length; i++) 
        printf("%02X ", buf[i]);
    printf("\n");

    delete[] buf;
}



StatusPacket::StatusPacket(uint8_t *buf, size_t length) {
    if(length < 6U || length < buf[3] + 4U)
        throw EXC_MALFORMED_PACKET;
    if(buf[0] != 0xFF || buf[1] != 0xFF)
        throw EXC_MALFORMED_PACKET;
    if(!validate_checksum(buf))
        throw EXC_BAD_CHECKSUM;
    
    num_parameters = buf[3] - 2;
    if(num_parameters > 0) {
        parameters = new uint8_t[num_parameters];
        memcpy(parameters, buf+5, num_parameters);
    } else {
        parameters = NULL;
    }

    error = buf[4];
    param_ptr = 0;
}

StatusPacket::~StatusPacket() {
    if(parameters)
        delete[] parameters;
}

void StatusPacket::debug_error() {
    printf("Errors: ");
    if(error == 0) {
        printf("NONE\n");
        return;
    } else {
        if(error & ERR_INSTRUCTION)
            printf("INSTRUCTION ");
        if(error & ERR_OVERLOAD)
            printf("OVERLOAD ");
        if(error & ERR_CHECKSUM)
            printf("CHECKSUM ");
        if(error & ERR_RANGE)
            printf("RANGE ");
        if(error & ERR_OVERHEAT)
            printf("OVERHEAT ");
        if(error & ERR_ANGLE_LIMIT)
            printf("ANGLE_LIMIT ");
        if(error & ERR_VOLTAGE)
            printf("ERR_VOLTAGE ");
        printf("\n");
    }
}

bool StatusPacket::validate_checksum(uint8_t *buf) {
    uint8_t checksum = 0;
    uint8_t len = buf[3];
    for(unsigned int i=0; i<len+1U; i++) 
        checksum += buf[2+i];
    checksum = ~checksum;
    return checksum == buf[len+3];
}

}

