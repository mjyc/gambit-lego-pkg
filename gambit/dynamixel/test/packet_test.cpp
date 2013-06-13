#include <dynamixel/packet.h>
#include <stdio.h>

using namespace dynamixel;

int main(int argc, char **argv) {
    InstructionPacket p(0x01, InstructionPacket::CMD_WRITE);
    p.add_byte(12);
    p.add_short(43620);
    p.debug_print();

    uint8_t buf[128];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0x01;
    buf[3] = 0x02;
    buf[4] = 0x24;
    buf[5] = 0xD8;

    try {
        StatusPacket s(buf, 6);
        s.debug_error();
    } catch(int e) {
        if(e == StatusPacket::EXC_MALFORMED_PACKET)
            printf("EXC_MALFORMED_PACKET\n");
        if(e == StatusPacket::EXC_BAD_CHECKSUM)
            printf("EXC_BAD_CHECKSUM\n");
        return -1;
    }


    return 0;
}

