#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>

#include <bus/interface_tty.h>
#include <dynamixel/packet.h>

using namespace dynamixel;

int main(int argc, char **argv) {
    if(argc < 6) {
        printf("usage: %s <baudrate> <id> <addr> <type> <value>\n", argv[0]);
        printf("valid types: short, byte\n");
        exit(-1);
    }
    int baudrate;
    int id = atoi(argv[2]);
    int addr = atoi(argv[3]);
    int value = atoi(argv[5]);


    if(strncmp(argv[1], "115200", 6) == 0) {
        baudrate = B115200;
    } else if(strncmp(argv[1], "57600", 5) == 0) {
        baudrate = B57600;
    } else {
        printf("unknown baudrate\n");
        exit(-2);
    }
    
    bus::InterfaceTTY *iface;
    try {
        iface = new bus::InterfaceTTY("/dev/tty81a", baudrate);
    } catch(int e) {
        printf("Failed to open USB interface: %d\n", e);
        exit(-1);
    }

    InstructionPacket c(id, InstructionPacket::CMD_WRITE);
    c.add_byte(addr);
    if(strncmp("byte", argv[4], 1) == 0) {
        c.add_byte(value);
    } else if(strncmp("short", argv[4], 1) == 0) {
        c.add_short(value);
    } else {
        printf("invalid type\n");
        exit(-1);
    }
    uint8_t buf[256];
    size_t length;
    size_t sent_length;

    length = c.get_binary(buf);
    sent_length = iface->write(buf, length);
    printf("Sent %d of %d bytes to device %d\n", sent_length, length, id);

    length = iface->read(buf, 6);
    printf("Received %d bytes\n", length);

    StatusPacket *s;
    try {
        s = new StatusPacket(buf, length);
    } catch(int e) {
        printf("Bad packet in response: %d\n", e);
        exit(e);
    }

    s->debug_error();

    delete s;
    delete iface;

    return 0;
}

