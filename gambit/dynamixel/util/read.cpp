#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>

#include <bus/interface_tty.h>
#include <dynamixel/packet.h>

using namespace dynamixel;

int main(int argc, char **argv) {
    if(argc < 5) {
        printf("usage: %s <baudrate> <id> <addr> <type>\n", argv[0]);
        printf("valid types: short, byte\n");
        exit(-1);
    }
    int baudrate;
    int id = atoi(argv[2]);
    int addr = atoi(argv[3]);
    
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

    InstructionPacket c(id, InstructionPacket::CMD_READ);
    c.add_byte(addr);
    
    size_t to_read = 6;
    if(strncmp("short", argv[4], 1) == 0) {
        to_read += 2;
        c.add_byte(2);
    } else if(strncmp("byte", argv[4], 1) == 0) {
        to_read += 1;
        c.add_byte(1);
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

    length = iface->read(buf, to_read);
    printf("Received %d bytes\n", length);

    StatusPacket *s;
    try {
        s = new StatusPacket(buf, length);
    } catch(int e) {
        printf("Bad packet in response: %d\n", e);
        exit(e);
    }

    s->debug_error();

    if(strncmp("short", argv[4], 1) == 0) {
        printf("VALUE (short): %d\n", s->read_short());
    } else if(strncmp("byte", argv[4], 1) == 0) {
        printf("VALUE (byte):  %d\n", s->read_byte());
    }
    
    delete s;
    delete iface;

    return 0;
}

