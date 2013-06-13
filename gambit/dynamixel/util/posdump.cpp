#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>

#include <bus/interface_tty.h>
#include <dynamixel/actuator.h>

using namespace dynamixel;

int main(int argc, char **argv) {
    if(argc < 2) {
        printf("usage: %s <id>\n", argv[0]);
        exit(-1);
    }
    int id = atoi(argv[1]);

    bus::InterfaceTTY *iface;
    try {
        iface = new bus::InterfaceTTY("/dev/tty81a", B115200);
    } catch(int e) {
        printf("Failed to open USB interface: %d\n", e);
        exit(-1);
    }

    RX28 a(iface, id);

    for(;;) {
        a.read_status();

        
        printf("pos: %04.01f ", a.get_position() * 180. / 3.1415926);
        //printf("spd: %04d ", s->read_short());
        //printf("ld:  %04d ", s->read_short());
        printf("vlt: %03.01f ", a.get_voltage());
        printf("tmp: %03.01f ", a.get_temperature());
        //printf("reg: %03d ", s->read_byte());
        //printf("mov: %03d ", s->read_byte());
        printf("\n");
        
    }

    
    delete iface;

    return 0;
}

