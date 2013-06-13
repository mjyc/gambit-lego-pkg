#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h>

#include <bus/interface_tty.h>
#include <dynamixel/actuator.h>

using namespace dynamixel;

int main(int argc, char **argv) {
    int id = 1;
    if(argc > 1) id = atoi(argv[1]);

    bus::InterfaceTTY *iface;
    try {
        iface = new bus::InterfaceTTY("/dev/tty81a", B115200);
    } catch(int e) {
        printf("Failed to open interface: %d\n", e);
        exit(-1);
    }

    RX28 a(iface, id);
    if(a.ping()) {
        printf("Node %d is present\n", id);
    } else {
        printf("Node %d is not present\n", id);
    }

    delete iface;

    return 0;
}

