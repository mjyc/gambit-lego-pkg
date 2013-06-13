
#ifndef __bus_interface_tty_h__
#define __bus_interface_tty_h__

#include <bus/interface.h>

namespace bus {

class InterfaceTTY : public Interface {

    public:

    // Exceptions
    static const int EXC_OPEN_FAILED = -1;
    static const int EXC_RESET_FAILED = -2;
    static const int EXC_INVALID_BAUDRATE = -3;
    static const int EXC_SETUP_FAILED = -4;

    static const unsigned int TIMEOUT = 10000;

    InterfaceTTY(const char *filename, unsigned int baudrate);
    ~InterfaceTTY();

    virtual size_t read(uint8_t *buf, size_t length);
    virtual size_t write(const uint8_t *buf, size_t length);

    protected:
    int fd;
};

}

#endif // __bus_interface_tty_h__
