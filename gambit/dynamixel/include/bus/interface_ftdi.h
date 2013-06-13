
#ifndef __bus_interface_ftdi_h__
#define __bus_interface_ftdi_h__

#include <bus/interface.h>
#include <ftd2xx.h>

namespace bus {

class InterfaceFTDI : Interface {

    public:

    // Exceptions
    static const int EXC_OPEN_FAILED = -1;
    static const int EXC_RESET_FAILED = -2;
    static const int EXC_INVALID_BAUDRATE = -3;
    static const int EXC_SETUP_FAILED = -4;

    InterfaceFTDI(const char *descriptor, unsigned int baudrate);
    ~InterfaceFTDI();

    virtual size_t read(uint8_t *buf, size_t length);
    virtual size_t write(const uint8_t *buf, size_t length);

    protected:
    FT_HANDLE ftdi;
};

}

#endif // __bus_interface_ftdi_h__
