#include <bus/interface_ftdi.h>
#include <stdlib.h>

namespace bus {

InterfaceFTDI::InterfaceFTDI(const char *descriptor, unsigned int baudrate) {
    FT_STATUS status;

    char *desc = strdup(descriptor);
    if(!desc) throw EXC_OPEN_FAILED;
    status = FT_OpenEx(desc, FT_OPEN_BY_DESCRIPTION, &ftdi);
    free(desc);
    if(status != FT_OK) throw EXC_OPEN_FAILED;

    if(FT_ResetDevice(ftdi) != FT_OK) throw EXC_RESET_FAILED;
    if(FT_SetBaudRate(ftdi, baudrate) != FT_OK) throw EXC_INVALID_BAUDRATE;
    if(FT_SetLatencyTimer(ftdi, 1) != FT_OK) throw EXC_SETUP_FAILED;
    if(FT_SetTimeouts(ftdi, 1000, 1000) != FT_OK) throw EXC_SETUP_FAILED;
}

InterfaceFTDI::~InterfaceFTDI() {
    FT_Close(ftdi);
}

size_t InterfaceFTDI::read(uint8_t *buf, size_t length) {
    FT_STATUS status;
    DWORD n;

    status = FT_Read(ftdi, (LPVOID)buf, (DWORD)length, &n);
    if(status != FT_OK) 
        return 0;
    else
        return n;
}

size_t InterfaceFTDI::write(const uint8_t *buf, size_t length) {
    FT_STATUS status;
    DWORD n;

    status = FT_Write(ftdi, (LPVOID)buf, (DWORD)length, &n);
    if(status != FT_OK)
        return 0;
    else
        return n;
}

}

