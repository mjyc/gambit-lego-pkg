#include <bus/interface_tty.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
namespace std {
#include <unistd.h>
}

namespace bus {

InterfaceTTY::InterfaceTTY(const char *filename, unsigned int baudrate) {
    fd = open(filename, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0) throw EXC_OPEN_FAILED;

    struct termios tio;
    bzero(&tio, sizeof(tio));
    tio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    // no minimum number of characters, no timer
    tio.c_cc[VTIME] = 0;
    tio.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &tio);
}

InterfaceTTY::~InterfaceTTY() {
    std::close(fd);
}

size_t InterfaceTTY::read(uint8_t *buf, size_t length) {
    size_t bytes_read = 0;
    unsigned int time_waited = 0;

    while(true) {
        bytes_read += std::read(fd, buf + bytes_read, length - bytes_read);
        if(bytes_read == length) break;
        std::usleep(100);
        time_waited += 100;
        if(time_waited > TIMEOUT) break;
    }
    return bytes_read;
}

size_t InterfaceTTY::write(const uint8_t *buf, size_t length) {
    return std::write(fd, buf, length);
}

}

