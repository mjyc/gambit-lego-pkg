
#ifndef __bus_interface_h__
#define __bus_interface_h__

#include <string.h>
#include <stdint.h>
#include <pthread.h>

namespace bus {

class Interface {

    public:
    Interface() { pthread_mutex_init(&mutex, NULL); }

    virtual size_t read(uint8_t *buf, size_t length) = 0;
    virtual size_t write(const uint8_t *buf, size_t length) = 0;

    inline void lock() { pthread_mutex_lock(&mutex); }
    inline void unlock() { pthread_mutex_unlock(&mutex); }

    protected:
    pthread_mutex_t mutex;

};

}

#endif // __bus_interface_h__

