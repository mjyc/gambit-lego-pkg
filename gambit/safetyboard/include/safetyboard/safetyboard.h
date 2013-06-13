#ifndef __safetyboard_h_
#define __safetyboard_h_

#include <ros/ros.h>
#include <dynamixel/packet.h>
#include <bus/interface.h>

#include <safetyboard/status.h>

class SafetyBoard {
    
    public:
    SafetyBoard(bus::Interface *bus_iface, unsigned int node_id);

    bool ping();
    int get_state();
    int get_fault();
    void set_state(int state);

    protected:
    bus::Interface *iface;
    unsigned int id;

    bool send_packet(dynamixel::InstructionPacket *req, dynamixel::StatusPacket **resp);
};

#endif // __safetyboard_h_

