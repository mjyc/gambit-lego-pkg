#ifndef __dynamixel_actuator_h_
#define __dynamixel_actuator_h_

#include <ros/ros.h>

#include <dynamixel/packet.h>
#include <bus/interface.h>

namespace dynamixel {

class RX28 {
    
    public:
    RX28(bus::Interface *bus_iface, unsigned int node_id);

    bool ping();
    bool read_status();
    void set_torque_enabled(bool enabled);
    void set_position(double new_pos);
    void set_compliance_margin(uint8_t margin);
    void set_compliance_slope(uint8_t slope);
    void set_punch(uint16_t punch);
    void set_torque_limit(uint16_t limit);

    void set_rads_per_count(double rads, double offset);
    void set_angle_limits(double min_angle, double max_angle);

    inline double get_position() { return position * rads_per_count + zero_offset; }
    inline double get_speed() { return speed; }
    inline double get_load() { return torque_enabled ? load : 0.0; }
    inline double get_voltage() { return voltage; }
    inline double get_temperature() { return temperature; }

    inline bool get_torque_enabled() { return torque_enabled; }
    inline uint8_t get_compliance_margin() { return compliance_margin; }
    inline uint8_t get_compliance_slope() { return compliance_slope; }
    inline uint8_t get_punch() { return compliance_punch; }
    inline uint16_t get_torque_limit() { return torque_limit; }
    inline double get_cmd_position() { return cmd_position * rads_per_count + zero_offset; }
    inline uint8_t get_status() { return status; }

    protected:
    bus::Interface *iface;
    unsigned int id;

    double rads_per_count;
    double zero_offset;
    double limit_min;
    double limit_max;

    uint8_t compliance_margin;
    uint8_t compliance_slope;
    uint16_t compliance_punch;
    uint16_t torque_limit;
    uint16_t position;
    int16_t speed;
    int16_t load;
    float voltage;
    uint8_t temperature;
    bool reg_instruction;
    bool moving;

    bool torque_enabled;
    uint16_t cmd_position;

    uint8_t status;

    bool send_packet(InstructionPacket *req, StatusPacket **resp);
};

}

#endif // __dynamixel_actuator_h_

