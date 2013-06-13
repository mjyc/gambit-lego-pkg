#include <dynamixel/actuator.h>

namespace dynamixel {

RX28::RX28(bus::Interface *bus_iface, unsigned int node_id) {
    iface = bus_iface;
    id = node_id;

    // Set angle parameters
    rads_per_count = 0.005113269;
    zero_offset = -2.617993878;

    limit_min = -2.617993878;
    limit_max = 2.617993878;

    // Initial status
    compliance_margin = 0;
    compliance_slope = 32;
    compliance_punch = 32;
    torque_limit = 0x03FF; // default from datasheet
    position = 0;
    speed = 0;
    load = 0;
    voltage = 0;
    temperature = 0;
    reg_instruction = false;
    moving = false;
    
    torque_enabled = false;
    cmd_position = 0;

    status = 0;
}

void RX28::set_rads_per_count(double rads, double offset) {
    rads_per_count = rads;
    zero_offset = offset;
}

bool RX28::ping() {
    InstructionPacket cmd(id, InstructionPacket::CMD_PING);
    StatusPacket *resp;

    if(send_packet(&cmd, &resp)) {
        delete resp;
        return true;
    } else {
        return false;
    }
}

bool RX28::read_status() {
    InstructionPacket cmd(id, InstructionPacket::CMD_READ);
    StatusPacket *resp;

    cmd.add_byte(REG_PRESENT_POSITION); // start of status area
    cmd.add_byte(10); // read 10 bytes
    
    bool success = send_packet(&cmd, &resp);
    if(!success) {
        ROS_WARN("Failed to read status from actuator %d", id);
        return false;
    }

    position = resp->read_short();
    speed = resp->read_short();
    speed = (speed & (1<<10)) ? (speed & 0x3FF) * -1 : speed;
    load = resp->read_short();
    load = (load & (1<<10)) ? (load & 0x3FF) * -1 : load;
    voltage = resp->read_byte() / 10.0;
    temperature = resp->read_byte();
    reg_instruction = (resp->read_byte() > 0);
    moving = (resp->read_byte() > 0);
    status = resp->get_error();

    if(!torque_enabled) cmd_position = position;

    delete resp;
    return true;
}

void RX28::set_torque_enabled(bool enabled) {
    torque_enabled = enabled;

    // Construct and send packet
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_TORQUE_ENABLE);
    cmd.add_byte(torque_enabled ? 1 : 0);
    send_packet(&cmd, NULL);
}

void RX28::set_compliance_margin(uint8_t margin) {
    compliance_margin = margin;

    // Construct and send packet
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_CW_COMPLIANCE_MARGIN);
    cmd.add_byte(margin);
    cmd.add_byte(margin);
    send_packet(&cmd, NULL);
}

void RX28::set_compliance_slope(uint8_t slope) {
    compliance_slope = slope;

    // Construct and send packet
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_CW_COMPLIANCE_SLOPE);
    cmd.add_byte(slope);
    cmd.add_byte(slope);
    send_packet(&cmd, NULL);
}

void RX28::set_punch(uint16_t punch) {
    compliance_punch = punch;

    // Construct and send packet
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_PUNCH);
    cmd.add_short(punch);
    send_packet(&cmd, NULL);
}

void RX28::set_torque_limit(uint16_t limit) {
    torque_limit = limit;

    // Construct and send packet
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_TORQUE_LIMIT);
    cmd.add_short(limit);
    send_packet(&cmd, NULL);
}

void RX28::set_position(double new_pos) {
    if(!torque_enabled) return;

    // Bound to set limits
    if(new_pos < limit_min) new_pos = limit_min;
    if(new_pos > limit_max) new_pos = limit_max;

    cmd_position = new_pos;

    // Convert to encoder ticks
    int encoder_pos = (new_pos - zero_offset) / rads_per_count;

    // Make sure we're not exceeding hardware limitations
    if(encoder_pos < 0) encoder_pos = 0;
    if(encoder_pos > 1023) encoder_pos = 1023;

    // Construct and send motor command
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_GOAL_POSITION);
    cmd.add_short(encoder_pos);
    send_packet(&cmd, NULL);
}

bool RX28::send_packet(InstructionPacket *req, StatusPacket **resp) {
    uint8_t buf[InstructionPacket::MAX_LENGTH];
    size_t length;
    size_t sent_length;
    
    length = req->get_binary(buf);

    iface->lock();
    sent_length = iface->write(buf, length);

    if(sent_length != length) {
        ROS_WARN("Failed to send complete packet");
        iface->unlock();
        return false;
    }

    if(resp) {
        length = iface->read(buf, 6);
        if(length != 6) {
            ROS_DEBUG("Failed to read response packet header");
            iface->unlock();
            return false;
        }

        size_t bytes_remaining = (buf[3] + 4) - 6;
        if(bytes_remaining) {
            length = iface->read(buf+6, bytes_remaining);
            if(length != bytes_remaining) {
                ROS_DEBUG("Failed to read response packet data");
                iface->unlock();
                return false;
            }
        }

        try {
            *resp = new StatusPacket(buf, buf[3] + 4);
        } catch(int e) {
            ROS_WARN("Failed to deserialize response packet: %d", e);
            iface->unlock();
            return false;
        }
    }

    iface->unlock();
    return true;
}

}

