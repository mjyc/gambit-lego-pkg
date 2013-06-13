#include <safetyboard/safetyboard.h>
#include <safetyboard/regmap.h>

using namespace dynamixel;

SafetyBoard::SafetyBoard(bus::Interface *bus_iface, unsigned int node_id) {
    iface = bus_iface;
    id = node_id;
}

bool SafetyBoard::ping() {
    InstructionPacket cmd(id, InstructionPacket::CMD_PING);
    StatusPacket *resp;

    if(send_packet(&cmd, &resp)) {
        delete resp;
        return true;
    } else {
        return false;
    }
}

int SafetyBoard::get_state() {
    InstructionPacket cmd(id, InstructionPacket::CMD_READ);
    StatusPacket *resp;

    cmd.add_byte(REG_STATE);
    cmd.add_byte(1);

    bool success = send_packet(&cmd, &resp);
    if(!success) {
        ROS_ERROR("Failed to read safety board state");
        return -1;
    }

    int state = resp->read_byte();
    delete resp;
    return state;
}

int SafetyBoard::get_fault() {
    InstructionPacket cmd(id, InstructionPacket::CMD_READ);
    StatusPacket *resp;

    cmd.add_byte(REG_FAULT);
    cmd.add_byte(1);

    bool success = send_packet(&cmd, &resp);
    if(!success) {
        ROS_ERROR("Failed to read safety board fault status");
        return -1;
    }

    int fault = resp->read_byte();
    delete resp;
    return fault;
}

void SafetyBoard::set_state(int state) {
    InstructionPacket cmd(id, InstructionPacket::CMD_WRITE);
    cmd.add_byte(REG_STATE);
    cmd.add_byte((uint8_t)state);
    send_packet(&cmd, NULL);
}

bool SafetyBoard::send_packet(InstructionPacket *req, StatusPacket **resp) {
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


