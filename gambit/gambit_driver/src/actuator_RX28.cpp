/**
 * @file
 * @author Brian D. Mayton <bmayton@bdm.cc>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2010, Intel Labs Seattle
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - Neither the name of Intel Labs Seattle nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * An implementation of Actuator for Dynamixel RX-28 (and similar) servomotors.
 */

#include <ros/ros.h>
#include <gambit_driver/actuator_RX28.h>

namespace gambit_driver {

ActuatorRX28::ActuatorRX28(bus::Interface *iface, unsigned int node_id) {
    rx28 = new dynamixel::RX28(iface, node_id);
    if(!rx28) {
        ROS_FATAL("Unable to create actuator (out of memory?)");
        throw "out of memory";
    }
}

ActuatorRX28::~ActuatorRX28() {
    delete rx28;
}

double ActuatorRX28::get_position() {
    return rx28->get_position();
}

double ActuatorRX28::get_velocity() {
    return rx28->get_speed() * RX28_SPEED_CONVERSION;
}

double ActuatorRX28::get_torque() {
    return rx28->get_load() * RX28_MAX_TORQUE / rx28->get_torque_limit();
}

bool ActuatorRX28::ping() {
    return rx28->ping();
}

void ActuatorRX28::set_torque_enabled(bool enabled) {
    rx28->set_torque_enabled(enabled);
}

void ActuatorRX28::read_status() {
    rx28->read_status();
    uint8_t status = rx28->get_status();
    if(status != 0) {
        double since_last_warning = (ros::Time::now() - last_error_warning).toSec();
        if(since_last_warning > 5.0 || last_error_state != status) {
            if(status & dynamixel::StatusPacket::ERR_INSTRUCTION)
                ROS_ERROR("Bad instruction for actuator on joint %d", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_OVERLOAD)
                ROS_ERROR("Actuator on joint %d is overloaded", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_CHECKSUM)
                ROS_ERROR("Bad checksum for actuator on joint %d", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_RANGE)
                ROS_ERROR("Range error for actuator on joint %d", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_OVERHEAT)
                ROS_ERROR("Actuator on joint %d is overheating", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_ANGLE_LIMIT)
                ROS_ERROR("Angle limit error for actuator on joint %d", DOFs[0]->get_index());
            if(status & dynamixel::StatusPacket::ERR_VOLTAGE)
                ROS_ERROR("Invalid supply voltage to actuator on joint %d", 
                    DOFs[0]->get_index());
            last_error_warning = ros::Time::now();
            last_error_state = status;
        }
    } else {
        last_error_state = 0;
    }
}

void ActuatorRX28::send_control() {
    rx28->set_position(get_cmd_position());
}

bool ActuatorRX28::set_property(DOFProperty &p) {
    if(p.property_name == "TORQUE_LIMIT") {
        if(p.float_value > RX28_MAX_TORQUE) p.float_value = RX28_MAX_TORQUE;
        else if(p.float_value < 0.0) p.float_value = 0.0;
        int tlim = (int)round(p.float_value * 1023.0 / RX28_MAX_TORQUE);
        rx28->set_torque_limit(tlim);
        return true;
    } else if(p.property_name == "COMPLIANCE_MARGIN") {
        if(p.int_value > 254) p.int_value = 254;
        if(p.int_value < 0) p.int_value = 0;
        rx28->set_compliance_margin(p.int_value);
        return true;
    } else if(p.property_name == "COMPLIANCE_SLOPE") {
        if(p.int_value > 254) p.int_value = 254;
        if(p.int_value < 0) p.int_value = 0;
        rx28->set_compliance_slope(p.int_value);
        return true;
    } else if(p.property_name == "COMPLIANCE_PUNCH") {
        if(p.int_value > 1023) p.int_value = 1023;
        if(p.int_value < 0) p.int_value = 0;
        rx28->set_punch(p.int_value);
        return true;
    }

    return false;
}

void ActuatorRX28::get_properties(std::vector<DOFProperty> &properties) {
    unsigned int idx = DOFs[0]->get_index();

    DOFProperty p_compl_margin;
    p_compl_margin.DOF_index = idx;
    p_compl_margin.property_name="COMPLIANCE_MARGIN";
    p_compl_margin.display_name="Margin";
    p_compl_margin.type="int";
    p_compl_margin.rw="rw";
    p_compl_margin.int_value = rx28->get_compliance_margin(); 
    properties.push_back(p_compl_margin);
    
    DOFProperty p_compl_slope;
    p_compl_slope.DOF_index = idx;
    p_compl_slope.property_name="COMPLIANCE_SLOPE";
    p_compl_slope.display_name="Slope";
    p_compl_slope.type="int";
    p_compl_slope.rw="rw";
    p_compl_slope.int_value = rx28->get_compliance_slope(); 
    properties.push_back(p_compl_slope);
    
    DOFProperty p_compl_punch;
    p_compl_punch.DOF_index = idx;
    p_compl_punch.property_name="COMPLIANCE_PUNCH";
    p_compl_punch.display_name="Punch";
    p_compl_punch.type="int";
    p_compl_punch.rw="rw";
    p_compl_punch.int_value = rx28->get_punch(); 
    properties.push_back(p_compl_punch);
    
    DOFProperty p_torque_limit;
    p_torque_limit.DOF_index = idx;
    p_torque_limit.property_name="TORQUE_LIMIT";
    p_torque_limit.display_name="Torque";
    p_torque_limit.type="float";
    p_torque_limit.rw="rw";
    p_torque_limit.float_value = rx28->get_torque_limit() * RX28_MAX_TORQUE / 1024.0; 
    properties.push_back(p_torque_limit);
    
    DOFProperty p_voltage;
    p_voltage.DOF_index = idx;
    p_voltage.property_name="VOLTAGE";
    p_voltage.display_name="Voltage";
    p_voltage.type="float";
    p_voltage.rw="r";
    p_voltage.float_value = rx28->get_voltage(); 
    properties.push_back(p_voltage);
    
    DOFProperty p_temperature;
    p_temperature.DOF_index = idx;
    p_temperature.property_name="TEMPERATURE";
    p_temperature.display_name="Temp.";
    p_temperature.type="int";
    p_temperature.rw="r";
    p_temperature.int_value = (int)round(rx28->get_temperature()); 
    properties.push_back(p_temperature);
}

}

