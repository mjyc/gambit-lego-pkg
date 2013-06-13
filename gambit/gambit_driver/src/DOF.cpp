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
 * Classes for representing robotic degrees of freedom.
 */

#include <ros/console.h>
#include <gambit_driver/DOF.h>

namespace gambit_driver {

DOF::DOF(std::string dof_name) {
    enabled = false;
    continuous_rotation = false;
    name = dof_name;
    offset = 0.0;
}

void DOF::add_actuator(Actuator *a, double weight) {
    actuators.push_back(a);
    weights.push_back(weight);
    if(actuators.size() == 1) {
        a->set_name(name);
    }
}

void DOF::set_limits(double angle_min, double angle_max) {
    limit_min = angle_min;
    limit_max = angle_max;
}

void DOF::set_offset(double joint_offset) {
    offset = joint_offset;
}

double DOF::get_absolute_position(bool apply_offset) {
    double p = 0.0;
    for(unsigned int i=0; i<actuators.size(); i++)
        p += actuators[i]->get_position() * weights[i];
    if(apply_offset)
        p = p + offset;
    return p;
}

double DOF::get_position() {
    double position = get_absolute_position();
    if(continuous_rotation) {
        position = atan2(sin(position), cos(position));
    }
    return position;
}

double DOF::get_velocity() {
    double v = 0.0;
    for(unsigned int i=0; i<actuators.size(); i++)
        v += actuators[i]->get_velocity() * weights[i];
    return v;
}

double DOF::get_torque() {
    double T = 0.0;
    for(unsigned int i=0; i<actuators.size(); i++)
        T += actuators[i]->get_torque() * weights[i];
    return T;
}

void DOF::update() {
    if(!enabled) {
        cmd_position = get_position() - offset;
    }
}

void DOF::set_enabled(bool en) {
    enabled = en;
    if(actuators.size() > 0)
        actuators[0]->set_torque_enabled(enabled);
}

void DOF::get_properties(std::vector<DOFProperty> &properties) {
    DOFProperty p_enabled;
    p_enabled.DOF_index = index;
    p_enabled.property_name = "DOF_ENABLED";
    p_enabled.display_name = "Enabled";
    p_enabled.type = "bool";
    p_enabled.rw = "rw";
    p_enabled.int_value = enabled;
    properties.push_back(p_enabled);

    if(actuators.size() > 0)
        actuators[0]->get_properties(properties);
}

void DOF::get_DOFInfo(DOFInfo &info) {
    info.DOF_index = index;
    info.display_name = name;
    info.position = get_position();
    info.target = cmd_position;
    info.limit_min = limit_min;
    info.limit_max = limit_max;

    get_properties(info.properties);
}

double DOF::get_cmd_position() {
    if(!enabled)
        return get_position();
    return cmd_position;
}

void DOF::set_cmd_position(double new_position, bool apply_offset) {
    if(!enabled)
        return; 
    if(continuous_rotation) {
        double cur_position = get_absolute_position();
        double a1 = atan2(sin(cur_position), cos(cur_position));
        double a2 = atan2(sin(new_position), cos(new_position));
        double da = a2 - a1;
        if(da < -M_PI) da += M_PI * 2;
        if(da > M_PI) da -= M_PI * 2;
        new_position = cur_position + da;
    }

    cmd_position = new_position - (apply_offset? offset : 0);
}

void DOF::set_cmd_position(double new_position, ros::Time timestamp, bool apply_offset) {
    set_cmd_position(new_position, apply_offset);
}

bool DOF::set_property(DOFProperty &p) {
    if(p.property_name == "DOF_ENABLED") {
        set_enabled(p.int_value);
        return true;
    } else {
        if(actuators.size() > 0)
            return actuators[0]->set_property(p);
    }
    return false;
}

void DOF::set_continuous_rotation(bool value) {
    continuous_rotation = value;
}

}

