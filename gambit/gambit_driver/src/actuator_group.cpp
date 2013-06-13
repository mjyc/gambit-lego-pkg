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
 * A group of actuators comprising a part of the robot that works together, such
 * as the wrist.
 */

#include <gambit_driver/actuator_group.h>

namespace gambit_driver {

ActuatorGroup::ActuatorGroup() {
    last_control_time = ros::Time::now();
    last_status_print_time = ros::Time::now();
    avg_control_duration = 0;
}

void ActuatorGroup::initialize() {
    ROS_DEBUG("Initializing actuator group");
}

bool ActuatorGroup::check_connectivity() {
    ROS_DEBUG("Assuming connectivity");
    return true;
}

void ActuatorGroup::control() { }


void ActuatorGroup::read_status() {
    for(unsigned int i=0; i<actuators.size(); i++) 
        actuators[i]->read_status();
    for(unsigned int i=0; i<DOFs.size(); i++) 
        DOFs[i]->update();
}

void ActuatorGroup::send_control() {
    for(unsigned int i=0; i<actuators.size(); i++) {
        actuators[i]->send_control();
    }
}

void ActuatorGroup::control_loop_timing_monitor() {
    double since_last_control = (ros::Time::now() - last_control_time).toSec();
    last_control_time = ros::Time::now();
    avg_control_duration = (avg_control_duration * 31.0 + since_last_control) / 32.0;
    if( (ros::Time::now() - last_status_print_time).toSec() > 10.0 ) {
        ROS_DEBUG("control loop running at %0.01f Hz", 1.0 / avg_control_duration);
        last_status_print_time = ros::Time::now();
    }    
}

void ActuatorGroup::get_ros_jointstate(sensor_msgs::JointState &js) {
    for(unsigned int i=0; i<DOFs.size(); i++) {
        DOF *d = DOFs[i];
        js.name.push_back(d->get_name());
        js.position.push_back(d->get_position());
        js.velocity.push_back(d->get_velocity());
        js.effort.push_back(d->get_torque());
    }
}

void ActuatorGroup::load_calibration_parameters() {
    double value;
    ros::NodeHandle nh("~");

    for(unsigned int i=0; i<DOFs.size(); i++) {
        DOF *d = DOFs[i];
        if(nh.getParam(d->get_name() + std::string("_offset"), value)) {
            ROS_DEBUG("Setting joint offset for %s to %06.02f deg", d->get_name().c_str(), 
                value * 180.0 / M_PI);
            d->set_offset(value);
        }
    }

    for(unsigned int i=0; i<actuators.size(); i++) {
        Actuator *a = actuators[i];
        a->load_calibration_parameters();
    }
}


}

