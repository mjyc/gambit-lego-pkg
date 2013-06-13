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
 * High-level definiton of the Gambit wrist.
 */


#include <gambit_driver/wrist.h>
#include <gambit_driver/actuator_RX28.h>
#include <gambit_driver/actuator_dummy.h>
#include <math.h>

namespace gambit_driver {

Wrist::Wrist(bus::Interface *interface) {
    iface = interface;

#ifdef DUMMY_WRIST
    ActuatorDummy *m1 = new ActuatorDummy();
    ActuatorDummy *m2 = new ActuatorDummy();
    ActuatorDummy *m3 = new ActuatorDummy();
    ActuatorDummy *m4 = new ActuatorDummy();
#else
    ActuatorRX28 *m1 = new ActuatorRX28(iface, 1);
    ActuatorRX28 *m2 = new ActuatorRX28(iface, 2);
    ActuatorRX28 *m3 = new ActuatorRX28(iface, 3);
    ActuatorRX28 *m4 = new ActuatorRX28(iface, 4);
#endif
    
    DOF *j1 = new DOF("wrist_roll");
    j1->set_limits(-150.0 * M_PI / 180.0, 150.0 * M_PI / 180.0);
    j1->add_actuator((Actuator*)m1, 1.0);
    m1->add_DOF(j1, 1.0);

    DOF *j2 = new DOF("wrist_pitch");
    j2->set_limits(-90.0 * M_PI / 180.0, 90.0 * M_PI / 180.0);
    j2->add_actuator((Actuator*)m2, 1.0);
    m2->add_DOF(j2, 1.0);

    DOF *j3 = new DOF("wrist_yaw");
    j3->set_limits(-150.0 * M_PI / 180.0, 150.0 * M_PI / 180.0);
    j3->add_actuator((Actuator*)m3, -1.0);
    j3->add_actuator(m2, -1.0);
    m3->add_DOF(j3, -1.0);
    m3->add_DOF(j2, -1.0);
    
    DOF *j4 = new DOF("gripper");
    j4->set_limits(-60.0 * M_PI / 180.0, 45.0 * M_PI / 180.0);
    j4->add_actuator((Actuator*)m4, 1.0);
    m4->add_DOF(j4, 1.0);

    actuators.push_back(m1);
    actuators.push_back(m2);
    actuators.push_back(m3);
    actuators.push_back(m4);

    DOFs.push_back(j1);
    DOFs.push_back(j2);
    DOFs.push_back(j3);
    DOFs.push_back(j4);

    load_calibration_parameters();

    ROS_DEBUG("Wrist configured with %d actuators in %d DOFs", actuators.size(), 
        DOFs.size());
}

Wrist::~Wrist() {
    unsigned int i;
    for(i=0; i<DOFs.size(); i++)
        delete DOFs[i];
    for(i=0; i<actuators.size(); i++)
        delete actuators[i];
}


bool Wrist::check_connectivity() {
    ROS_DEBUG("Checking wrist connectivity");
    bool all_ok = true;
    for(unsigned int i=0; i<actuators.size(); i++) {
        ROS_DEBUG("Probing for actuators[%d]", i);
        bool present = actuators[i]->ping();
        if(present)
            ROS_DEBUG("Got response from actuators[%d]", i);
        else
            ROS_ERROR("wrist: actuators[%d] NOT PRESENT", i);
        all_ok = all_ok && present;
    }
    return all_ok;
}

void Wrist::initialize() {
    ROS_DEBUG("Initializing wrist");
    for(unsigned int i=0; i<actuators.size(); i++) {
        actuators[i]->set_torque_enabled(false);
    }
    read_status();
}

void Wrist::get_ros_jointstate(sensor_msgs::JointState &js) {
    ActuatorGroup::get_ros_jointstate(js);

    DOF *gripper = DOFs[3];
    double r = GRIPPER_HUB_RADIUS;
    double l = GRIPPER_LINKAGE_LENGTH;
    double t = M_PI/2.0 - gripper->get_position();
    double d = r * cos(t) + sqrt(l*l - pow(r * sin(t), 2.0));

    js.name.push_back("gripper_left");
    js.position.push_back(d);
    js.velocity.push_back(0.0);
    js.effort.push_back(gripper->get_torque()/2.0);
    
    js.name.push_back("gripper_right");
    js.position.push_back(-d);
    js.velocity.push_back(0.0);
    js.effort.push_back(gripper->get_torque()/2.0);
}

void Wrist::control() {
    read_status();
    send_control();
    control_loop_timing_monitor();
}

}

