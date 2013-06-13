/**
 * @file
 * @author brian d. mayton <bmayton@bdm.cc>
 * @version 1.0
 *
 * @section license
 *
 * copyright (c) 2010, intel labs seattle
 * all rights reserved.
 * 
 * redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - neither the name of intel labs seattle nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  this software is provided by the copyright holders and contributors "as is"
 *  and any express or implied warranties, including, but not limited to, the
 *  implied warranties of merchantability and fitness for a particular purpose
 *  are disclaimed. in no event shall the copyright holder or contributors be
 *  liable for any direct, indirect, incidental, special, exemplary, or
 *  consequential damages (including, but not limited to, procurement of
 *  substitute goods or services; loss of use, data, or profits; or business
 *  interruption) however caused and on any theory of liability, whether in
 *  contract, strict liability, or tort (including negligence or otherwise)
 *  arising in any way out of the use of this software, even if advised of the
 *  possibility of such damage.
 *
 * @section description
 *
 * Actuator group for fake wrist dofs.
 */

#include <armsim/fake_wrist.h>

namespace gambit_driver {

FakeWrist::FakeWrist() {
    ActuatorDummy *m1 = new ActuatorDummy();
    ActuatorDummy *m2 = new ActuatorDummy();
    ActuatorDummy *m3 = new ActuatorDummy();
    ActuatorDummy *m4 = new ActuatorDummy();

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
}

bool FakeWrist::check_connectivity() {
    return true;
}

void FakeWrist::control() {
    read_status();
    send_control();
}

void FakeWrist::get_ros_jointstate(sensor_msgs::JointState &js) {
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


}

