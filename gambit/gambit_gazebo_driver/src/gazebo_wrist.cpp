
#include <gambit_gazebo_driver/gazebo_wrist.h>

namespace gambit_driver {

GazeboWrist::GazeboWrist(gazebo::physics::ModelPtr model) {
    ActuatorGazebo *m1 = new ActuatorGazebo(model,"wrist_roll");
    ActuatorGazebo *m2 = new ActuatorGazebo(model,"wrist_pitch");
    ActuatorGazebo *m3 = new ActuatorGazebo(model,"wrist_yaw");
    // ActuatorGazeboGripper class has hard-coded joint names:
    // "gripper_left" and "gripper_right"
    ActuatorGazeboGripper *m4 = new ActuatorGazeboGripper(model);

    // Make sure to have DOF class' joint names match with ActuatorGazebo
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

    // "/arm/ros_joint_state" requires joint name "gripper" as
    // its 7th joint-value
    // Check echoed topics from "/arm/ros_joint_state" using
    // armsim or armlib executables.
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

bool GazeboWrist::check_connectivity() {
    return true;
}

void GazeboWrist::control() {
    read_status();
    send_control();
}

void GazeboWrist::get_ros_jointstate(sensor_msgs::JointState &js) {
    ActuatorGroup::get_ros_jointstate(js);

    DOF *gripper = DOFs[3]; // "gripper" joint
    double d = gripos2ROSpos(actuators[3]->get_position());

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

