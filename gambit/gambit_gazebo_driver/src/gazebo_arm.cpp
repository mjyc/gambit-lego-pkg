
#include <gambit_gazebo_driver/gazebo_arm.h>

namespace gambit_driver {

GazeboArm::GazeboArm(gazebo::physics::ModelPtr model) {
    ActuatorGazebo *m1 = new ActuatorGazebo(model,"waist");
    ActuatorGazebo *m2 = new ActuatorGazebo(model,"shoulder");
    ActuatorGazebo *m3 = new ActuatorGazebo(model,"elbow");

    DOF *j1 = new DOF("waist");
    j1->set_limits(-M_PI, M_PI);
    j1->add_actuator((Actuator*)m1, 1.0);
    j1->set_continuous_rotation(true);
    m1->add_DOF(j1, 1.0);

    DOF *j2 = new DOF("shoulder");
    j2->set_limits(-M_PI, M_PI);
    j2->add_actuator((Actuator*)m2, 1.0);
    j2->set_continuous_rotation(true);
    m2->add_DOF(j2, 1.0);

    DOF *j3 = new DOF("elbow");
    j3->set_limits(-M_PI, M_PI);
    j3->add_actuator((Actuator*)m3, 1.0);
    j3->set_continuous_rotation(true);
    m3->add_DOF(j3, 1.0);

    actuators.push_back(m1);
    actuators.push_back(m2);
    actuators.push_back(m3);

    DOFs.push_back(j1);
    DOFs.push_back(j2);
    DOFs.push_back(j3);
}

bool GazeboArm::check_connectivity() {
    return true;
}

void GazeboArm::control() {
    read_status();
    send_control();
}

}


