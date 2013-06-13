#ifndef ACTUATOR_GAZEBO_H
#define ACTUATOR_GAZEBO_H

#include <gazebo.hh>
#include <common/common.hh>
#include <common/PID.hh>
#include <physics/physics.hh>

#include <gambit_driver/actuator.h>

namespace gambit_driver {

class ActuatorGazebo: public Actuator {
public:
    ActuatorGazebo(gazebo::physics::ModelPtr model, std::string joint_name);

    double get_position();
    double get_velocity();
    double get_torque();
    bool ping();
    void set_torque_enabled(bool enabled);
    void read_status();
    virtual void send_control();

    //void set_status(double position, double velocity, double torque);

protected:
    gazebo::physics::ModelPtr model_;
    gazebo::common::PID pid_;
    gazebo::physics::JointPtr joint_;
    gazebo::common::Time last_update_time_;

    double position_;
    double velocity_;
    double torque_;

    std::string joint_name_;
};

}

#endif // ACTUATOR_GAZEBO_H
