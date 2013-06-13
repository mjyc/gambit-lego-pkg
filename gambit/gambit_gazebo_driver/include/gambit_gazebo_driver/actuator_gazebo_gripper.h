#ifndef ACTUATOR_GAZEBO_GRIPPER_H
#define ACTUATOR_GAZEBO_GRIPPER_H

#include <gazebo.hh>
#include <common/common.hh>
#include <common/PID.hh>
#include <physics/physics.hh>

#include <gambit_driver/actuator.h>
#include <gambit_gazebo_driver/gripper_utils.h>

namespace gambit_driver {

class ActuatorGazeboGripper: public Actuator {
public:
    ActuatorGazeboGripper(gazebo::physics::ModelPtr model);

    double get_position();
    double get_velocity();
    double get_torque();
    bool ping();
    void set_torque_enabled(bool enabled);
    void read_status();
    virtual void send_control();

protected:
    gazebo::physics::ModelPtr model_;
    gazebo::common::PID pidL_;
    gazebo::common::PID pidR_;
    gazebo::physics::JointPtr jointL_;
    gazebo::physics::JointPtr jointR_;
    gazebo::common::Time last_update_time_;

    double positionL_;
    double positionR_;
    double velocity_;
    double torque_;
};


ActuatorGazeboGripper::ActuatorGazeboGripper(gazebo::physics::ModelPtr model) {
    model_ = model;
    // uses hard coded joint names
    jointL_ = model_->GetJoint("gripper_left");
    jointR_ = model_->GetJoint("gripper_right");
    pidL_ = gazebo::common::PID(100, 0, 1, 0, 0, 100, -100);
    pidR_ = gazebo::common::PID(100, 0, 1, 0, 0, 100, -100);
    last_update_time_ = model_->GetWorld()->GetSimTime();

    positionL_ = 0.0;
    positionR_ = 0.0;
    velocity_ = 0.0;
    torque_ = 0.0;
}

double ActuatorGazeboGripper::get_position() {
    // returns gripper-space value
    return ROSpos2gripos(positionL_);
}

double ActuatorGazeboGripper::get_velocity() {
    return velocity_;
}

double ActuatorGazeboGripper::get_torque() {
    return torque_;
}

bool ActuatorGazeboGripper::ping() {
    return true;
}

void ActuatorGazeboGripper::set_torque_enabled(bool enabled) {
    /* unimplemented */
}

void ActuatorGazeboGripper::read_status() {
    // keep ROS-joint-space joint_states
    positionL_ = jointL_->GetAngle(0).Radian();
    positionR_ = jointR_->GetAngle(0).Radian();
}

void ActuatorGazeboGripper::send_control() {
    gazebo::common::Time current_time = model_->GetWorld()->GetSimTime();
    double dt = current_time.Double()-last_update_time_.Double();

    double cur_posL = positionL_;
    double cur_posR = positionR_;
    // commands are in gripper-space
    double target_posL = gripos2ROSpos( get_cmd_position() );
    double target_posR = -1.0*gripos2ROSpos( get_cmd_position() );

    pidL_.SetCmd(target_posL);
    pidL_.Update(cur_posL-target_posL,dt);
    pidR_.SetCmd(target_posR);
    pidR_.Update(cur_posR-target_posR,dt);

    jointL_->SetForce(0, pidL_.GetCmd());
    jointR_->SetForce(0, pidR_.GetCmd());
    last_update_time_ = current_time;
}

}

#endif // ACTUATOR_GAZEBO_GRIPPER_H
