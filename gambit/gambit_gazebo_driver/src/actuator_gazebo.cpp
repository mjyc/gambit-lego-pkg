
#include <gambit_gazebo_driver/actuator_gazebo.h>

namespace gambit_driver {

ActuatorGazebo::ActuatorGazebo(gazebo::physics::ModelPtr model, std::string joint_name) {
    model_ = model;
    joint_ = model_->GetJoint(joint_name);
    pid_ = gazebo::common::PID(100, 0, 1, 0, 0, 100, -100);
    last_update_time_ = model_->GetWorld()->GetSimTime();

    position_ = 0.0;
    velocity_ = 0.0;
    torque_ = 0.0;

    joint_name_ = joint_name;
}

double ActuatorGazebo::get_position() {
    return position_;
}

double ActuatorGazebo::get_velocity() {
    return velocity_;
}

double ActuatorGazebo::get_torque() {
    return torque_;
}

bool ActuatorGazebo::ping() {
    return true;
}

void ActuatorGazebo::set_torque_enabled(bool enabled) {
    /* unimplemented */
}

void ActuatorGazebo::read_status() {
    position_ = joint_->GetAngle(0).Radian();
}

void ActuatorGazebo::send_control() {
    gazebo::common::Time current_time = model_->GetWorld()->GetSimTime();
    double dt = current_time.Double()-last_update_time_.Double();
    double cur_pos = position_;
    double target_pos = get_cmd_position();
    pid_.SetCmd(target_pos);
    pid_.Update(cur_pos-target_pos,dt);
    joint_->SetForce(0, pid_.GetCmd());
    last_update_time_ = current_time;
}

}
