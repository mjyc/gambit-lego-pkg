#include <armlib/hybrid.h>
#include "ik_hybrid.cpp"

namespace armlib {

Hybrid::Hybrid() :
    _spinner(1, &_cb_queue)
{
    _latency_usec = 150000;
    _n_dofs = 8;
    _n_arm_dofs = 7;
    _n_manip_dofs = 1;
    _data_valid = false;
    _encoder_position.resize(_n_dofs);

    _joint_weights.push_back(3.0);
    _joint_weights.push_back(3.0);
    _joint_weights.push_back(1.5);
    _joint_weights.push_back(1.5);
    _joint_weights.push_back(0.5);
    _joint_weights.push_back(0.5);
    _joint_weights.push_back(0.5);
    _joint_weights.push_back(0.5);

    for(unsigned int i=0; i<_n_dofs; i++)
        _joints_cr.push_back(false);

    // We want to receive joint information asynchronously, but don't want to be so bold as
    // to switch every node that uses armlib over to an asynchronous callback model.  Setting
    // up our own CallbackQueue for just this object's pubs/subs lets us do this cleanly.
    ros::NodeHandle nh;
    nh.setCallbackQueue(&_cb_queue);

    _joint_target_pub = nh.advertise<gambit_msgs::JointTargets>("/hybrid/joint_targets", 1); 
    _joint_encoder_sub = nh.subscribe("/hybrid/joint_state", 1, &Hybrid::joint_encoder_cb, this);
    _spinner.start();

    get_encoder_pos(_commanded_pos);
}

void Hybrid::joint_encoder_cb(const gambit_msgs::JointStateConstPtr &joints) {
    assert(joints->positions.size() == _n_dofs);
    lock();
    for(unsigned int i=0; i<_n_dofs; i++)
        _encoder_position[i] = joints->positions[i];
    if(_at_target && !_closed_loop_mode)
        _commanded_pos = _encoder_position;
    _data_valid = true;
    unlock();
}

bool Hybrid::get_encoder_pos(js_vect &position) {
    _data_valid = false;
    while(!_data_valid) {
        if(_should_terminate || !ros::ok()) return false;
        usleep(1000);
    }
    lock();
    position = _encoder_position;
    unlock();
    return true;
}

bool Hybrid::set_target_pos(js_vect &position) {
    assert(position.size() == _n_dofs);
    gambit_msgs::JointTargets jv;
    jv.header.stamp = ros::Time::now();
    for(unsigned int i=0; i<_n_dofs; i++) {
        jv.indices.push_back(i);
        jv.targets.push_back(position[i]);
    }
    _joint_target_pub.publish(jv);
    return true;
}

bool Hybrid::check_joint_limits(js_vect &position) {
    assert(position.size() >= _n_dofs);
    return (
        position[0] >= -2.6 && position[0] <= 2.6 &&
        position[1] >= -2.0 && position[1] <= 2.0 &&
        position[2] >= -2.8 && position[2] <= 2.8 &&
        position[3] >= -0.9 && position[2] <= M_PI &&
        position[4] >= -2.618 && position[4] <= 2.618 &&
        position[5] >= -1.571 && position[5] <= 1.571 &&
        position[6] >= -2.618 && position[6] <= 2.618 &&
        position[7] >= -0.530 && position[7] <= 2.618
    );
}

bool Hybrid::check_arm_joint_limits(js_vect &position) {
    assert(position.size() >= _n_arm_dofs);
    return (
        position[0] >= -2.6 && position[0] <= 2.6 &&
        position[1] >= -2.0 && position[1] <= 2.0 &&
        position[2] >= -2.8 && position[2] <= 2.8 &&
        position[3] >= -0.9 && position[2] <= M_PI &&
        position[4] >= -2.618 && position[4] <= 2.618 &&
        position[5] >= -1.571 && position[5] <= 1.571 &&
        position[6] >= -2.618 && position[6] <= 2.618 
    );
}

bool Hybrid::check_manip_joint_limits(js_vect &position) {
    assert(position.size() >= _n_manip_dofs);
    return position[0] >= -0.530 && position[0] <= 2.618;
}

bool Hybrid::inverse_kinematics(double *position, double *orientation, 
    std::vector<js_vect> &solutions) {
    
    ik_hybrid::IKReal eetrans[3];
    ik_hybrid::IKReal eerot[9];
    ik_hybrid::IKReal vfree[1] = {0.0};
    solutions.clear();

    for(unsigned int i=0; i<3; i++)
        eetrans[i] = position[i];
    for(unsigned int i=0; i<9; i++)
        eerot[i] = orientation[i];

    std::vector<ik_hybrid::IKSolution> vsolutions;
    bool ik_success = ik_hybrid::ik(eetrans, eerot, vfree, vsolutions);
    if(!ik_success) {
        return false;
    }

    std::vector<ik_hybrid::IKReal> sol(_n_arm_dofs);
    for(unsigned int i=0; i<vsolutions.size(); i++) {
        std::vector<ik_hybrid::IKReal> vsolfree(vsolutions[i].GetFree().size());
        vsolutions[i].GetSolution(&sol[0], &vsolfree[0]);
        js_vect js_sol;
        for(unsigned int j=0; j<_n_arm_dofs; j++)
            js_sol.push_back(sol[j]);
        if(check_arm_joint_limits(js_sol))
            solutions.push_back(js_sol);
    }

    return !solutions.empty();
}

}

