#include <armlib/gambit.h>
#include "ik_gambit.cpp"

namespace armlib {

Gambit::Gambit() :
    		_spinner(1, &_cb_queue)
{
	_latency_usec = 250000;
	_n_dofs = 7;
	_n_arm_dofs = 6;
	_n_manip_dofs = 1;
	_data_valid = false;
	_encoder_position.resize(_n_dofs);

	_joint_weights.push_back(1.5);
	_joint_weights.push_back(1.5);
	_joint_weights.push_back(1.0);
	_joint_weights.push_back(0.5);
	_joint_weights.push_back(0.5);
	_joint_weights.push_back(0.5);
	_joint_weights.push_back(0.5);

	_joints_cr.push_back(true);
	_joints_cr.push_back(true);
	_joints_cr.push_back(true);
	_joints_cr.push_back(false);
	_joints_cr.push_back(false);
	_joints_cr.push_back(false);
	_joints_cr.push_back(false);

	ros::NodeHandle nh;
	nh.setCallbackQueue(&_cb_queue);
	_joint_target_pub = nh.advertise<gambit_msgs::JointTargets>("/arm/joint_targets", 1);
	_joint_encoder_sub = nh.subscribe("/arm/joint_state", 1, &Gambit::joint_encoder_cb, this);
	_spinner.start();

	get_encoder_pos(_commanded_pos);
}

void Gambit::joint_encoder_cb(const gambit_msgs::JointStateConstPtr &joints) {
	assert(joints->positions.size() == _n_dofs);
	lock();
	for(unsigned int i=0; i<_n_dofs; i++)
		_encoder_position[i] = joints->positions[i];
	if(_at_target && !_closed_loop_mode)
		_commanded_pos = _encoder_position;
	_data_valid = true;
	unlock();
}

bool Gambit::get_encoder_pos(js_vect &position) {
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

bool Gambit::set_target_pos(js_vect &position) {
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

bool Gambit::check_joint_limits(js_vect &position) {
	assert(position.size() >= _n_dofs);
	return (
			position[3] >= -2.618 && position[3] <= 2.618 &&
			position[4] >= -1.571 && position[4] <= 1.571 &&
			position[5] >= -2.618 && position[5] <= 2.618 &&
			position[6] >= -0.530 && position[6] <= 2.618
	);
}

bool Gambit::check_arm_joint_limits(js_vect &position) {
	assert(position.size() >= _n_arm_dofs);
	return (
			position[3] >= -2.618 && position[3] <= 2.618 &&
			position[4] >= -1.571 && position[4] <= 1.571 &&
			position[5] >= -2.618 && position[5] <= 2.618
	);
}

bool Gambit::check_manip_joint_limits(js_vect &position) {
	assert(position.size() >= _n_manip_dofs);
	return position[0] >= -0.530 && position[0] <= 2.618;
}

bool Gambit::inverse_kinematics(double *position, double *orientation, 
		std::vector<js_vect> &solutions) {

	ik_gambit::IKReal eetrans[3];
	ik_gambit::IKReal eerot[9];
	ik_gambit::IKReal vfree[1] = {0.0};
	solutions.clear();

	for(unsigned int i=0; i<3; i++)
		eetrans[i] = position[i];
	for(unsigned int i=0; i<9; i++)
		eerot[i] = orientation[i];

	std::vector<ik_gambit::IKSolution> vsolutions;
	bool ik_success = ik_gambit::ik(eetrans, eerot, vfree, vsolutions);
	if(!ik_success) {
		//printf("debug: ik failed\n");
		return false;
	}

	std::vector<ik_gambit::IKReal> sol(_n_arm_dofs);
	//printf("debug: %d solutions\n", vsolutions.size());
	for(unsigned int i=0; i<vsolutions.size(); i++) {
		std::vector<ik_gambit::IKReal> vsolfree(vsolutions[i].GetFree().size());
		vsolutions[i].GetSolution(&sol[0], &vsolfree[0]);
		js_vect js_sol;
		//printf("debug: solution: ");
		for(unsigned int j=0; j<_n_arm_dofs; j++) {
			js_sol.push_back(sol[j]);
			//printf("%f ", sol[j]);
		}
		if(check_arm_joint_limits(js_sol)) {
			solutions.push_back(js_sol);
			//printf("(reachable)");
		}
		//printf("\n");
	}

	//printf("debug: &solutions: %p\n", &solutions);
	//printf("debug: solutions.size(): %d\n", solutions.size());
	return !solutions.empty();
}


}

