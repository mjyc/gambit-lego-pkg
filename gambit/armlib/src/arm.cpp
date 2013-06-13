#include <armlib/arm.h>
#include <armlib/js_linear_trajectory.h>
#include <armlib/recorded_trajectory.h>
#include <armlib/geom.h>

#include <Eigen/Core>
#include <assert.h>
#include <float.h>

namespace armlib {

Arm::Arm() :
    _max_velocity(1.2),
    _max_accel(0.75),
    _hard_limit(2.0),
    _control_loop_rate(1.0 / update_period)
{
    _latency_usec = 0;
    _should_terminate = false;

    _closed_loop_mode = false;
    _at_target = true;
    _cur_traj = NULL;

    pthread_mutex_init(&_lock, NULL);
    pthread_create(&_control_thread, NULL, Arm::control_thread_loop, this);

    //stupid workaround - good enough?
	ros::NodeHandle foo; //workaround
}

void Arm::set_limits(float vlimit, float alimit, float hard_vlimit) {
    _max_velocity = vlimit;
    _max_accel = alimit;
    _hard_limit = hard_vlimit;
}

Arm::~Arm() {
    stop();
}

void Arm::get_actual_joint_pos(js_vect &position) {
    get_encoder_pos(position);
}

void Arm::stop() {
    void *ret;

    _should_terminate = true;
    pthread_join(_control_thread, &ret);
}

void *Arm::control_thread_loop(void *obj) {
    ROS_DEBUG("Starting arm control loop");
    Arm *self = (Arm*)obj;
    while(!self->_should_terminate) {
        self->lock();

        double tnow = ros::Time::now().toSec();
        // Check on the trajectory queue, performing maintenance and grabbing the next 
        // trajectory if we're ready for it
        if(self->_cur_traj == NULL || 
            tnow - self->_traj_start_time > self->_cur_traj->get_length()) {
            if(!self->_traj_queue.empty()) {
                // no trajectory is active, but there are some in the queue, so we grab
                // the next one and start executing it
                if(self->_cur_traj != NULL) delete self->_cur_traj;
                self->_cur_traj = self->_traj_queue.front();
                self->_traj_queue.pop_front();
                self->_at_target = false;
                self->_traj_start_time = tnow;
            } else {
                // no trajectory running or trajectory has finished; make sure we release
                // the associated memory and set _at_target
                if(self->_cur_traj != NULL) {
                    delete self->_cur_traj;
                    self->_cur_traj = NULL;
                }
                self->_at_target = true;
            }
        }

        // If there is a currently running trajectory, perform the next step in executing it
        if(self->_cur_traj != NULL) {
            js_vect step;
            self->_cur_traj->evaluate(tnow - self->_traj_start_time, step);

            Eigen::VectorXf p0(self->_n_dofs);
            Eigen::VectorXf p1(self->_n_dofs);
            for(unsigned int i=0; i<self->_n_dofs; i++) {
                p0[i] = self->_commanded_pos[i];
                p1[i] = step[i];
            }

            Eigen::VectorXf dx = p1 - p0;
			// Eigen 2.0 style
			// if(dx.cwise().abs().maxCoeff() > self->_hard_limit) {
			// Eigen 3.0 style
            if(dx.array().abs().maxCoeff() > self->_hard_limit) {
                ROS_ERROR("Hard velocity limit exceeded; dropping motion step");
                self->unlock();
                continue;
            }

            self->_commanded_pos = step;
            self->set_target_pos(step);
        }

        self->unlock();
        self->_control_loop_rate.sleep();
    }
    pthread_exit(NULL);
}

void Arm::stop_motion() {
    lock();

    if(_cur_traj) delete _cur_traj;
    _cur_traj = NULL;

    while(!_traj_queue.empty()) {
        Trajectory *t = _traj_queue.front();
        _traj_queue.pop_front();
        delete t;
    }

    _at_target = true;
    _closed_loop_mode = false;

    unlock();
}

void Arm::set_closed_loop_mode(bool mode) {
    lock();
    if(mode)
        stop_motion();
    _closed_loop_mode = mode;
    unlock();
}

void Arm::move_toward(js_vect &target, float vlimit) {
    assert(target.size() == _n_dofs);
    if(vlimit < 0) vlimit = _max_velocity;
    lock();
    if(!_closed_loop_mode || _cur_traj != NULL) {
        unlock();
        return;
    }

    Eigen::VectorXf dx(_n_dofs);

    for(unsigned int i=0; i<_n_dofs; i++) {
       dx[i] = target[i] - _commanded_pos[i];
    }   

    // Eigen 2.0 style
    // float vmax = dx.cwise().abs().maxCoeff();
    // Eigen 3.0 style
    float vmax = dx.array().abs().maxCoeff();
    if(vmax > vlimit * update_period) {
        dx = (dx / vmax) * (vlimit * update_period);
    }

    for(unsigned int i=0; i<_n_dofs; i++) {
        _commanded_pos[i] += dx[i];
    }

    set_target_pos(_commanded_pos);
    _at_target = true;
    unlock();
}

void Arm::go_to(js_vect &target, dir_vect &preferred_directions, float vlimit, float alimit) {
    assert(target.size() == _n_dofs);
    if(vlimit < 0) vlimit = _max_velocity;
    if(alimit < 0) alimit = _max_accel;
    lock();
    
    if(_closed_loop_mode) {
        unlock();
        return;
    }
    
    js_vect start_pos;
    if(_traj_queue.size() > 0) {
        _traj_queue.back()->get_end(start_pos);
    } else if(!_at_target && _cur_traj != NULL) {
        _cur_traj->get_end(start_pos);
    } else {
        start_pos = _commanded_pos;
    }
    unlock();

    /* Handle continuous rotation */
    for(unsigned int i=0; i<_n_dofs; i++) {
        if(_joints_cr[i]) {
            double a1 = atan2(sin(start_pos[i]), cos(start_pos[i]));
            double a2 = atan2(sin(target[i]), cos(target[i]));
            double da = a2 - a1;
            if(i >= preferred_directions.size() || preferred_directions[i] == R_SHORTEST
                || fabs(da) < SHORTEST_ANGLE_TOLERANCE) {
                while(da < -M_PI) da += M_PI * 2;
                while(da > M_PI) da -= M_PI * 2;
                target[i] = start_pos[i] + da;
            } else if(preferred_directions[i] == R_NEGATIVE) {
                while(da > 0) da -= M_PI * 2;
                target[i] = start_pos[i] + da;
            } else if(preferred_directions[i] == R_POSITIVE) {
                while(da < 0) da += M_PI * 2;
                target[i] = start_pos[i] + da;
            }
        }
    }
    
    /* Compute trajectory */
    JSLinearTrajectory *t = new JSLinearTrajectory(start_pos, target, _joint_weights,
        vlimit, alimit, update_period);

    lock();
    _traj_queue.push_back(t);
    _at_target = false;
    unlock();
}

void Arm::go_to(js_vect &target, float vlimit, float alimit) {
    dir_vect directions;
    for(unsigned int i=0; i<_n_dofs; i++)
        directions.push_back(R_SHORTEST);
    go_to(target, directions, vlimit, alimit);
}

void Arm::go_to_sync(js_vect &target, float vlimit, float alimit) {
    go_to(target, vlimit, alimit);
    wait_until_stopped();
}

js_vect Arm::closest_point(std::vector<js_vect> &candidates, js_vect &reference,
    dir_vect &preferred_directions) {
    assert(candidates.size() > 0);
    double min_dist = DBL_MAX;
    js_vect *closest = NULL;

    for(unsigned int i=0; i<candidates.size(); i++) {
        double sum = 0.0;
        js_vect &a = candidates[i];
        js_vect &b = reference;
        assert((a.size() <= b.size()) && (a.size() <= _n_dofs));
        for(unsigned int j=0; j<a.size(); j++) {
            if(_joints_cr[j]) {
                double da = geom::angle_difference(a[j], b[j]);
                if(i >= preferred_directions.size() || preferred_directions[i] == R_SHORTEST
                    || fabs(da) < SHORTEST_ANGLE_TOLERANCE) {
                    while(da < -M_PI) da += M_PI * 2;
                    while(da > M_PI) da -= M_PI * 2;
                } else if(preferred_directions[i] == R_NEGATIVE) {
                    while(da > 0) da -= M_PI * 2;
                } else if(preferred_directions[i] == R_POSITIVE) {
                    while(da < 0) da += M_PI * 2;
                }
                sum += pow(da, 2.0);
            } else {
                sum += pow(a[j] - b[j], 2.0);
            }
        }
        double dist = sqrt(sum);
        if(dist < min_dist) {
            min_dist = dist;
            closest = &candidates[i];
        }
    }

    return *closest;
}

js_vect Arm::closest_point(std::vector<js_vect> &candidates, js_vect &reference) {
    dir_vect dir;
    for(unsigned int i=0; i<reference.size(); i++)
        dir.push_back(R_SHORTEST);
    return closest_point(candidates, reference, dir);
}

void Arm::play_trajectory(const char *filename) {
    RecordedTrajectory *t;

    lock();
    if(_closed_loop_mode) {
        unlock();
        return;
    }

    js_vect start_pos;
    if(_traj_queue.size() > 0) {
        _traj_queue.back()->get_end(start_pos);
    } else if(!_at_target && _cur_traj != NULL) {
        _cur_traj->get_end(start_pos);
    } else {
        start_pos = _commanded_pos;
    }

    unlock();

    try {
        t = new RecordedTrajectory(filename);
    } catch(int e) {
        ROS_ERROR("Error reading trajectory, will not be played");
        return;
    }
    
    js_vect traj_start;
    t->get_start(traj_start);

    JSLinearTrajectory *pre = new JSLinearTrajectory(start_pos, traj_start, _joint_weights,
        _max_velocity, _max_accel);

    lock();
    _traj_queue.push_back(pre);
    _traj_queue.push_back(t);
    _at_target = false;
    unlock();

}

void Arm::wait_until_stopped(bool include_latency) {
    while(!_at_target && ros::ok())
        usleep(1000);
    if(include_latency)
        usleep(_latency_usec);
}

bool Arm::inverse_kinematics(double x, double y, double z, double roll, double pitch,
    double yaw, std::vector<js_vect> &solutions) {
    
    double position[3] = {x, y, z};
    double orientation[9];

    geom::rot_matrix_from_rpy(orientation, roll, pitch, yaw);
    return inverse_kinematics(position, orientation, solutions);
}

}

