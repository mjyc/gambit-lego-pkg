#include <armlib/js_linear_trajectory.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>


namespace armlib {

JSLinearTrajectory::JSLinearTrajectory(js_vect &start, js_vect &end, js_vect &joint_weights,
    double vmax, double amax, double timestep) {
    _n_dofs = start.size();
    _timestep = timestep;
    assert(_n_dofs > 0 && _n_dofs == end.size() && _n_dofs == joint_weights.size());

    Eigen::VectorXf q_start(_n_dofs);
    Eigen::VectorXf q_end(_n_dofs);
    Eigen::VectorXf weights(_n_dofs);

    for(unsigned int i=0; i<_n_dofs; i++) {
        q_start[i] = start[i];
        q_end[i] = end[i];
        weights[i] = joint_weights[i];
    }

    Eigen::VectorXf dx= q_end - q_start;
	// Eigen2.0 code
    // Eigen::VectorXf dxw = dx.cwise() * weights;
    // Eigen3.0 code
    Eigen::VectorXf dxw = dx.array() * weights.array();
    
	// Eigen2 to Eigen3 change
    // float dx_max = dxw.cwise().abs().maxCoeff();
	float dx_max = dxw.array().abs().maxCoeff();
    int num_const_vel_steps = (dx_max/vmax - vmax/amax)/_timestep + 1;

    std::vector<float> traj_scale_list;

    if(num_const_vel_steps < 1) {
        unsigned int num_accel_steps = (sqrt(dx_max/amax) / 2.0 / _timestep) + 1;

        // speed up segment
        for(unsigned int i=0; i<num_accel_steps; i++) {
            float t = (float)(i+1) / (float)num_accel_steps;
            traj_scale_list.push_back(0.5*t*t);
        }

        // slow down segment
        for(unsigned int i=num_accel_steps; i>0; i--) {
            float t = (float)(i-1) / (float)num_accel_steps;
            traj_scale_list.push_back(1.0-0.5*t*t);
        }
    } else {
        unsigned int num_accel_steps = vmax/amax/_timestep + 1;
        float x1 = (0.5 * num_accel_steps)/(num_accel_steps + num_const_vel_steps);
        float x2 = 1.0 - x1;
        traj_scale_list.push_back(0.0);

        // speed up segment
        for(unsigned int i=0; i<num_accel_steps; i++) {
            float t = (float)(i+1) / (float)num_accel_steps;
            traj_scale_list.push_back(x1*t*t);
        }

        // constant velocity segment
        for(unsigned int i=0; i<(unsigned)num_const_vel_steps; i++) {
            float t = (i+1) / (float)num_const_vel_steps;
            traj_scale_list.push_back(x1 + (x2 - x1)*t);
        }

        // slow down segment
        for(unsigned int i=num_accel_steps; i>0; i--) {
            float t = (float)(i-1) / (float)num_accel_steps;
            traj_scale_list.push_back(1.0-(1-x2)*t*t);
        }
    }

    for(unsigned int i=0; i<traj_scale_list.size(); i++) {
        _points.push_back(q_start + dx * traj_scale_list[i]);
    }

    _n_points = _points.size();
}

double JSLinearTrajectory::get_length() {
    return _n_points * _timestep;
}

void JSLinearTrajectory::get_start(js_vect &joints) {
    assert(_n_points > 0);
    joints.resize(_n_dofs);
    for(unsigned int i=0; i<_n_dofs; i++)
        joints[i] = _points[0][i];
}

void JSLinearTrajectory::get_end(js_vect &joints) {
    assert(_n_points > 0);
    joints.resize(_n_dofs);
    for(unsigned int i=0; i<_n_dofs; i++)
        joints[i] = _points[_n_points-1][i];
}

void JSLinearTrajectory::evaluate(double t, js_vect &joints) {
    assert(_n_points > 0);
    if(t < 0.0) {
        get_start(joints);
    } else if(t >= get_length()) {
        get_end(joints);
    } else {
        joints.resize(_n_dofs);
        float pos = t / _timestep;
        unsigned int i = (unsigned int)pos;
        if(i >= _n_points) {
            get_end(joints);
            return;
        }
        float d = pos - i;
        Eigen::VectorXf p0 = _points[i];
        Eigen::VectorXf p1;
        //printf("%d %d %d\n", _n_points, i, i+1);
        if(i+1 < _n_points) p1 = _points[i+1];
        else p1 = _points[i];

        for(unsigned int j=0; j<_n_dofs; j++) {
            joints[j] = (p0[j] * (1.0-d)) + (p1[j] * d);
        }
    }
}

unsigned int JSLinearTrajectory::get_num_dofs() {
    return _n_dofs;
}

void JSLinearTrajectory::print() {
    printf("Trajectory: %d points, lasting %05.02f seconds\n", _n_points,
        get_length());
    for(unsigned int i=0; i<_n_points; i++) {
        for(unsigned int j=0; j<_n_dofs; j++) {
            printf("%07.02f ", _points[i][j] * 180.0 / M_PI);
        }
        printf("\n");
    }
}

std::string JSLinearTrajectory::get_name() {
    return std::string("JS Linear Trajectory");
}

}

