#include <armlib/recorded_trajectory.h>
#include <ros/console.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>


namespace armlib {

RecordedTrajectory::RecordedTrajectory(const char *filename) {
    FILE *f = fopen(filename, "r");
    char buf[1024];

    if(f == NULL) {
        ROS_FATAL("Unable to open trajectory file %s", filename);
        throw -1;
    }

    int n;
    char *ret = fgets(buf, 1024, f);
    if(ret == NULL || strlen(buf) < 11 || strncmp(buf, "TRAJECTORY", 10) != 0) {
        ROS_FATAL("Invalid trajectory file header (%s)", filename);
        throw -1;
    }

    n = sscanf(buf+11, "%u %lf", &_n_dofs, &_timestep);
    if(n<2) {
        ROS_FATAL("Invalid trajectory file header (%s)", filename);
        throw -1;
    }

    while(!feof(f)) {
        ret = fgets(buf, 1024, f);
        if(ret == NULL) break;
        char *cur;
        int bytes_read;
        cur = buf;
        Eigen::VectorXf p(_n_dofs);
        for(unsigned int i=0; i<_n_dofs; i++) {
            n = sscanf(cur, "%f%n", &p[i], &bytes_read);
            if(n<1) {
                ROS_FATAL("Not enough joints in trajectory (%s)", filename);
                throw -1;
            }
            cur += bytes_read;
        }
        _points.push_back(p);
    }
    _n_points = _points.size();
}


double RecordedTrajectory::get_length() {
    return _n_points * _timestep;
}

void RecordedTrajectory::get_start(js_vect &joints) {
    assert(_n_points > 0);
    joints.resize(_n_dofs);
    for(unsigned int i=0; i<_n_dofs; i++)
        joints[i] = _points[0][i];
}

void RecordedTrajectory::get_end(js_vect &joints) {
    assert(_n_points > 0);
    joints.resize(_n_dofs);
    for(unsigned int i=0; i<_n_dofs; i++)
        joints[i] = _points[_n_points-1][i];
}

void RecordedTrajectory::evaluate(double t, js_vect &joints) {
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

unsigned int RecordedTrajectory::get_num_dofs() {
    return _n_dofs;
}

void RecordedTrajectory::print() {
    printf("Trajectory: %d points, lasting %05.02f seconds\n", _n_points,
        get_length());
    for(unsigned int i=0; i<_n_points; i++) {
        for(unsigned int j=0; j<_n_dofs; j++) {
            printf("%07.02f ", _points[i][j] * 180.0 / M_PI);
        }
        printf("\n");
    }
}

std::string RecordedTrajectory::get_name() {
    return _filename;
}

}

