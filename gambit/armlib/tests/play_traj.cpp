#include <ros/ros.h>
#include <armlib/WAM.h>
#include <armlib/hybrid.h>
#include <armlib/arm.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if(argc < 3) {
        printf("usage: %s <robot> <trajfile>, where robot is one of {wam,hybrid}\n", argv[0]);
        exit(-1);
    }

    ros::init(argc, argv, "play_traj");

	ros::Time::init();
    
    armlib::Arm *a;
    unsigned int n_dofs;

    if(argv[1][0] == 'w') {
        a = new armlib::WAM();
        n_dofs = 7;
    } else if(argv[1][0] == 'h') {
        a = new armlib::Hybrid();
        n_dofs = 8;
    } else {
        printf("invalid robot type\n");
        exit(-1);
    }

    
    a->play_trajectory(argv[2]);
    a->wait_until_stopped();
    ros::shutdown();
}

