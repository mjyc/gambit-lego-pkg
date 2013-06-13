#include <ros/ros.h>
// #include <armlib/WAM.h>
// #include <armlib/hybrid.h>
#include <armlib/gambit.h>
#include <armlib/arm.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>

int main(int argc, char **argv) {
    if(argc < 2) {
        printf("usage: %s <robot> <dofs>, where robot is one of {wam,hybrid,gambit}\n", argv[0]);
        exit(-1);
    }

    ros::init(argc, argv, "go_to_zero");

    ros::Time::init(); //since cturtle
    
    armlib::Arm *a;
    unsigned int n_dofs;

    // if(argv[1][0] == 'w') {
    //     a = new armlib::WAM();
    //     n_dofs = 7;
    // } else if(argv[1][0] == 'h') {
    //     a = new armlib::Hybrid();
    //     n_dofs = 8;
    // } else 
		if(argv[1][0] == 'g') {
        a = new armlib::Gambit();
        n_dofs = 7;
    } else {
        printf("invalid robot type\n");
        exit(-1);
    }

    
    armlib::js_vect pos;
    int i;
    for(i=2; i<argc; i++) {
        pos.push_back(atof(argv[i]) * M_PI / 180.0);
    }

    while(pos.size() < n_dofs)
        pos.push_back(0.0);

	for (size_t i = 0; i < pos.size(); i++) {
	  std::cout << pos[i] << " " << std::endl;
	}

    a->go_to_sync(pos);
    ros::shutdown();
}

