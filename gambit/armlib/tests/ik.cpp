#include <ros/ros.h>
#include <armlib/WAM.h>
#include <armlib/hybrid.h>
#include <armlib/gambit.h>
#include <armlib/arm.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if(argc < 8) {
        printf("usage: %s robot x y z roll pitch yaw, where robot is one of {wam,hybrid,gambit}\n", argv[0]);
        exit(-1);
    }

    ros::init(argc, argv, "ik_test");
    
	ros::Time::init();

    armlib::Arm *a;
    unsigned int n_dofs;

    if(argv[1][0] == 'w') {
        a = new armlib::WAM();
        n_dofs = 7;
    } else if(argv[1][0] == 'h') {
        a = new armlib::Hybrid();
        n_dofs = 8;
    } else if(argv[1][0] == 'g') {
        a = new armlib::Gambit();
        n_dofs = 7;
    } else {
        printf("invalid robot type\n");
        exit(-1);
    }

    double x = atof(argv[2]);
    double y = atof(argv[3]);
    double z = atof(argv[4]);
    double roll = atof(argv[5]);
    double pitch = atof(argv[6]);
    double yaw = atof(argv[7]);

    armlib::js_vect current_pos;
    a->get_actual_joint_pos(current_pos);

    printf("current:  ");
    for(unsigned int j=0; j<current_pos.size(); j++) {
        printf("% 6.01f ", current_pos[j] * 180.0 / M_PI);
    }
    printf("\n");

    std::vector<armlib::js_vect> solutions;
    bool ik_success = a->inverse_kinematics(x, y, z, roll, pitch, yaw, solutions);

    if(ik_success) {
        for(unsigned int i=0; i<solutions.size(); i++) {
            printf("solution: ");
            armlib::js_vect *sol = &solutions[i];
            for(unsigned int j=0; j<sol->size(); j++) {
                printf("% 6.01f ", sol->at(j) * 180.0 / M_PI);
            }
            printf("\n");
        }

        armlib::js_vect target = a->closest_point(solutions, current_pos);
        
        printf("selected: ");
        for(unsigned int j=0; j<target.size(); j++) 
            printf("% 6.01f ", target[j] * 180.0 / M_PI);
        printf("\n");

        for(unsigned int i=0; i<a->get_n_manip_dofs(); i++)
            target.push_back(0.0);

        a->go_to_sync(target);
    } else {
        printf("No solutions found.\n");
    }
    
    ros::shutdown();
}

