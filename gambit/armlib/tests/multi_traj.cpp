#include <ros/ros.h>
#include <armlib/WAM.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "go_to_zero");
	ros::Time::init();
    armlib::WAM w;
    
    ros::Rate r(10.0);

    armlib::js_vect zeropos;
    for(unsigned int i=0; i<7; i++) {
        zeropos.push_back(0.0);
    }

    armlib::js_vect homepos;
    homepos.push_back(0.0);
    homepos.push_back(-2.0);
    homepos.push_back(0.0);
    homepos.push_back(M_PI);
    homepos.push_back(0.0);
    homepos.push_back(0.0);
    homepos.push_back(0.0);

    w.go_to(homepos);
    w.go_to(zeropos);
    w.go_to(homepos);
    w.go_to_sync(zeropos);
    ros::shutdown();
}

