#include <iostream>
#include <ros/ros.h>
#include "gambit_manip/arm_interface.h"

using namespace std;

int main (int argc, char** argv){

    if(argc < 2) {
        cerr << "usage: " << argv[0] << " <posID>" << endl;
        cerr << "    posID options:" << endl;
        cerr << "        b = back" << endl;
        cerr << "        f = front" << endl;
        cerr << "        m = manip" << endl;
        exit(-1);
    }

    ros::init(argc, argv, "go_to_known_pos");
    ros::NodeHandle nh;

    ROS_INFO("go to given position in 1.0 sec.");
	ROS_INFO_STREAM("input: " << argv[1][0]);
    ROS_INFO("WARNING - Double check initial position.");
    ROS_INFO("WARNING - NO SAFETY CHECK!");

    sleep(1);
    SpeedParams params;
    params.gotoXSpeed = 0.5;
    params.windingSpeed = 1.5;
    params.windingSpeed2 = 1.0;
    ManipArmIF aif(nh,params);

	char posID = argv[1][0];
	if (posID == 'b') {
        aif.go_to_back();
	} else if (posID == 'f') {
		aif.go_to_front();
	} else if (posID == 'm') {
		aif.go_to_manip();
    } else if (posID == 'v') {
        aif.go_to_offview();
    } else {
		ROS_INFO_STREAM("Unknown posID = " << posID);
    }

    sleep(1);
    return 0;
}

