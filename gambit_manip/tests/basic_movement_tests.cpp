#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include "gambit_manip/arm_interface.h"

using namespace std;

int main (int argc, char** argv){

    if(argc < 2) {
        cerr << "usage: " << argv[0] << " <posID>" << endl;
        cerr << "    posID options:" << endl;
        cerr << "        b = back" << endl;
        cerr << "        f = back2front" << endl;
        cerr << "        m = front2manip" << endl;
        cerr << "        i = gotoikpose" << endl;
        cerr << "        l = move_object_to_left" << endl;
        cerr << "        r = move_object_to_right" << endl;
        cerr << "        o = move_object_to_offtable" << endl;
        exit(-1);
    }

    char posID = argv[1][0];
    double yangle,x,y,z;
    tf::Vector3 coords;
    if (posID == 'i' || posID == 'l' || posID == 'r' || posID == 'o') {
        if(argc < 6) {
            cerr << "    options i,l,t,o requires additional 4 doubles - " << endl;
            cerr << "        <yawangle> <x> <y> <z>" << endl;
            exit(-1);
        } else {
            stringstream yangle_str(argv[2]);
            yangle_str >> yangle;
            stringstream x_str(argv[3]);
            x_str >> x;
            stringstream y_str(argv[4]);
            y_str >> y;
            stringstream z_str(argv[5]);
            z_str >> z;

            cout << "parsed doubles: " << endl;
            cout << "    " << yangle << ", " << x << ", " << y << ", " << z << endl;
            coords[0] = x;
            coords[1] = y;
            coords[2] = z;
        }
    }

    ros::init(argc, argv, "basic_movement_tests");
    ros::NodeHandle nh;

    ROS_INFO("go to given position in 1.0 sec.");
    ROS_INFO_STREAM("input: " << argv[1][0]);
    ROS_INFO("WARNING - Double check initial position.");
    ROS_INFO("WARNING - NO SAFETY CHECK!");

    sleep(1);
    SpeedParams params;
    params.gotoXSpeed = 0.3;
    params.windingSpeed = 1.5;
    params.windingSpeed2 = 1.0;
    params.hoverSpeed = 0.4;
    params.manipSpeed = 0.2;
    ManipArmIF aif(nh,params);

    if (posID == 'b') {
        ROS_INFO("calling go_to_back()");
        aif.go_to_back();
    } else if (posID == 'f') {
        ROS_INFO("calling go_from_back_to_front()");
        aif.go_from_back_to_front();
    } else if (posID == 'm') {
        aif.go_to_manip();
        ROS_INFO("calling go_to_manip()");

    } else if (posID == 'i') {
        ROS_INFO("calling go_to_ikpose()");
        cout << "retval = " << aif.go_to_ikpose(yangle,coords,0.3) << endl;
    } else if (posID == 'l') {
        ROS_INFO("calling move_object_to_left()");
        cout << "retval = " << aif.move_object_to_left(yangle,coords) << endl;
    } else if (posID == 'r') {
        ROS_INFO("calling move_object_to_right()");
        cout << "retval = " << aif.move_object_to_right(yangle,coords) << endl;
    } else if (posID == 'o') {
        ROS_INFO("calling move_object_to_offtable()");
        cout << "retval = " << aif.move_object_to_offtable(yangle,coords) << endl;
    } else {
        ROS_INFO_STREAM("Unknown posID = " << posID);
    }

    sleep(1);

    return 0;
}
