#include <ros/ros.h>
#include <armlib/WAM.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "print_positions");
	ros::Time::init();
    armlib::WAM w;
    
    ros::Rate r(10.0);

    while(ros::ok()) {
        std::vector<float> positions;
        w.get_actual_joint_pos(positions);
        for(unsigned int i=0; i<positions.size(); i++) {
            printf("%05.01f ", positions[i] * 180. / M_PI);
        }
        printf("\n");
        r.sleep();
    }
}

