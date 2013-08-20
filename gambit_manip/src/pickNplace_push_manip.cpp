#include <iostream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
//#include <opencv2/core/core.hpp> // for user control
#include <opencv2/highgui/highgui.hpp>

#include "gambit_manip/thread_base.h"
#include "gambit_manip/arm_interface.h"

class SimpleUI: public ThreadBase {
public:
    ArmIF* aifPtr_;

    SimpleUI(ArmIF* aifPtr)
        : aifPtr_(aifPtr) {
        cv::namedWindow("SimpleUI");
        std::cout << "[SimpleUI] Press 'Space' to stop the motion" << std::endl;
        std::cout << "[SimpleUI] Press 'Control-C' to stop kill the program" << std::endl;
    }

    void run() {
        int key = cv::waitKey(30) % 256;
        while (ros::ok()) {
            if ((unsigned char) key == ' ') {
                aifPtr_->stop_motion();
                std::cout << "[SimpleUI] User stop called." << std::endl;
            }
            key = cv::waitKey(30) % 256;
        }
    }
};



int main(int argc, char** argv) {

    if(argc < 2) {
        std::cerr << "usage: " << argv[0] << " <posID>" << std::endl;
        std::cerr << "    posID options:" << std::endl;
        std::cerr << "        b = back" << std::endl;
        std::cerr << "        f = front" << std::endl;
        std::cerr << "        m = manip" << std::endl;
        std::cerr << "        v = offview" << std::endl;
        exit(-1);
    }

    ros::init(argc, argv, "pickNplace_push_manip");
    ros::NodeHandle nh;

    char posID = argv[1][0];
    ROS_INFO_STREAM("Input initial state: " << posID);
    ROS_INFO("WARNING - Double check initial position.");
    ROS_INFO("Program ready in 1.0 sec.");
    sleep(1);

    SpeedParams params;
    params.gotoXSpeed = 0.3;
    params.windingSpeed = 1.3;
    params.windingSpeed2 = 0.7;
    params.hoverSpeed = 0.75;
    params.manipSpeed = 0.2;
    params.offviewSpeed = 0.7;

    boost::shared_ptr<BasicManipStateMachine> manipStateMachine = boost::make_shared<BasicManipStateMachine> (nh,posID);
    manipStateMachine->setSpeed(params);
    SimpleUI simpleUI(manipStateMachine.get());
    simpleUI.start();
    ros::spin();
    simpleUI.join();

    return 0;
}
