#include <iostream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/thread/locks.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "gambit_manip/arm_interface.h"

class ThreadBase {
private:
    boost::shared_ptr<boost::thread> m_thread_ptr;

public:
    ThreadBase() : m_thread_ptr() { }
    virtual ~ThreadBase() { }

    virtual void run() = 0;

    void start() {
        if (m_thread_ptr == NULL)
            m_thread_ptr.reset(
                        new boost::thread(
                            boost::lambda::bind(&ThreadBase::run, this)));
        else
            throw std::runtime_error("multiple start");
    }

    void join() {
        if (m_thread_ptr)
            m_thread_ptr->join();
    }

    void interrupt() {
        if (m_thread_ptr)
            m_thread_ptr->interrupt();
    }
};

class UserInterface: public ThreadBase {
public:
    boost::shared_ptr<ManipArmIF> aifPtr_;

    UserInterface(boost::shared_ptr<ManipArmIF> aifPtr)
        : aifPtr_(aifPtr)
    {
        cv::namedWindow("controller");
    }

    boost::shared_ptr<ManipArmIF> getHandle() {
        return aifPtr_;
    }

    void run() {
        while (true) {
            int key = cv::waitKey(30);
            if ((unsigned char) key == 'q')
                aifPtr_->stop();
        }
    }
};


int main (int argc, char** argv){

    ros::init(argc, argv, "go_to_known_pos");
    ros::NodeHandle nh;

    ROS_INFO("go to given position in 1.0 sec.");
    //ROS_INFO_STREAM("input: " << argv[1][0]);
    ROS_INFO("WARNING - Double check initial position.");
    ROS_INFO("WARNING - NO SAFETY CHECK!");

    sleep(1);
    SpeedParams params;
    params.gotoXSpeed = 0.5;
    params.windingSpeed = 1.5;
    params.windingSpeed2 = 1.0;
    boost::shared_ptr<ManipArmIF> aifPtr = boost::make_shared<ManipArmIF>(nh,params);
    UserInterface uif(aifPtr);
    uif.start();
    for (int i = 0; i < 10; ++i) {
        aifPtr->go_to_back();
        aifPtr->go_to_front();
    }
    uif.join();
}

