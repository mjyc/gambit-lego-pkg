#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp> // for FPS_CALC_CV
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/tableevent_detection.h>
#include <gambit_perception/handtracking.h>

#define FPS_CALC_CV(_WHAT_) \
do \
{ \
    static int64 count = 0;\
    static double last = cv::getTickCount() / cv::getTickFrequency();\
    if (++count == 100) \
    { \
        double now = cv::getTickCount() / cv::getTickFrequency(); \
        std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
        count = 0; \
        last = now; \
    } \
}while(false)



class EventMainNode : public ImgDepSyncNode {
protected:
    static std::string default_params_path_;

    cv::string params_path_;
    cv::Mat mask_;
    cv::Mat tableModel_;

public:
    EventMainNode() {
        priv_nh_.param<std::string>("params_path", params_path_, default_params_path_);
        mask_ = cv::imread(params_path_ + "mask.png", -1);
        tableModel_ = cv::imread(params_path_ + "baseTable.png", -1);
    }

    virtual ~EventMainNode() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        cv::Mat img = bgrFrame_;
        cv::Mat dep = depthFrame_;

        FPS_CALC_CV("mainloop");
        int key = cv::waitKey(1) % 256;
        if (key == 27)
            ros::shutdown();
    }
};
std::string EventMainNode::default_params_path_ = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "learn_basetable");
    EventMainNode evMainNode;
    ros::spin();
}
