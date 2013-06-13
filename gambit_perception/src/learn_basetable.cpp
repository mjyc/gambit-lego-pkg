#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/cv_tableimgproc.h>


class BaseTableLearnerNode : public ImgDepSyncNode {
protected:
    static std::string default_params_path_;

    std::string params_path_;
    BaseTableLearner::Ptr BTLearner_;

public:
    BaseTableLearnerNode() {
        priv_nh_.param<std::string>("params_path", params_path_, default_params_path_);
        BTLearner_ = boost::make_shared<BaseTableLearner>(params_path_);
    }

    virtual ~BaseTableLearnerNode() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        if (BTLearner_->computeBaseTable(bgrFrame_, depthFrame_))
            ros::shutdown();
        // image gets published with 30fps, no need to delay from here
        cv::waitKey(1);
    }
};
std::string BaseTableLearnerNode::default_params_path_ = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "learn_basetable");
    BaseTableLearnerNode btlearnernode;
    ros::spin();
}

