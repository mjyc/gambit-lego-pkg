#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/cv_tableimgproc.h>


class IMaskNode : public ImgDepSyncNode {
protected:
    static std::string default_params_path_;

    std::string params_path_;
    InteractiveMask::Ptr iMask_;

public:
    IMaskNode() {
        priv_nh_.param<std::string>("params_path", params_path_, default_params_path_);
        iMask_ = boost::make_shared<InteractiveMask>(params_path_);
    }

    virtual ~IMaskNode() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        iMask_->getMask(bgrFrame_, depthFrame_);
        ros::shutdown();
    }
};
std::string IMaskNode::default_params_path_ = "/home/mjyc/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "interactive_mask");
    IMaskNode imasknode;
    ros::spin();
}


