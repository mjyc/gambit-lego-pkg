#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
//#include <pcl/point_types.h>
#include <ros/ros.h>
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/cv_tableimgproc.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class CaptureNode : public ImgDepSyncNode {
protected:
    static std::string default_params_path_;

    std::string params_path_;
    CaptureImages::Ptr capture_;

public:
    CaptureNode() {
        priv_nh_.param<std::string>("params_path", params_path_, default_params_path_);
        capture_ = boost::make_shared<CaptureImages>(params_path_);
    }

    virtual ~CaptureNode() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        PointCloud::Ptr cloudPtr(new PointCloud);
        toPointCloud(bgrFrame_,depthFrame_,model_.fx(),model_.fy(),model_.cx()/2.0,model_.cy()/2.0,cloudPtr);

        if (capture_->update(bgrFrame_, depthFrame_,cloudPtr))
            ros::shutdown();
        cv::waitKey(1);
    }
};
std::string CaptureNode::default_params_path_ = "/home/mjyc/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_images");
    CaptureNode capturenode;
    ros::spin();
}



