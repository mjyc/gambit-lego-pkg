#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include "depth_traits.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gambit_perception {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class SyncImgDepNodelet : public nodelet::Nodelet
{
    // Subscriptions
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_, sub_rgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    // Publications
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_img_;
    image_transport::Publisher image_pub_dep_;

    virtual void onInit();

    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void SyncImgDepNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();

    sub_depth_.subscribe(nh, "/camera/depth_registered/image_rect", 1);
    sub_rgb_  .subscribe(nh, "/camera/rgb/image_rect_color", 1);
    sub_info_ .subscribe(nh, "/camera/rgb/camera_info", 1);

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    int queue_size = 1;
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
    sync_->registerCallback(boost::bind(&SyncImgDepNodelet::imageCb, this, _1, _2, _3));

    it_.reset(new image_transport::ImageTransport(nh));
    image_pub_img_ = it_->advertise("/camera/rgb/image_rect_color_sync", 1);
    image_pub_dep_ = it_->advertise("/camera/depth_registered/image_rect_sync", 1);

    std::cout << "hellow    " << std::endl;
}

void SyncImgDepNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::ImageConstPtr& rgb_msg_in,
                               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    // Check for bad inputs
    if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
    {
        NODELET_ERROR_THROTTLE(5, "Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                               depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
        return;
    }

    // Check if the input image has to be resized
    sensor_msgs::ImageConstPtr rgb_msg = rgb_msg_in;
    if (depth_msg->width != rgb_msg->width || depth_msg->height != rgb_msg->height)
    {
        sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
        info_msg_tmp.width = depth_msg->width;
        info_msg_tmp.height = depth_msg->height;
        float ratio = float(depth_msg->width)/float(rgb_msg->width);
        info_msg_tmp.K[0] *= ratio;
        info_msg_tmp.K[2] *= ratio;
        info_msg_tmp.K[4] *= ratio;
        info_msg_tmp.K[5] *= ratio;
        info_msg_tmp.P[0] *= ratio;
        info_msg_tmp.P[2] *= ratio;
        info_msg_tmp.P[5] *= ratio;
        info_msg_tmp.P[6] *= ratio;

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_bridge::CvImage cv_rsz;
        cv_rsz.header = cv_ptr->header;
        cv_rsz.encoding = cv_ptr->encoding;
        cv::resize(cv_ptr->image.rowRange(0,depth_msg->height/ratio), cv_rsz.image, cv::Size(depth_msg->width, depth_msg->height));
        rgb_msg = cv_rsz.toImageMsg();

        //NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
        //                       depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
        //return;
    } else
        rgb_msg = rgb_msg_in;

    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
        cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("dep", cv_ptr_depth->image);
    cv::imshow("rgb", cv_ptr_rgb->image);
    std::cout << "called" << std::endl;
    cv::waitKey(30);

//    image_pub_img_.publish(*rgb_msg);
//    image_pub_dep_.publish(*depth_msg);
}

} // namespace gambit_perception

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (gambit_perception, sync_img_dep_nodelet, gambit_perception::SyncImgDepNodelet, nodelet::Nodelet);

