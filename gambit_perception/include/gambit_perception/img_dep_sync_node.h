#ifndef IMG_DEP_SYNC_NODE_H
#define IMG_DEP_SYNC_NODE_H

#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gambit_perception/cv_utils.h>

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

class ImgDepSyncNode {

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_, sub_rgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    image_geometry::PinholeCameraModel model_;

    cv::Mat bgrFrame_;
    cv::Mat depthFrame_;

public:
    ImgDepSyncNode() :
        nh_(),
        priv_nh_("~") {

        sub_depth_.subscribe(nh_, "/camera/depth_registered/sw_registered/image_rect", 1);
        sub_rgb_  .subscribe(nh_, "/camera/rgb/image_rect_color", 1);
        sub_info_ .subscribe(nh_, "/camera/rgb/camera_info", 1);

        int queue_size = 5;
        sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
        sync_->registerCallback(boost::bind(&ImgDepSyncNode::imageCb, this, _1, _2, _3));
    }

    virtual ~ImgDepSyncNode() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg_in,
                 const sensor_msgs::CameraInfoConstPtr& info_msg) {

        // Check for bad inputs
        if (depth_msg->header.frame_id != rgb_msg_in->header.frame_id)
        {
            ROS_ERROR("Depth image frame id [%s] doesn't match RGB image frame id [%s]",
                      depth_msg->header.frame_id.c_str(), rgb_msg_in->header.frame_id.c_str());
            return;
        }

        // Update camera model
        model_.fromCameraInfo(info_msg);

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
            model_.fromCameraInfo(info_msg_tmp);

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
        } else
            rgb_msg = rgb_msg_in;

        // Convert to OpenCV cv::Mat
        cv_bridge::CvImagePtr cv_ptr_depth;
        cv_bridge::CvImagePtr cv_ptr_rgb;
        try {
            cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
            // cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
            cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // copy over new data and unit conversion
        bgrFrame_ = cv_ptr_rgb->image;
        toOpenCVDepthMap(cv_ptr_depth->image, depthFrame_);
    }
};


#endif // IMG_DEP_SYNC_NODE_H
