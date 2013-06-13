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

// Ported from Jinna's oasis_svn_repo code

namespace gambit_perception {

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

typedef union
{
    struct /*anonymous*/
    {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    };
    float float_value;
    long long_value;
} RGBValue;

class EventMainNodelet : public nodelet::Nodelet
{
    // Subscriptions
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_, sub_rgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    typedef ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    // Publications
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    ros::Publisher pub_point_cloud_;

    image_geometry::PinholeCameraModel model_;

    virtual void onInit();

    void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);

    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::ImageConstPtr& rgb_msg,
                 const PointCloud::Ptr& cloud_msg,
                 int red_offset, int green_offset, int blue_offset, int color_step);

    // Processing related
    cv::Mat bgrFrame_;
    cv::Mat depthFrame_;

    // TODO: take it out to object
    std::string param_path_;
    cv::Mat mask_;
    cv::Mat tablemodel_;
};

void EventMainNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    //ros::NodeHandle& private_nh = getPrivateNodeHandle();

    sub_depth_.subscribe(nh, "/camera/depth_registered/image_rect", 1);
    sub_rgb_  .subscribe(nh, "/camera/rgb/image_rect_color", 1);
    sub_info_ .subscribe(nh, "/camera/rgb/camera_info", 1);

    // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
    int queue_size = 1;
    sync_.reset( new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_) );
    sync_->registerCallback(boost::bind(&EventMainNodelet::imageCb, this, _1, _2, _3));

    pub_point_cloud_ = nh.advertise<PointCloud>("gambit/depth_registered/points", 1);

    //TODO: separate these out later
    //TODO: check if file exist or not
    param_path_ = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params/";
    mask_ = cv::imread(param_path_ + "mask.png");
    tablemodel_ = cv::imread(param_path_ + "baseTable.png");
}

void EventMainNodelet::imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
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

        //NODELET_ERROR_THROTTLE(5, "Depth resolution (%ux%u) does not match RGB resolution (%ux%u)",
        //                       depth_msg->width, depth_msg->height, rgb_msg->width, rgb_msg->height);
        //return;
    } else
        rgb_msg = rgb_msg_in;



    //---- My code ----//
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try {
        cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
        cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    bgrFrame_ = cv_ptr_rgb->image;
	depthFrame_ = cv_ptr_depth->image;
    // toOpenCVDepthMap(cv_ptr_depth->image, depthFrame_);
    // downsampling
    cv::resize(bgrFrame_, bgrFrame_, cv::Size(), 0.5, 0.5);
    cv::resize(depthFrame_, depthFrame_, cv::Size(), 0.5, 0.5);

	//cv::imshow("rgb",bgrFrame_);
	//cv::imshow("depth",depthFrame_);

    //superHandMask(depthFrame_, tablemodel_, mask_);



    // Supported color encodings: RGB8, BGR8, MONO8
    int red_offset, green_offset, blue_offset, color_step;
    if (rgb_msg->encoding == enc::RGB8)
    {
        red_offset   = 0;
        green_offset = 1;
        blue_offset  = 2;
        color_step   = 3;
    }
    else if (rgb_msg->encoding == enc::BGR8)
    {
        red_offset   = 2;
        green_offset = 1;
        blue_offset  = 0;
        color_step   = 3;
    }
    else if (rgb_msg->encoding == enc::MONO8)
    {
        red_offset   = 0;
        green_offset = 0;
        blue_offset  = 0;
        color_step   = 1;
    }
    else
    {
        NODELET_ERROR_THROTTLE(5, "Unsupported encoding [%s]", rgb_msg->encoding.c_str());
        return;
    }

    // Allocate new point cloud message
    PointCloud::Ptr cloud_msg (new PointCloud);
    cloud_msg->header = depth_msg->header; // Use depth image time stamp
    cloud_msg->height = depth_msg->height;
    cloud_msg->width  = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);

    if (depth_msg->encoding == enc::TYPE_16UC1)
    {
        convert<uint16_t>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    }
    else if (depth_msg->encoding == enc::TYPE_32FC1)
    {
        convert<float>(depth_msg, rgb_msg, cloud_msg, red_offset, green_offset, blue_offset, color_step);
    }
    else
    {
        NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
        return;
    }

    pub_point_cloud_.publish (cloud_msg);
}

template<typename T>
void EventMainNodelet::convert(const sensor_msgs::ImageConstPtr& depth_msg,
                               const sensor_msgs::ImageConstPtr& rgb_msg,
                               const PointCloud::Ptr& cloud_msg,
                               int red_offset, int green_offset, int blue_offset, int color_step)
{
    // Use correct principal point from calibration
    float center_x = model_.cx();
    float center_y = model_.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / model_.fx();
    float constant_y = unit_scaling / model_.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    const uint8_t* rgb = &rgb_msg->data[0];
    int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;
    PointCloud::iterator pt_iter = cloud_msg->begin ();

    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step, rgb += rgb_skip)
    {
        for (int u = 0; u < (int)cloud_msg->width; ++u, rgb += color_step)
        {
            pcl::PointXYZRGB& pt = *pt_iter++;
            T depth = depth_row[u];

            // Check for invalid measurements
            if (!DepthTraits<T>::valid(depth))
            {
                pt.x = pt.y = pt.z = bad_point;
            }
            else
            {
                // Fill in XYZ
                pt.x = (u - center_x) * depth * constant_x;
                pt.y = (v - center_y) * depth * constant_y;
                pt.z = DepthTraits<T>::toMeters(depth);
            }

            // Fill in color
            RGBValue color;
            color.Red   = rgb[red_offset];
            color.Green = rgb[green_offset];
            color.Blue  = rgb[blue_offset];
            color.Alpha = 0;
            pt.rgb = color.float_value;
        }
    }
}

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (gambit_perception, event_main_nodelet, gambit_perception::EventMainNodelet, nodelet::Nodelet);
