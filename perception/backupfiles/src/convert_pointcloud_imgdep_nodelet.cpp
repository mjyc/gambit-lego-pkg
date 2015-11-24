#include <iostream>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gambit_perception/depth_traits.h>


namespace gambit_perception
{
	// typedefs
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
	namespace enc = sensor_msgs::image_encodings;


	class ConvertPointCloud2ImgDep : public nodelet::Nodelet
    {
		// members
		ros::Publisher image_pub_;
		ros::Publisher depth_pub_;
        ros::Subscriber sub_;
		PointCloudPtr cloud_; // place holder
		sensor_msgs::ImagePtr rgb_msg_;
		sensor_msgs::ImagePtr depth_msg_;

		void convert2rgbd(const PointCloudConstPtr& cloud_in, \
						  const sensor_msgs::ImagePtr& rgb_msg_out, \
						  const sensor_msgs::ImagePtr& depth_msg_out) {

			// Allocate new Image messages
			depth_msg_out->header   = cloud_in->header;
			depth_msg_out->encoding = enc::TYPE_32FC1; // using pcl rep
			// OpenCV rep uses TYPE_16UC1
			depth_msg_out->height   = cloud_in->height;
			depth_msg_out->width    = cloud_in->width;
			depth_msg_out->step     = cloud_in->width * sizeof (float); // OpenCV rep uses uint16_t
			depth_msg_out->data.resize( depth_msg_out->height * depth_msg_out->step);


			rgb_msg_out->header   = cloud_in->header;
			rgb_msg_out->encoding = "bgr8";
			rgb_msg_out->height   = cloud_in->height;
			rgb_msg_out->width    = cloud_in->width;
			rgb_msg_out->step     = cloud_in->width * sizeof (float) * 3; // 3: bgr
			rgb_msg_out->data.resize( rgb_msg_out->height * rgb_msg_out->step);


			// depth value pointers
			float* depth_row = reinterpret_cast<float*>(&depth_msg_out->data[0]); // pointer to first depth row
			int row_step = depth_msg_out->step / sizeof (float);
			// bgr format
			const uint8_t* rgb = &rgb_msg_out->data[0]; // pointer to first rgb data
			int rgb_skip = rgb_msg_out->step - rgb_msg_out->width * 3;

			PointCloud::iterator pt_iter = cloud_->begin ();
			float bad_point = std::numeric_limits<float>::quiet_NaN ();

			for (int v = 0; v < (int)cloud_->height; ++v, depth_row += row_step, rgb += rgb_skip)
			{
				for (int u = 0; u < (int)cloud_->width; ++u, rgb += 3)
				{
					pcl::PointXYZRGB& pt = *pt_iter++;

					// copy over organized depth data
					//float depth = (!DepthTraits<float>::valid(pt.z)) ? bad_point : (float) pt.z;
					// if (!DepthTraits<float>::valid(pt.z))
					// 	std::cout << pt.z << std::endl;
					//depth_row[u] = depth;
					depth_row[u] = pt.z;

					// copy over rgb data
					uint8_t * pixel = &(rgb_msg_out->data[v * rgb_msg_out->step + u * 3]);
					memcpy (pixel, &(pt.rgb), 3 * sizeof (uint8_t));
				}
			}
		}

		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
			// Convert sensor_msgs::PointCloud2 type to pcl::PointCloud<pcl::PointXYZRGB> type
			//cloud_.reset(new PointCloud);
			pcl::fromROSMsg (*input, *cloud_);
			
			// Do conversion
			//sensor_msgs::ImagePtr rgb_msg( new sensor_msgs::Image );
			//sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
			convert2rgbd(cloud_, rgb_msg_, depth_msg_);
			
			// publish outputs
			depth_pub_.publish(depth_msg_);
			image_pub_.publish(rgb_msg_);
		}
		
		virtual void onInit() {
			// Initialize ROS
            ros::NodeHandle& nh = getNodeHandle();

			// Create a ROS subscriber for the input point cloud
            sub_ = nh.subscribe ("/camera/depth_registered/points", 1, &ConvertPointCloud2ImgDep::cloud_cb, this);

			// Create a ROS publisher for the output point cloud
			image_pub_ = nh.advertise<sensor_msgs::Image> ("/camera/rgb/image_rect_color_sync", 1);
			depth_pub_ = nh.advertise<sensor_msgs::Image> ("/camera/depth_registered/image_rect_sync", 1);

			cloud_.reset(new PointCloud);
			rgb_msg_.reset( new sensor_msgs::Image );
			depth_msg_.reset( new sensor_msgs::Image );
		}    
    };
	
}


// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (gambit_perception, convert_pointcloud_imgdep, gambit_perception::ConvertPointCloud2ImgDep, nodelet::Nodelet)

