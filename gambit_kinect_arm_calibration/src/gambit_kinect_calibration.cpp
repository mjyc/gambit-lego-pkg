#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <boost/thread/mutex.hpp>

// Global Variables
tf::Transform table_plane_transform_;


// Projecting a camera axis down to a found table plane
/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction){

  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);

  //make sure z points "up"
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
	z = -1.0 * z;
	ROS_INFO("flipped z");
  }
    
  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  ROS_DEBUG("in getPlaneTransform, x: %0.3f, %0.3f, %0.3f", x[0], x[1], x[2]);
  ROS_DEBUG("in getPlaneTransform, y: %0.3f, %0.3f, %0.3f", y[0], y[1], y[2]);
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  return tf::Transform(orientation, position);
}

// Cloud callback handler - publishes current camera to table_plane transformation
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients); 

  // IMPORTANT: Saves current transformation info to the global varibale.
  table_plane_transform_ = getPlaneTransform(coefficients, -1);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(table_plane_transform_, ros::Time::now(), "camera_rgb_optical_frame", "table_plane"));
}

// poorman's Spin()
boost::mutex mtx_;
bool move2next_;
struct sigaction sigIntHandler_;

void ctrlC_handler(int s) {
  boost::mutex::scoped_lock lock (mtx_);
  move2next_ = true;
}

void prepCtrlCTrap() {
  move2next_ = false;
  
  sigIntHandler_.sa_handler = ctrlC_handler;
  sigemptyset(&sigIntHandler_.sa_mask);
  sigIntHandler_.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler_, NULL);
}

bool ok() {
  bool trapped = false;
  boost::mutex::scoped_lock lock (mtx_);
  if (move2next_)
	trapped = true;
  return !trapped;
}


// Main chunk of code.
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "kinect_calibrator");
  ros::NodeHandle nh;

  // Broadcast fps
  ros::Rate loop_rate(100);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);


  // Calibration part
  std::cout << "Hit Ctrl-C when the rviz axis and the landmark axis are matched..." << std::endl;
  
  prepCtrlCTrap();
  while(ok()) {
  	ros::spinOnce();
  };
  

  // Broadcasting part
  std::cout << "Publishing static transformations from now." << std::endl;

  // IMPORTANT: from table_plane to arm0 - from hand measurements.
  double x_offset =  0.2 + 0.062;//0.06;   // 4.2cm
  double y_offset = -0.1 - 0.043 + 0.025;//0.046;  // 4.3cm
  double z_offset =  0.0;
  double rx_offset = 1.57079633 + 0.122173048; //7degree
  double ry_offset = 0.0;
  double rz_offset = 0.0;


  // Start looping and publishing
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  prepCtrlCTrap();
  while(ok()) {
    br.sendTransform( tf::StampedTransform(table_plane_transform_, ros::Time::now(), "camera_rgb_optical_frame", "table_plane") );

    transform.setOrigin( tf::Vector3(x_offset,y_offset,z_offset) );
    transform.setRotation( tf::Quaternion(rx_offset,ry_offset,rz_offset) );
    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "table_plane", "arm0") );
  };

  std::cout << "program is terminating..." << std::endl;
}
