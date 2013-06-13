#include <cmath>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp> // for FPS_CALC_CV
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/tableevent_detection.h>
#include <gambit_perception/handtracking.h>
#include <gambit_perception/icra2013/tablestate.h>
#include <gambit_perception/tf_utils.h>

#include <visualization_msgs/MarkerArray.h>
#include <gambit_manip/MoveGripper.h>


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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


class EventMainNodeTest : public ImgDepSyncNode {
protected:
    static const std::string default_params_path_;

    std::string params_path_;
    int table_noise_threshold_;
    double min_objsize_frac_;
    int obj_removal_thresh_;
    int max_num_obj_;

    TableState::Ptr tableState_;
    bool updateOn_;

    PointCloud::Ptr cloudPtr_;
    PointCloud::Ptr cloud2Ptr_;

    ros::Publisher pub_pointcloud_objects_;
    ros::Publisher pub_pointcloud_objects2_;
    ros::Publisher pub_pointcloud_bbox_;
    ros::Publisher pub_markers_;
    ros::ServiceServer clear_tobj_srv_;
    tf::TransformListener listener_;

public:
    EventMainNodeTest() {
        nh_.param<std::string>("params_path", params_path_, default_params_path_);
        nh_.param<int>("table_noise_threshold", table_noise_threshold_, TABLE_NOISE_THRESHOLD);
        nh_.param<double>("min_objsize_frac", min_objsize_frac_, MIN_OBJSIZE_FRAC);
        nh_.param<int>("obj_removal_thresh", obj_removal_thresh_, OBJ_REMOVAL_THRESH);
        nh_.param<int>("max_num_obj", max_num_obj_, MAX_NUM_OBJ);

        std::cout << "[EventMainNodeTest]" << std::endl;
        std::cout << "    params_path = " << params_path_ << std::endl;
        std::cout << "    table_noise_threshold = " << table_noise_threshold_ << std::endl;
        std::cout << "    min_objsize_frac = " << min_objsize_frac_ << std::endl;
        std::cout << "    obj_removal_thresh = " << obj_removal_thresh_ << std::endl;

        // Load img params
        cv::Mat mask = cv::imread(params_path_ + "mask.png", -1);
        cv::Mat tableModel = cv::imread(params_path_ + "baseTable.png", -1);
        std::vector< std::vector<cv::Point> > areaContours;
        cv::Size imgSize;
        InteractiveArea::readAreaFile(params_path_ + "area.txt", areaContours, imgSize);

        // Read val params from ros param server
        TableParams params;
        params.table_noise_threshold = table_noise_threshold_;
        params.min_objsize_frac = min_objsize_frac_;
        params.obj_removal_thresh = obj_removal_thresh_;
        tableState_ = boost::make_shared<TableState>(tableModel, tableModel, mask, areaContours, params, max_num_obj_);
        updateOn_ = false;

        cloudPtr_.reset(new PointCloud);
        cloud2Ptr_.reset(new PointCloud);

        pub_pointcloud_objects_ = nh_.advertise<PointCloud>("objects", 1);
        pub_pointcloud_objects2_ = nh_.advertise<PointCloud>("objects2", 1);
        pub_pointcloud_bbox_ = nh_.advertise<PointCloud>("bbox", 1);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("texts",1);

        cv::namedWindow("fgMask");
    }

    virtual ~EventMainNodeTest() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                         const sensor_msgs::ImageConstPtr& rgb_msg_in,
                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        cv::Mat img = bgrFrame_;
        cv::Mat dep = depthFrame_;

        if (updateOn_)
            tableState_->update(img,dep);


        // Print out events & publish text
        visualization_msgs::MarkerArray markers;
        BOOST_FOREACH(TableEvent event, tableState_->getEvents()) {
            if (event.getType() == TableEvent::CREATE) {
                // std::cout << "[EventMainNodeTest] " << event.getObj()->getName() + " create event" << std::endl;
                markers.markers.push_back(createMarker(depth_msg->header.frame_id, event));

            } else if (event.getType() == TableEvent::REMOVE) {
                // std::cout << "[EventMainNodeTest] " << event.getObj()->getName() + " remove event" << std::endl;
                markers.markers.push_back(createMarker(depth_msg->header.frame_id, event));

            } else if (event.getType() == TableEvent::REAPPEAR) {
                // std::cout << "[EventMainNodeTest] " << event.getObj()->getName() + " reappear event" << std::endl;
                markers.markers.push_back(createMarker(depth_msg->header.frame_id, event));

            }
        }
        if (markers.markers.size() != 0)
            pub_markers_.publish(markers);


        // Publish pointclouds
        BOOST_FOREACH(TableObject::Ptr obj, tableState_->getTableObjectVector()) {
            if (!obj->getOffTable()) {
                // objects
                cloudPtr_->header = depth_msg->header;
                cloudPtr_->header.stamp = ros::Time(0);
                obj->toCloud(model_.fx(),model_.fy(),model_.cx(),model_.cy(),cloudPtr_);
                pub_pointcloud_objects_.publish (cloudPtr_);

                // objects2
                cloud2Ptr_->header = depth_msg->header;
                cloud2Ptr_->header.stamp = ros::Time(0);
                obj->toLocStateCloud(model_.fx(),model_.fy(),model_.cx(),model_.cy(),cloud2Ptr_);
                pub_pointcloud_objects2_.publish (cloud2Ptr_);

                // bbox
                publishBBox(obj,depth_msg->header.frame_id,cloudPtr_);
            }
        }

        //FPS_CALC_CV("event_main_test");
        int key = cv::waitKey(1) % 256;
        if (key == 27) // ESC
            ros::shutdown();
        else if ((unsigned char) key == 'c') {
            tableState_->clear();
            std::cout << "[EventMainNodeTest] TableObject and TableEvent cleared, set NUM_OBJ to 0" << std::endl;
        } else if ((unsigned char) key == ' ') {
            if (updateOn_) {
                updateOn_ = false;
                std::cout << "[EventMainNodeTest] Update OFF" << std::endl;
                BOOST_FOREACH(TableObject::Ptr objPtr, tableState_->getTableObjectVector()) {
                    std::cout << objPtr->getName() << " " << objPtr->getLocState() << std::endl;
                }
            } else {
                updateOn_ = true;
                std::cout << "[EventMainNodeTest] Update ON" << std::endl;
            }
        } else if ((unsigned char) key == 'u') {
            std::cout << "[EventMainNodeTest] Update" << std::endl;
            tableState_->update(img,dep);
            BOOST_FOREACH(TableObject::Ptr objPtr, tableState_->getTableObjectVector()) {
                std::cout << objPtr->getName() << " " << objPtr->getLocState() << std::endl;

                //---- Compute grasping point math BEGIN ----//
                // Bounding box computation
                PointCloud::Ptr bPoints(new PointCloud);
                bPoints->header.frame_id = depth_msg->header.frame_id;
                bPoints->header.stamp = ros::Time(0);
                objPtr->boundingPoints(model_.fx(),model_.fy(),model_.cx(),model_.cy(), bPoints);

                // Centroid computation
                PointCloud::Ptr objCloud(new PointCloud);
                objPtr->toCloud(model_.fx(),model_.fy(),model_.cx(),model_.cy(),objCloud);
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*objCloud,centroid);
                centroid[3] = 1;

                // Get transform
                tf::StampedTransform transform;
                try{
                    listener_.lookupTransform("/arm0",depth_msg->header.frame_id,ros::Time(0),transform);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
                Eigen::Matrix4f AffineTransform;
                tfToEigen(transform,AffineTransform);

                // Compute grasp angle
                centroid = AffineTransform * centroid;
                //std::cout << "centroid after transform =\n" << centroid << std::endl;
                transformPointCloud(*bPoints, *bPoints, transform);
                double x = bPoints->points[1].x - bPoints->points[0].x;
                double y = bPoints->points[1].y - bPoints->points[0].y;
                double h = sqrt(x*x + y*y);
                double angle = sin(1.0*y/h);
                //std::cout << "angle = " << angle << std::endl;

                std::cout << angle + 1.57 << " " << centroid[0] << " " << centroid[1] << " " << centroid[2] + 0.07 - 0.0075 << std::endl;
                //---- Compute grasping point math END ----//
            }
        }
    }

    pcl::PointXYZRGB publishBBox(TableObject::Ptr obj, std::string frameID, PointCloud::Ptr& objCloud) {
        // Bounding box computation
        PointCloud::Ptr bPoints(new PointCloud);
        bPoints->header.frame_id = frameID;
        bPoints->header.stamp = ros::Time(0);
        obj->boundingPoints(model_.fx(),model_.fy(),model_.cx(),model_.cy(), bPoints);

        // Centroid computation
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*objCloud,centroid);

        pcl::PointXYZRGB centroidPt;
        centroidPt.x = centroid[0];
        centroidPt.y = centroid[1];
        centroidPt.z = centroid[2];
        centroidPt.r = 255;
        centroidPt.g = 0;
        centroidPt.b = 0;
        bPoints->push_back(centroidPt);
        pub_pointcloud_bbox_.publish(bPoints);

        return centroidPt;
    }

    visualization_msgs::Marker createMarker(std::string frameID, TableEvent event) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = frameID;
        marker.header.stamp = ros::Time(0);
        marker.ns = "objects";
        marker.id = boost::lexical_cast<int>(event.getObj()->getName()[3]);
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        if (event.getType() == TableEvent::CREATE || event.getType() == TableEvent::REAPPEAR)
            marker.action = visualization_msgs::Marker::ADD;
        else if (event.getType() == TableEvent::REMOVE)
            marker.action = visualization_msgs::Marker::DELETE;

        Eigen::Vector4f centroid;
        PointCloud::Ptr cloudPtr(new PointCloud);
        event.getObj()->toCloud(model_.fx(),model_.fy(),model_.cx(),model_.cy(),cloudPtr);
        pcl::compute3DCentroid(*cloudPtr,centroid);

        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2]-0.075;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.033;
        marker.color.a = 1.0;
        std::string text = event.getObj()->getName();
        if (event.getObj()->getLocState() == TableObject::LEFT) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            text += " left";
        } else if (event.getObj()->getLocState() == TableObject::RIGHT) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            text += " right";
        } else if (event.getObj()->getLocState() == TableObject::UNKNOWN)
            text += " unknown";
        else
            text += " offTable";
        marker.text = text;
        marker.lifetime = ros::Duration();

        return marker;
    }
};
const std::string EventMainNodeTest::default_params_path_ = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "learn_basetable");
    EventMainNodeTest evMainNode;
    ros::spin();
}


