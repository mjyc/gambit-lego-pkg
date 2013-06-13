#include <cmath>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp> // for FPS_CALC_CV
#include <pcl/point_types.h>
//#include <tf/transform_listener.h>

#include <gambit_perception/handtracking.h>
#include <gambit_perception/img_dep_sync_node.h>
#include <gambit_perception/tablestate.h>
#include <gambit_perception/tf_utils.h>
#include <gambit_perception/marker_utils.h>


// TODO:
// 1. make sure what I want to exactly publish and where.
// 2. clean up headers little bit.


#define FPS_CALC_CV(_WHAT_) \
    do \
{ \
    static int64 count = 0;\
    static double last = cv::getTickCount() / cv::getTickFretranuency();\
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

    ros::ServiceServer clear_tobj_srv_;
    tf::TransformListener listener_;

    ros::Publisher pub_text_;
    ros::Publisher pub_points_;

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

        // Read val params from ros param server
        TableParams params;
        params.table_noise_threshold = table_noise_threshold_;
        params.min_objsize_frac = min_objsize_frac_;
        params.obj_removal_thresh = obj_removal_thresh_;
        tableState_ = boost::make_shared<TableState>(tableModel, tableModel, mask, params, &model_, max_num_obj_);
        updateOn_ = false;

//        cloudPtr_.reset(new PointCloud);
//        cloud2Ptr_.reset(new PointCloud);

//        pub_pointcloud_objects_ = nh_.advertise<PointCloud>("objects", 1);
//        pub_pointcloud_objects2_ = nh_.advertise<PointCloud>("objects2", 1);
//        pub_pointcloud_bbox_ = nh_.advertise<PointCloud>("bbox", 1);
        pub_text_ = nh_.advertise<visualization_msgs::Marker>("texts",1);
        pub_points_ = nh_.advertise<visualization_msgs::Marker>("points",1);

        cv::namedWindow("fgMask");
    }

    virtual ~EventMainNodeTest() {}

    virtual void imageCb(const sensor_msgs::ImageConstPtr& depth_msg,
                         const sensor_msgs::ImageConstPtr& rgb_msg_in,
                         const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ImgDepSyncNode::imageCb(depth_msg, rgb_msg_in, info_msg);

        cv::Mat img = bgrFrame_;
        cv::Mat dep = depthFrame_;

        
        if (updateOn_) {
            //tableState_->update(img,dep);
            BOOST_FOREACH(TableEvent event, tableState_->update(img,dep)) {
                pub_text_.publish(createTextMarker(depth_msg->header.frame_id, event));
                pub_points_.publish(createPointsMarker(depth_msg->header.frame_id, event));
            }
        }



        // Controller

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
            BOOST_FOREACH(TableEvent event, tableState_->update(img,dep)) {
                pub_points_.publish(createPointsMarker(depth_msg->header.frame_id, event));
            }
        }

    }

};
const std::string EventMainNodeTest::default_params_path_ = "/home/mjyc/Projects/HIL/codes/ros/stacks/ros-pkg-nsl/trunk/gambit_perception/params/";


int main(int argc, char **argv)
{
    ros::init(argc, argv, "learn_basetable");
    EventMainNodeTest evMainNode;
    ros::spin();
}


