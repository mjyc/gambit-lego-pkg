// NOTE: cv_utils.h and other files debug main
#include <iostream>
#include <limits>

#include <boost/foreach.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <gambit_perception/cv_utils.h>
#include <gambit_perception/handtracking.h>
#include <gambit_perception/tf_utils.h>
#include <gambit_perception/icra2013/tablestate.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


void rangeTest(cv::Mat I) {
    int nRows = I.rows;
    int nCols = I.cols;

    int cnt = 0;
    for (int i = 0; i < nRows*nCols; ++i) {
        std::cout << i << std::endl;
        float value = I.at<float>(i); // Run time error here.
        if( isinf(value) && value < 0){
            std::cout << "Object too close to measure." << std::endl;
        } else if( isinf(value) && value > 0){
            std::cout << "No objects detected in range." << std::endl;
        } else if( isnan(value) ){
            //std::cout << "This is an erroneous, invalid, or missing measurement." << std::endl;
            //std::cout << value << std::endl;
            cnt++;
        }
    }
    std::cout << "cnt = " << cnt << std::endl;

    cv::Mat dep;
    toOpenCVDepthMap(I,dep);
    std::cout << "after conversion = " << nRows * nCols - cv::countNonZero( dep ) << std::endl;
}

void bigNumberTest() {
    int bigInt = 65535;
    unsigned int big = 65535;
    std::cout << "bigInt = " << bigInt << std::endl;
    std::cout << "big unassigned int = " << big << std::endl;
}

void patchValueTest(cv::Mat dep) {
    int big = 65535;

    std::cout << dep.size().area() - cv::countNonZero(dep) << std::endl;
    patchValue16UC1(dep,0,big);

    cv::Mat I = dep;
    int cntBig = 0;
    int cnt0 = 0;
    for( int i = 0; i < I.rows; ++i)
        for( int j = 0; j < I.cols; ++j ) {
            if (I.at<uint16_t>(i,j) == big)
                cntBig++;
            else if (I.at<uint16_t>(i,j) == 0)
                cnt0++;
        }

    std::cout << cntBig << std::endl;
    std::cout << cnt0 << std::endl;
}

void tfUtilsTest() {
    // create empty transform
    tf::Transform transform;
    tf::Matrix3x3 temp = transform.getBasis();
    printRotation( transform.getBasis() );
    printTranslation( transform.getOrigin() );
    std::cout << std::endl;

    // convert to affine transform
    Eigen::Matrix4f t;
    tfToEigen(transform,t);
    std::cout << "t\n" << t << std::endl << std::endl;

    // now convert back to bt
    eigenToTf(t,transform);
    printRotation( transform.getBasis() );
    printTranslation( transform.getOrigin() );
    std::cout << std::endl;

    // load up
    std::cout << "loadEigenMatrixFromFile(\"params/Tcw.txt\") = \n" \
              << loadEigenMatrixFromFile("params/Tcw.txt") << std::endl;
}

void transformTest(int argc, char **argv) {

    ros::init(argc, argv, "cv_utils_test");
    ros::NodeHandle nh;

    ros::Publisher pub_point_cloud = nh.advertise<PointCloud>("points", 1);
    ros::Publisher pub_centroid = nh.advertise<PointCloud>("centroid", 1);
    ros::Publisher pub_corners = nh.advertise<PointCloud>("corners", 1);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

    Eigen::Matrix4f transform = loadEigenMatrixFromFile("params/Tcw.txt");
    //    tf::TransformListener listener;

    //    tf::StampedTransform trans;
    //    try{
    //        listener.lookupTransform("/turtle2", "/turtle1",
    //                                 ros::Time(0), trans);
    //    }
    //    catch (tf::TransformException ex){
    //        ROS_ERROR("%s",ex.what());
    //    }


    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        cv::Mat croppedImg = cv::imread("croppedImg.png",-1);
        cv::Mat croppedDep = cv::imread("croppedDep.png",-1);

        PointCloud::Ptr cloudMsg(new PointCloud);
        cloudMsg->header.frame_id = "/camera_rgb_optical_frame";
        cloudMsg->height = croppedDep.rows;
        cloudMsg->width  = croppedDep.cols;
        cloudMsg->is_dense = false;
        cloudMsg->points.resize (cloudMsg->height * cloudMsg->width);
        toPointCloud(croppedImg,croppedDep, 525.0, 525.0, 319.5, 239.5,cloudMsg);

        // e.g. idle objects are in blue color
        BOOST_FOREACH(pcl::PointXYZRGB& pt, cloudMsg->points) {
            pt.r = 0; pt.g = 0; pt.b = 255;
        }

        //transformPointCloud(*cloudMsg,*cloudMsg,transform);
        pcl::transformPointCloud(*cloudMsg, *cloudMsg, transform);
        pub_point_cloud.publish(cloudMsg);


        // ==== Compute Centroid ==== //
        PointCloud::Ptr centroidCloud(new PointCloud);
        centroidCloud->header.frame_id = "/camera_rgb_optical_frame";
        pcl::PointXYZRGB mean;
        double x = 0;
        double y = 0;
        double z = 0;
        double cnt = 0.0;
        BOOST_FOREACH(const pcl::PointXYZRGB pt, cloudMsg->points) {
            if (std::isfinite(pt.z)) {
                //std::cout << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
                x += pt.x;
                y += pt.y;
                z += pt.z;
                cnt++;
            }
        }

        mean.x = x/cnt;
        mean.y = y/cnt;
        mean.z = z/cnt;
        mean.r = 255;
        mean.g = 0;
        mean.b = 0;

        centroidCloud->push_back(mean);
        std::cout << "centroidCloud->height = " << centroidCloud->height << std::endl;
        std::cout << "centroidCloud->width = " << centroidCloud->width << std::endl;
        std::cout << "centroidCloud->is_dense = " << centroidCloud->is_dense << std::endl << std::endl;
        pub_centroid.publish(centroidCloud);


        // ==== Publish visu marker ==== //;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_rgb_optical_frame";
        marker.header.stamp = ros::Time(0);
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = mean.x;
        marker.pose.position.y = mean.y;
        marker.pose.position.z = mean.z+0.1;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        //        marker.scale.x =. 1;
        //        marker.scale.y = 1;
        marker.scale.z = 0.075;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.text = "obj1";
        marker.lifetime = ros::Duration();

        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vis_pub.publish( marker );


        // ==== Compute Corners ==== //
        pcl::PointXYZRGB a = cloudMsg->points[0];
        pcl::PointXYZRGB b = cloudMsg->points[cloudMsg->width-1];
        pcl::PointXYZRGB c = cloudMsg->points[cloudMsg->width*(cloudMsg->height-1)];
        pcl::PointXYZRGB d = cloudMsg->points[cloudMsg->width*cloudMsg->height-1];

        PointCloud::Ptr corners(new PointCloud);
        corners->header.frame_id = "/camera_rgb_optical_frame";
        corners->push_back(a);
        corners->push_back(b);
        corners->push_back(c);
        corners->push_back(d);
        BOOST_FOREACH(pcl::PointXYZRGB& pt, corners->points) {
            pt.r = 0;
            pt.g = 255;
            pt.b = 0;
        }

        pub_corners.publish(corners);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
}

void eigenDotProductTest() {

    Eigen::Vector4f vec;
    vec << 0.174473, 0.131647, 0.807896, 1;
    Eigen::Matrix4f mat;
    mat << -0.250022, -0.905596,  0.342614,  0.152598,
            -0.94818,  0.157345,  -0.27604,  0.256125,
            0.196072, -0.393876, -0.898008,  0.781473,
            0,         0,         0,         1;

    std::cout << mat * vec << std::endl;
}

void transformTest2(int argc, char **argv) {
    ros::init(argc, argv, "cv_utils_test");
    ros::NodeHandle nh;

    ros::Publisher pub_point_cloud = nh.advertise<PointCloud>("points", 1);
    ros::Publisher pub_centroid = nh.advertise<PointCloud>("centroid", 1);
    ros::Publisher pub_corners = nh.advertise<PointCloud>("corners", 1);


    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok())
    {
        cv::Mat croppedImg = cv::imread("croppedImg.png",-1);
        cv::Mat croppedDep = cv::imread("croppedDep.png",-1);

        PointCloud::Ptr cloudMsg(new PointCloud);
        cloudMsg->header.frame_id = "/camera_rgb_optical_frame";
        cloudMsg->height = croppedDep.rows;
        cloudMsg->width  = croppedDep.cols;
        cloudMsg->is_dense = false;
        cloudMsg->points.resize (cloudMsg->height * cloudMsg->width);
        toPointCloud(croppedImg,croppedDep, 525.0, 525.0, 319.5, 239.5,cloudMsg);

        std::cout << "cloudMsg->points = " << cloudMsg->points.size() << std::endl;

        // e.g. idle objects are in blue color
        BOOST_FOREACH(pcl::PointXYZRGB& pt, cloudMsg->points) {
            pt.r = 0; pt.g = 0; pt.b = 255;
        }
        pub_point_cloud.publish(cloudMsg);


        // ==== Compute Corners ==== //
        PointCloud::Ptr cloudMsg2(new PointCloud);
        cloudMsg2->header.frame_id = "/camera_rgb_optical_frame";
        for (int v = 0; v < croppedDep.rows; ++v)
        {
            for (int u = 0; u < croppedDep.cols; ++u)
            {
                uint16_t depth = croppedDep.at<uint16_t>(v,u);
                cloudMsg2->points.push_back(toPointXYZRGB(u,v,depth,255,0,255, \
                                                          525.0, 525.0, 319.5, 239.5));
            }
        }
        std::cout << cloudMsg2->points.size() << std::endl;
        cloudMsg2->is_dense = false;
        //cloudMsg2->points.resize (cloudMsg2->height * cloudMsg2->width);

        pub_corners.publish(cloudMsg2);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
}

void readAreaFileTest() {
    std::vector< std::vector<cv::Point> > areaData;
    cv::Size imgSize;
    InteractiveArea::readAreaFile("params/area.txt",areaData,imgSize);

    std::cout << "imgSize:\n" << imgSize.height << ", " << imgSize.width << std::endl;

    for (int i = 0; i < areaData.size(); ++i) {
        std::cout << "area" << boost::lexical_cast<std::string>(i) << std::endl;
        for (int j = 0; j < areaData[i].size(); ++j) {
            std::cout << areaData[i][j].x << ", " << areaData[i][j].y << std::endl;
        }
        std::cout << std::endl;
    }

    std::vector<cv::Mat> areaMasks;
    InteractiveArea::getAreaMasksFromFile("params/area.txt",areaMasks);
}

void inheritanceTest() {
    cv::Mat mask;
    TableParams params;

//    TableStateICRA2013 test1(mask,mask,mask,params);
//    std::cout << test1.getEvents().size() << std::endl;

//    TableStateICRA2013::Ptr test2 = boost::make_shared<TableStateICRA2013>(mask,mask,mask,params);
//    std::cout << test2->getEvents().size() << std::endl;

//    // worked - both compiletime and runtime!
//    TableState::Ptr test3 = boost::make_shared<TableStateICRA2013>(mask,mask,mask,params);
//    std::cout << test3->getEvents().size() << std::endl;
}

int main(int argc, char **argv) {
    //    std::cout << "cv_utils_test started." << std::endl;

    //    cv::Mat mask = cv::imread("params/mask.png",-1);
    //    cv::Mat tablemodel = cv::imread("params/baseTable.png",-1);

    //    cv::Mat img = cv::imread("img.png",-1);
    //    cv::Mat dep = cv::imread("dep.png",-1);
    //    cv::Mat rawdepth = cv::imread("rawdepth.png",CV_32FC1);

    //    rangeTest(rawdepth);
    //    bigNumberTest();
    //    tfUtilsTest();
    //transformTest(argc,argv);
    //eigenDotProductTest();
    //transformTest2(argc,argv);
    readAreaFileTest();
    //inheritanceTest();

    return 1;
}

