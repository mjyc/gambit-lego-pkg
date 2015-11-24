#ifndef MARKERS_UTILS_H
#define MARKERS_UTILS_H

#include <boost/lexical_cast.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <gambit_perception/tableevent.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

visualization_msgs::Marker createTextMarker(std::string frameID, TableEvent event) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time(0);
    marker.ns = "object_name";
    marker.id = boost::lexical_cast<int>(event.getObj()->getName()[3]);

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    if (event.getType() == TableEvent::CREATE || event.getType() == TableEvent::REAPPEAR)
        marker.action = visualization_msgs::Marker::ADD;
    else if (event.getType() == TableEvent::REMOVE)
        marker.action = visualization_msgs::Marker::DELETE;

    Eigen::Vector4f centroid = event.getObj()->getCentroid2();

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2]-0.075;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.033;

    std::cout << event.getObj()->getName() << "'s color = ";
    std::cout << event.getObj()->getColor()[0] << event.getObj()->getColor()[1] << event.getObj()->getColor()[2] << "(rgb)" << std::endl;

    std::string text = event.getObj()->getName();
    marker.color.r = event.getObj()->r;
    marker.color.g = event.getObj()->g;
    marker.color.b = event.getObj()->b;
    marker.color.a = 1.0;

    marker.text = text;
    marker.lifetime = ros::Duration();

    return marker;
}


visualization_msgs::Marker createPointsMarker(std::string frameID, TableEvent event) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time(0);
    marker.ns = "single_color_points";

    if (event.getType() == TableEvent::CREATE || event.getType() == TableEvent::REAPPEAR)
        marker.action = visualization_msgs::Marker::ADD;
    else if (event.getType() == TableEvent::REMOVE)
        marker.action = visualization_msgs::Marker::DELETE;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.id = boost::lexical_cast<int>(event.getObj()->getName()[3]);

    marker.scale.x = 0.002;
    marker.scale.y = 0.002;
    marker.scale.z = 1.0;

    marker.color.r = event.getObj()->r;
    marker.color.g = event.getObj()->g;
    marker.color.b = event.getObj()->b;
    marker.color.a = 1.0;

    PointCloud::Ptr cloudPtr(new PointCloud);
    event.getObj()->toCloud(cloudPtr);

    for(size_t i=0; i<cloudPtr->points.size(); i++) {
        if (cloudPtr->points[i].z == cloudPtr->points[i].z) {
            geometry_msgs::Point p;
            p.x = cloudPtr->points[i].x;
            p.y = cloudPtr->points[i].y;
            p.z = cloudPtr->points[i].z;
            marker.points.push_back(p);
        }
    }

    return marker;
}

//visualization_msgs::Marker toBoundingMarker() {

//}

#endif // MARKERS_UTILS_H
