#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            double rollangle = 3.14;
            tf::Vector3 coords;
            tf::Quaternion q; q.setRPY(rollangle, 0.0, -M_PI);
            tf::Pose pose(q, coords);
            ros::Time latest;
            listener.getLatestCommonTime("/arm0", "/arm0", latest, NULL);
            listener.getLatestCommonTime("/arm0", "/arm0", latest, NULL);
            tf::Stamped<tf::Pose> ps(pose, latest, "/arm0");
            tf::Stamped<tf::Pose> ps_to;
            listener.transformPose("/arm0", ps, ps_to);

            listener.lookupTransform("/arm0", "/arm0",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        cout << "transform.getOrigin()" << endl;
        cout << transform.getOrigin().x() << endl;
        cout << transform.getOrigin().y() << endl;
        cout << transform.getOrigin().z() << endl;
        cout << endl;
        rate.sleep();
    }
    return 0;
};

