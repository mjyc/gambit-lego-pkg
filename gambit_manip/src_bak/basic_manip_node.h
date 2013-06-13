#ifndef _basic_manip_node_h_
#define _basic_manip_node_h_

#include <string>
#include <sound_play/sound_play.h>
#include "gambit_manip/arm_interface.h"

#include "gambit_manip/MoveObject.h"

//////////////////////////////////////////////////

class BasicManip
{
protected:
    ros::NodeHandle nh_global_;
    ros::NodeHandle nh_local_;

    Arm_IF aif_obj_;
    armlib::Arm *robot_;

    tf::TransformListener *listener_;

    //---- service servers ----//
    ros::ServiceServer move_object_;

    //---- various heights based on object heights ----//
    // height to move around table above objects
    double hover_height_;
    // when holding a object, raise how high before moving it
    double move_height_;
    // don't let gripper-tip go below board-plane
    double min_putdown_height_;

    std::string arm_ref_;
    std::string table_plane_ref_;

public:
    //---- static const variables ----//
    // palm to tip
    static const double gripper_length_ = 0.07;
    //added to object height, so "how high to drop it from"
    static const double object_placement_height_ = 0.008;
    // how far above the top of the object the palm should be when grabbing
    static const double pickup_offset_ = 0.035;

    // gripper length - pickup offset + epsilon
    static const double minimum_object_height_ = 0.0355;
    // what works for images
    static const double minimum_hover_height_ = 0.19;
    // what works for images
    static const double minimum_move_height_ = 0.25;

    // see end of this file
    static const std::string default_arm_ref_;
    static const std::string default_table_plane_ref_;

    BasicManip (ros::NodeHandle nh);
    ~BasicManip();

    //---- smaller components ----//
    int go_to_table_plane_coords_feasible_p(tf::Vector3 coords, \
											armlib::js_vect& pos);
    int go_to_table_plane_coords_with_rollangle_feasible_p( \
	  double, tf::Vector3 coords, armlib::js_vect& pos);

    int go_to_table_plane_coords(tf::Vector3 coords);
    int go_to_table_plane_coords_with_rollangle(double, tf::Vector3);
	
	// go_to_table_plane_coords with a fixed coords[2] (z-axis).
    int go_above_table_plane_coords(tf::Vector3 coords);
    int go_above_table_plane_with_rollangle(double, tf::Vector3 coords);

    int grasp_arbitrarily_placed_object(tf::Vector3 coords);

    int put_down_arbitrarily_placed_object(tf::Vector3 coords);

    int pick_up_arbitrarily_placed_object(tf::Vector3 coords);

    int move_arbitrarily_placed_object(tf::Vector3 coords_from, tf::Vector3 coords_to);

    // ROS
    bool move_object_srv(gambit_manip::MoveObject::Request &req, \
                            gambit_manip::MoveObject::Response &resp);

    int run();

}; // end class

// way around to define const std::string
const std::string BasicManip::default_arm_ref_ = "arm0";
const std::string BasicManip::default_table_plane_ref_ = "arm0";//"table_plane";
//const std::string BasicManip::default_table_plane_ref_ = "table_plane";
#endif // _basic_manip_node_h_

