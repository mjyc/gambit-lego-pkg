#include <stdio.h>
#include <vector>
#include <cctype>
#include <cstdlib>

//ros
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>

//transforms and math
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//robot
#include <armlib/gambit.h>

//manip
#include "gambit_manip/basic_manip_node.h"
#include "gambit_manip/arm_interface.h"



BasicManip::BasicManip (ros::NodeHandle nh)
  : nh_global_(nh),
    nh_local_("~"),
    aif_obj_(nh)
{
  // elements of various external classes
  robot_ = aif_obj_.get_armlib_obj();
  listener_ = new tf::TransformListener;

  // services provided here
  move_object_ = nh_global_.advertiseService("move_object", &BasicManip::move_object_srv, this);

  // parameter server - various counters, default values, etc.
  nh_local_.param<double>("hover_height",hover_height_,minimum_hover_height_);
  nh_local_.param<double>("move_height",move_height_,minimum_move_height_);
  nh_local_.param<double>("move_height",min_putdown_height_,gripper_length_);
  nh_local_.param<std::string>("arm_ref",arm_ref_,default_arm_ref_);
  nh_local_.param<std::string>("table_plane_ref",table_plane_ref_,default_table_plane_ref_);
}

BasicManip::~BasicManip()
{
}

//===================================================================================================
// Check and see if a usable IK solution is returned. Wrapper for default hand pose.
int BasicManip::go_to_table_plane_coords_feasible_p(tf::Vector3 coords, armlib::js_vect& pos) {
  return(go_to_table_plane_coords_with_rollangle_feasible_p(M_PI, coords, pos));
}

/*
 *  Get an IK solution for the proposed points, in some coordinates, and returns 0 if
 *  the proposed pose is feasible and an error code otherwise.  Re-does a lot of work from
 *  go_to_coords_with_rollangle;
 */
int BasicManip::go_to_table_plane_coords_with_rollangle_feasible_p( \
  double rollangle, tf::Vector3 coords, armlib::js_vect& pos) {
  // return error codes:
  //
  // 0: OK
  // 1: no IK solution found
  // 6: move gripper into ground
  // 8: other unknown error

  //coords[1] = coords[1] + 0.045;
  //coords[0] = coords[0] + 0.057;


  if (coords[2] < gripper_length_) {
    ROS_ERROR("Illegal z-coord %f would put grippers through table_plane.", coords[2]);
    return 6;  // ERRORCODE 6 - move gripper into ground
  }

  // gambit_kinect_calibration publishes transform from table_plane to table_plane;
  // gambit_kinect_calibration publishes transform from table_plane to arm:
  try{
    // get transformation and desired pose in new coordinate frame
    ros::Time latest;
    listener_->getLatestCommonTime(table_plane_ref_, arm_ref_, latest, NULL);

    // The quaternion is translated into roll/pitch/yaw
    //Pi's to rotate around the Z axis so hand points down at table_plane
    tf::Quaternion q; q.setRPY(rollangle, 0.0, -M_PI);
    tf::Pose pose(q, coords);

    // transform things
    tf::Stamped<tf::Pose> ps(pose, latest, table_plane_ref_);
    tf::Stamped<tf::Pose> ps_to;
    listener_->transformPose(arm_ref_, ps, ps_to);

    std::vector<armlib::js_vect> vsolutions;
    bool ikSuccess = aif_obj_.arm_ik_for_pose(ps_to, vsolutions);

    ROS_INFO("input coord %f,%f,%f", coords[0], coords[1], coords[2]);

    // If no solutions at all were found, escape.
    if (!ikSuccess) {
      ROS_ERROR("No IK found for %f,%f,%f", coords[0], coords[1], coords[2]);
      sleep(1);
      return 1;  // ERRORCODE 1 - no IK solution found
    }

    //set pos to best solution
    aif_obj_.arm_get_best_legal_solution(vsolutions, pos);

    //check if torso's backwards. No longer belongs in feasibility check,
    //but might get reinstated.
    bool torso_forward = aif_obj_.torso_forward_p(pos); //pos getting assigned
    if (!torso_forward) {
      ROS_WARN("No non-backwards IK found for %f,%f,%f", coords[0], coords[1], coords[2]);
            
      sleep(1);
      return 2;  // ERRORCODE 2 - IK solutions require turning backwards
    }
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 8; // ERRORCODE 8 - other unknown error
  }
  return 0;  // ERRORCODE 0 - all's well
}


/**
 * Trampolines to go_to_table_plane_coords_with_rollangle, adding a default hand orientation.
 */
int BasicManip::go_to_table_plane_coords(tf::Vector3 coords) {
  ROS_DEBUG("Coords: %f %f %f", coords[0], coords[1], coords[2]);
  return(go_to_table_plane_coords_with_rollangle(M_PI, coords));
}

// [2012.05.13 MJYC] there are many overlaps between go_to... and go_to..._feasible_p
// may want to clean up in the future.
/**
 * Given an <x,y,z> vector of coordinates in the frame of the table_plane, transforms them into
 * <x,y,z> points in the frame of the actuator, and calls go_to_pose in arm_interface to make
 * it happen.  ABSOLUTELY NO coordinate-based movement of the arm should happen without ending
 * up in this function.  (This also makes it the right place to add absolute overall offsets.)
 *
 * This bottoms out in go_to_pose, which should ONLY be called from here (or from pose-based
 * motion, but that should all be in arm_interface.)
 */
int BasicManip::go_to_table_plane_coords_with_rollangle(double rollangle, tf::Vector3 coords) {

  armlib::js_vect pos;
  int feasible = go_to_table_plane_coords_with_rollangle_feasible_p(rollangle, coords, pos);
  if (feasible > 0)
    return feasible;

  try{
    //set gripper to be unchanged
    armlib::js_vect current_pos;
    robot_->get_actual_joint_pos(current_pos);
    pos.push_back(current_pos[6]);

    // [2012.05.13 MJYC] TODO: safety check before move!
    aif_obj_.go_to_pose(pos);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return 8;  // ERRORCODE 8 - other unknown error
  }
  return 0;  // ERRORCODE 0 - all's well
}


/**
 * @brief Given x-y coordinates, get a default z and perform the motion
 */
int BasicManip::go_above_table_plane_coords(tf::Vector3 coords) {
  coords[2] = hover_height_;
  ROS_INFO("Moving to %f,%f,%f", coords[0], coords[1], coords[2]);
  return(go_to_table_plane_coords(coords));
}

/**
 * @brief Given x-y coordinates, get a default z and perform the motion
 */
int BasicManip::go_above_table_plane_with_rollangle(double rollangle, tf::Vector3 coords) {
  coords[2] = hover_height_;
  ROS_INFO("Moving to %f,%f,%f", coords[0], coords[1], coords[2]);
  return(go_to_table_plane_coords_with_rollangle(rollangle, coords));
}

/**
 * Attempt to grasp a object from a point directly above it.
 */
int BasicManip::grasp_arbitrarily_placed_object(tf::Vector3 coords) {
  //drop to expected location
  int gotthere = go_to_table_plane_coords(coords);  // 0: good; 1: ik fail; 2: backwards
  if(gotthere != 0) {;
    return(gotthere);
  }

  //close
  aif_obj_.close_gripper();

  // where did the gripper actually stop? (are we gripping something?)
  if ( !aif_obj_.is_gripper_fully_closed() ) { //we did grab it
    //lift
    int fail = go_to_table_plane_coords(tf::Vector3 (coords[0], coords[1], move_height_));
    if(fail > 0) {
      return(3);  // ERRORCODE 3 - Can't lift the arm
    }
    return (0);
  }

  // we didn't grab it: bail
  return(4);  // ERRORCODE 4 - Didn't grab a piece
}

/**
 */
int BasicManip::put_down_arbitrarily_placed_object(tf::Vector3 coords) {
  // go to above intended spot
  int fail = go_to_table_plane_coords(tf::Vector3 (coords[0], coords[1], move_height_));
  if(fail != 0) {
    return(fail);
  }
  //drop to expected location
  fail = go_to_table_plane_coords(coords);
  if(fail != 0) {
    return(fail);
  }
  //open
  aif_obj_.open_gripper();
  //lift
  go_to_table_plane_coords(tf::Vector3 (coords[0], coords[1], move_height_));
  return (0);
}

/**
 * todo doc
 */
int BasicManip::pick_up_arbitrarily_placed_object(tf::Vector3 coords) {
  // position roughly
  int val = go_above_table_plane_coords(coords);
  if (val != 0) {
    return 6;  // ERRORCODE 6 - go to object position failed
  }
  // open gripper
  ROS_DEBUG("Opening gripper...");
  aif_obj_.open_gripper();
  // grasp_object drops and closes gripper
  return(grasp_arbitrarily_placed_object(coords));
}

/**
 * @brief Given two tf::Vector3 coordinates, tries to move a chess object from one to the other.
 */
int BasicManip::move_arbitrarily_placed_object(tf::Vector3 coords_from, tf::Vector3 coords_to) {

  aif_obj_.go_from_watch_to_move();

  ROS_INFO("from coord %f,%f,%f", coords_from[0], coords_from[1], coords_from[2]);
  ROS_INFO("to   coord %f,%f,%f", coords_to[0], coords_to[1], coords_to[2]);

  std::cout << "calling pick up" << std::endl;
  //pick it up
  int pickedup = pick_up_arbitrarily_placed_object(coords_from);
//    if(pickedup != 0) {
//
//        return (pickedup);
//    }
  std::cout << "done calling pick up" << std::endl;
  // put it down - gently, hopefully
  int putdown = put_down_arbitrarily_placed_object(coords_to);
//    if(putdown != 0) {
//
//        return (putdown);
//    }

  aif_obj_.go_from_move_to_watch();

  return (0);
}


bool BasicManip::move_object_srv(gambit_manip::MoveObject::Request &req, gambit_manip::MoveObject::Response &resp) {
  tf::Vector3 coords_from, coords_to;

  coords_from[0] = req.from_x;
  coords_from[1] = req.from_y;
  coords_from[2] = req.from_z;
  coords_to[0] = req.to_x;
  coords_to[1] = req.to_y;
  coords_to[2] = req.to_z;

  resp.retval = move_arbitrarily_placed_object(coords_from, coords_to);
  return (true);
}


int BasicManip::run()
{
  ros::spin();
  return 0;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "basic_manip_node");
  ros::NodeHandle nh;

  sleep(2);
  BasicManip basic_manip_node(nh);
  std::cout << "Ready to handle services..." << std::endl;
  sleep(2);

  return basic_manip_node.run();
}

