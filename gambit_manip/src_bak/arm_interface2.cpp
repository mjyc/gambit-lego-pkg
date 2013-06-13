
//ros
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>

//transforms and math
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//robot
#include <armlib/trajectory.h>
#include <armlib/geom.h>
#include <armlib/gambit.h>

//local
//#include <gambit_manip/arm_interface2.h>
#include "../src_bak/arm_interface2.h"

#define debug_calls(x)
#define debug_deep(x)

#include <assert.h>
#include <iostream>

Arm_IF::Arm_IF (ros::NodeHandle nh)
    : nh_global_(nh),
      frameId_("arm0")
{
    // slow range: 0.5 ~ 1.0
    // fast range: 1.5 ~ 2.0
    startspeed_ = 0.5;   // make it slow!
    speed_ = 0.5;        // unclear max
    windingspeed_ = 1.5; // unclear max

    // simulation
//    startspeed_ = 2.0;   // make it slow!
//    speed_ = 2.0;        // unclear max
//    windingspeed_ = 2.0; // unclear max

    markerPublisher_ = nh_global_.advertise<visualization_msgs::Marker> ("arm_interface_marker", 1);
    robot_ = new armlib::Gambit();

    gripper_closed_ = false;
}

Arm_IF::~Arm_IF()
{
}


/**
 * Point at the sky
 * NO SAFTY CHECK!
 */
void Arm_IF::go_to_zero() {
    armlib::js_vect wait_pos;
    //Arm_IF aif_obj;
    wait_pos = make_pos_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if (gripper_closed_)
        wait_pos[6] = closed_gripper_angle_;
    else
        wait_pos[6] = open_gripper_angle_;
    go_to_pose(wait_pos);
    return;
}

/**
 * Go to customized starting position
 */
void Arm_IF::go_to_start() {
    armlib::js_vect start_pos;
    start_pos = back_pos;
    if (gripper_closed_)
        start_pos[6] = closed_gripper_angle_;
    else
        start_pos[6] = open_gripper_angle_;
    go_to_pose(start_pos, startspeed_);
}

/**
 * Go to customized position
 */
void Arm_IF::go_to_front() {
    armlib::js_vect start_pos;
    start_pos = front_pos;
    if (gripper_closed_)
        start_pos[6] = closed_gripper_angle_;
    else
        start_pos[6] = open_gripper_angle_;
    go_to_pose(start_pos, startspeed_);
    return;
}

/**
 * Go to customized position
 */
void Arm_IF::go_to_manip() {
    armlib::js_vect manipready_pos;
    manipready_pos = manip_pos;
    if (gripper_closed_)
        manipready_pos[6] = closed_gripper_angle_;
    else
        manipready_pos[6] = open_gripper_angle_;
    go_to_pose(manipready_pos, speed_);
}


/**
 * Unwind to right next to the kinect pole.
 */
bool Arm_IF::go_from_back_to_front() {
    // starting position check attempt
    if (!is_joint_at(back_pos, one_degree_tolerances)) {
        ROS_ERROR("joints are not in ""back"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper
    robot_->go_to(front_pos, windingspeed_); //fast speed
    robot_->wait_until_stopped();

    return true;
}

/**
  * move to a point above the play area from which to play
  */
bool Arm_IF::go_from_front_to_manip() {
    // starting position check attempt
    if (!is_joint_at(front_pos, one_degree_tolerances)) {
        ROS_ERROR("joints are not in ""front"" position.");
        return false;
    }

    // go_to_known_start (still "move" position)
    gripper_closed_ = false; //since manip_pos opens the gripper
    armlib::dir_vect dir;
    dir.push_back(armlib::R_NEGATIVE);
    dir.push_back(armlib::R_POSITIVE);
    dir.push_back(armlib::R_NEGATIVE);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);
    robot_->go_to(manip_pos, dir, speed_); // slow
    robot_->wait_until_stopped();

    return true;
}

/**
  * move to the near kinect pole position
  */
bool Arm_IF::go_from_manip_to_front() {
    // starting position check attempt
    if (!is_joint_at(manip_pos, one_degree_tolerances)) {
        ROS_ERROR("joints are not in ""manip"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper
    armlib::dir_vect dir;
    dir.push_back(armlib::R_POSITIVE);
    dir.push_back(armlib::R_NEGATIVE);
    dir.push_back(armlib::R_POSITIVE);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);
    dir.push_back(armlib::R_SHORTEST);

    robot_->go_to(front_pos, dir, speed_); // slow
    robot_->wait_until_stopped();

    return true;
}

/**
  * wind back
  */
bool Arm_IF::go_from_front_to_back() {
    // starting position check attempt
    if (!is_joint_at(front_pos, one_degree_tolerances)) {
        ROS_ERROR("joints are not in ""front"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper
    robot_->go_to(back_pos, windingspeed_); // fast speed
    robot_->wait_until_stopped();

    return true;
}

bool Arm_IF::unwind() {
    return go_from_back_to_front() && go_from_front_to_manip();
}

bool Arm_IF::wind() {
    return go_from_manip_to_front() && go_from_front_to_back();
}


//---- Utility functions ----//
/**
 * Given seven joint positions, make a usable pose vector from them. Useful for
 * hardcoded positions like zero, move_pos or watch_pos.
 */
armlib::js_vect Arm_IF::make_pos_vector (float i, float j, float k, float l, float m, float n, float o) {
    ROS_DEBUG("Call: make_pos_vector");
    armlib::js_vect pos;
    pos.push_back(i); pos.push_back(j); pos.push_back(k); pos.push_back(l);
    pos.push_back(m); pos.push_back(n); pos.push_back(o);
    return pos;
}

/** Draw a pink cube at the given pose x/y board coordinates.  Useful for testing
 * whether a pose is where you think it is on a table (e.g., whether you're
 * sending the gripper where you think you are).  Relies on the member function
 * markerId for global counting-up of cubes.
 *
 * Note that the "height" variable controls the z axis. See the variable posZ to
 * use the actual Z value of the pose instead.
 */
void Arm_IF::mark_coord(tf::Stamped<tf::Pose> pos) {
    ROS_DEBUG("Call: mark_coord Pose");
    float height = 0.14; // board-surface level for markers

    ros::Publisher publisher = markerPublisher_;
    std::string frameId = frameId_;	// this is what IK reasons in

    //take apart the pose
    tf::Vector3 transMatrix(pos.getOrigin()[0], pos.getOrigin()[1], pos.getOrigin()[2]);
    tf::Quaternion q(pos.getRotation()[0], pos.getRotation()[1], pos.getRotation()[2], pos.getRotation()[3]);

    float posX = transMatrix[0];
    float posY = transMatrix[1];
    float posZ = transMatrix[2]; // uncomment this if you don't want board-surface Z
    //float posZ = height;
    float orientX = q.getX();
    float orientY = q.getY();
    float orientZ = q.getZ();
    float orientW = q.getW();

    visualization_msgs::Marker coord_dot;

    coord_dot.header.frame_id = frameId;
    coord_dot.header.stamp = ros::Time::now();
    coord_dot.ns = "coord_dots"; // unique name for this marker-space
    coord_dot.id = markerId_;

    coord_dot.type = visualization_msgs::Marker::CUBE;
    coord_dot.action = visualization_msgs::Marker::ADD;

    // size of marker
    coord_dot.scale.x = 0.02;
    coord_dot.scale.y = 0.02;
    coord_dot.scale.z = 0.02;

    // color of marker
    coord_dot.color.a = 1.0; //opaque
    coord_dot.color.r = 1.0;
    coord_dot.color.g = 0.0;
    coord_dot.color.b = 0.5;

    //whereabouts of marker
    coord_dot.pose.position.x = posX;
    coord_dot.pose.position.y = posY;
    coord_dot.pose.position.z = posZ;
    coord_dot.pose.orientation.x = orientX;
    coord_dot.pose.orientation.y = orientY;
    coord_dot.pose.orientation.z = orientZ;
    coord_dot.pose.orientation.w = orientW;

    markerId_++;
    publisher.publish(coord_dot);
}

/**
 * Given a pose, get IK solutions for that pose and populate the vsolutions vector,
 * returning false if no IK solutions were found and true otherwise.
 *
 * This function assumes a gambit/WAM arrangement with a single free joint hardcoded to
 * have a value of 0 (center-of-range for most joints).  This returns all solutions, not
 * just the legal ones.
 */
bool Arm_IF::arm_ik_for_pose(tf::Stamped<tf::Pose> ps_to, std::vector<armlib::js_vect>& vsolutions) {
    debug_calls(printf("Call: arm_ik_for_pose"););
    double eerotation[9],eetranslation[3];
    tf::Matrix3x3 rotMatrix = ps_to.getBasis();
    tf::Vector3 transMatrix = ps_to.getOrigin();

    // mark it
    mark_coord(ps_to);

    // populate 3x3 rotation matrix
    unsigned int n = 0;
    for(unsigned int idx = 0; idx < 3; idx++)
        for(unsigned int jdx = 0; jdx < 3; jdx++) {
            eerotation[n] = rotMatrix[idx][jdx];
            n++; }

    // populate 1x3 translation matrix
    n = 0;

    for(unsigned int idx = 0; idx < 3; idx++) {
        eetranslation[n] = transMatrix[idx];
        n++; }

    // run IK (populate vsolutions)
    bool bSuccess = robot_->inverse_kinematics(eetranslation, eerotation, vsolutions);
    return bSuccess;
}

/**
 * Given a nonzero list of floats, return the one that is the arm pose
 * closest to the current position.
 */
bool Arm_IF::arm_get_nearest_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos) {
    // find one of those solutions that is legal
    bool legal_solution_found = false;
    assert(vsolutions.size() > 0);      // this renders the return value a bit silly

    // figure out where we are now
    armlib::js_vect current_pos;
    robot_->get_actual_joint_pos(current_pos);

    pos = robot_->closest_point(vsolutions, current_pos);

    return legal_solution_found;
}

///**
// * Given a nonzero list of floats, return the one that is the best
// * pose for the arm.
// */
//bool Arm_IF::arm_get_best_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos) {
//    debug_calls(printf("Call: arm_get_best_legal_solution"););
//    bool success = arm_get_nearest_legal_solution(vsolutions, pos);
//    return success;
//}

//there could be some problems with the continuous joints with this type of checking
bool Arm_IF::is_joint_at(armlib::js_vect desired_pos, armlib::js_vect tolerances) {
    armlib::js_vect pos;
    robot_->get_actual_joint_pos(pos);

    if (!(pos.size() == desired_pos.size() && desired_pos.size() == tolerances.size())) {
        ROS_ERROR("error related with input pos dimension");
        return false;
    }

    bool check = true;
    for (int i = 0; i < pos.size(); ++i) {
        float difference = desired_pos[i] - pos[i];
        while (difference < -1*M_PI) difference += 2*M_PI;
        while (difference > M_PI) difference -= 2*M_PI;
        if (abs(difference) > tolerances[i])
            check = false;
    }
    return check;
}

int Arm_IF::go_to_pose(armlib::js_vect pos, double vel) {
    if (robot_->check_arm_joint_limits(pos)) {
        robot_->go_to_sync(pos, vel);
        if (is_gripper_fully_closed())
            gripper_closed_ = true;
        else
            gripper_closed_ = false;
        return 0;
    } else {
        ROS_ERROR("go_to_pose called with illegal coordinates; failing.");
        return 1;
    }
}

/**
 * Go to the pose specified in the input. This is the right place to mess with
 * things like sanity checks, timeouts, and velocity, and should always be gone
 * through.
 */
int Arm_IF::go_to_pose(armlib::js_vect pos) {
    this->go_to_pose(pos, speed_);
//    if (robot_->check_arm_joint_limits(pos)) {
//        robot_->go_to_sync(pos, speed_);
//        if (is_gripper_fully_closed())
//            gripper_closed_ = true;
//        else
//            gripper_closed_ = false;
//        return 0;
//    } else {
//        ROS_ERROR("go_to_pose called with illegal coordinates; failing.");
//        return 1;
//    }
}


/**
 * Given some (probably proposed) pose, provide a boolean as to whether
 * the torso (joint 0) is angled towards the board or away from it.
 * Toward is: -90 < angle < 90.  (90 degrees in radians is 1.57079633.)
 */
bool Arm_IF::torso_forward_p(armlib::js_vect& pos) {
    //	double cutoff = 1.57079633;	// 90 degrees
    double cutoff = 1.83259571;	// 105 degrees
    //	double cutoff = 2.0943951;	// 120 degrees
    //	double cutoff = 2.35619449;	// 135 degrees
    assert(pos.size() > 0);
    float torso_pos = pos[0];
    return (torso_pos <= cutoff  &&  torso_pos >= (cutoff*-1));
}


/**
 * Open the gripper close to maximum.
 */
void Arm_IF::open_gripper() {
    armlib::js_vect pos;
    // set the gripper value, leave everything else unchanged
    robot_->get_actual_joint_pos(pos);
    pos[6] = open_gripper_angle_; //max open is ~0.785 == 45 degrees
    gripper_closed_ = false;
    go_to_pose(pos);
    return;
}

/**
 * Close the gripper as much as it will close.
 */
void Arm_IF::close_gripper() {
    armlib::js_vect pos;
    // set the gripper value, leave everything else unchanged
    robot_->get_actual_joint_pos(pos);
    pos[6] = closed_gripper_angle_; //max closed is ~ -1.047 == -60 degrees
    go_to_pose(pos);
    gripper_closed_ = true;
    return;
}

/**
 *
 */
bool Arm_IF::is_gripper_fully_closed() {
    armlib::js_vect pos;
    robot_->get_actual_joint_pos(pos);
    bool check = (pos[6] < -0.92); // ~ -52 degrees
    return (check);
}

/**
 *
 */
bool Arm_IF::is_gripper_set_to_closed() {
    return (gripper_closed_);
}
