#include <assert.h>
#include <iostream>
#include <boost/make_shared.hpp>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <armlib/trajectory.h>
#include <armlib/geom.h>
#include <armlib/gambit.h>
#include "gambit_manip/arm_interface.h"

/**
 * Given seven joint positions, make a usable pose vector from them. Useful for
 * hardcoded positions like zero, move_pos or watch_pos.
 */
armlib::js_vect ArmIF::make_pos_vector (float i, float j, float k, float l, float m, float n, float o) {
    ROS_DEBUG("Call: make_pos_vector");
    armlib::js_vect pos;
    pos.push_back(i); pos.push_back(j); pos.push_back(k); pos.push_back(l);
    pos.push_back(m); pos.push_back(n); pos.push_back(o);
    return pos;
}

double ArmIF::speed_range_check(double speed) {
    // slow range: 0.2 ~ 0.7
    // fast range: 1.0 ~ 1.5
    if (speed < min_speed_) {
        ROS_WARN_STREAM("[ArmIF] Input speed " << speed);
        ROS_WARN_STREAM("[ArmIF] Speed can't be slower than " << min_speed_);
        speed = min_speed_;
    } else if (speed > max_speed_) {
        ROS_WARN_STREAM("[ArmIF] Input speed " << speed);
        ROS_WARN_STREAM("[ArmIF] Speed can't be faster than " << max_speed_);
        speed = max_speed_;
    }
    return speed;
}


ArmIF::ArmIF (ros::NodeHandle nh)
    : nh_global_(nh),
      robot_(boost::make_shared<armlib::Gambit>()),
      gripper_closed_(false)
{
    nh_global_.param<double>("speed", speed_, 0.5);              // default speed_ = 0.5
    nh_global_.param<std::string>("arm_ref", arm_ref_, "arm0");  // default arm_ref_ = "arm0"
    speed_ = ArmIF::speed_range_check(speed_);
}

ArmIF::~ArmIF() {}


/**
 * Given a pose, get IK solutions for that pose and populate the vsolutions vector,
 * returning false if no IK solutions were found and true otherwise.
 *
 * This function assumes a gambit/WAM arrangement with a single free joint hardcoded to
 * have a value of 0 (center-of-range for most joints).  This returns all solutions, not
 * just the legal ones.
 */
bool ArmIF::arm_ik_for_pose(tf::Stamped<tf::Pose> ps_to, std::vector<armlib::js_vect>& vsolutions) {
    ROS_DEBUG("[ArmIF] Call: arm_ik_for_pose");
    double eerotation[9],eetranslation[3];
    tf::Matrix3x3 rotMatrix = ps_to.getBasis();
    tf::Vector3 transMatrix = ps_to.getOrigin();

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
    //lock();
    bool bSuccess = robot_->inverse_kinematics(eetranslation, eerotation, vsolutions);
    return bSuccess;
}

/**
 * Given a nonzero list of floats, return the one that is the arm pose
 * closest to the current position.
 */
bool ArmIF::arm_get_nearest_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos) {
    // find one of those solutions that is legal
    bool legal_solution_found = false;
    assert(vsolutions.size() > 0);      // this renders the return value a bit silly

    // figure out where we are now
    armlib::js_vect current_pos;
    //lock();
    robot_->get_actual_joint_pos(current_pos);

    pos = robot_->closest_point(vsolutions, current_pos);

    return legal_solution_found;
}

bool ArmIF::is_joint_at(armlib::js_vect desired_pos, armlib::js_vect tolerances) {
    armlib::js_vect pos;
    robot_->get_actual_joint_pos(pos);

    if (!(pos.size() == desired_pos.size() && desired_pos.size() == tolerances.size())) {
        ROS_ERROR("error related with input pos dimension");
        return false;
    }

    bool check = true;
    for (size_t i = 0; i < pos.size(); ++i) {
        float difference = desired_pos[i] - pos[i];
        while (difference < -1*M_PI) difference += 2*M_PI;
        while (difference > M_PI) difference -= 2*M_PI;
        if (abs(difference) > tolerances[i])
            check = false;
    }
    return check;
}

// return error codes:
//
// 0: OK
// 1: illegal input pos
// 2: motion ended by the user
int ArmIF::go_to_pose(armlib::js_vect pos, double vel) {
    if (robot_->check_arm_joint_limits(pos)) {
        // motion begins
        set_user_stopped_flag(false);
        robot_->go_to_sync(pos, vel);

        // check for gripper state
        if (is_gripper_fully_closed())
            gripper_closed_ = true;
        else
            gripper_closed_ = false;

        // check stopped condition
        if (get_user_stopped_flag()) {
            return 2;
        }
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
int ArmIF::go_to_pose(armlib::js_vect pos) {
    return this->go_to_pose(pos, speed_);
}

/**
 * Given some (probably proposed) pose, provide a boolean as to whether
 * the torso (joint 0) is angled towards the board or away from it.
 * Toward is: -90 < angle < 90.  (90 degrees in radians is 1.57079633.)
 */
bool ArmIF::torso_forward_p(armlib::js_vect& pos) {
    //	double cutoff = 1.57079633;	// 90 degrees
    double cutoff = 1.83259571;	// 105 degrees
    //	double cutoff = 2.0943951;	// 120 degrees
    //	double cutoff = 2.35619449;	// 135 degrees
    assert(pos.size() > 0);
    float torso_pos = pos[0];
    return (torso_pos <= cutoff  &&  torso_pos >= (cutoff*-1));
}

void ArmIF::stop_motion() {
    robot_->stop_motion();
    set_user_stopped_flag(true);
}


/**
 * Open the gripper close to maximum.
 */
void ArmIF::open_gripper() {
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
void ArmIF::close_gripper() {
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
bool ArmIF::is_gripper_fully_closed() {
    armlib::js_vect pos;
    robot_->get_actual_joint_pos(pos);
    bool check = (pos[6] < -0.92); // ~ -52 degrees
    return (check);
}

/**
 *
 */
bool ArmIF::is_gripper_set_to_closed() {
    return (gripper_closed_);
}


/**
 * Point at the sky
 * NO SAFTY CHECK!
 */
void ArmIF::go_to_zero() {
    armlib::js_vect wait_pos;

    wait_pos = make_pos_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    if (gripper_closed_)
        wait_pos[6] = closed_gripper_angle_;
    else
        wait_pos[6] = open_gripper_angle_;
    go_to_pose(wait_pos);
    return;
}





ManipArmIF::ManipArmIF(ros::NodeHandle nh, SpeedParams spdParams)
    : ArmIF(nh),
      spdParams_(spdParams)
{
    // TODO - make them ros parameters
    gripper_length_ = 0.07;
    move_height_ = 0.2;
    object_height_ = 0.09;

    spdParams_.gotoXSpeed = ArmIF::speed_range_check(spdParams_.gotoXSpeed);
    spdParams_.windingSpeed = ArmIF::speed_range_check(spdParams_.windingSpeed);
    spdParams_.windingSpeed2 = ArmIF::speed_range_check(spdParams_.windingSpeed2);
    spdParams_.hoverSpeed = ArmIF::speed_range_check(spdParams_.hoverSpeed);
    spdParams_.manipSpeed = ArmIF::speed_range_check(spdParams_.manipSpeed);
}

ManipArmIF::~ManipArmIF()
{}


/**
 * Go to customized starting position
 */
void ManipArmIF::go_to_back() {
    armlib::js_vect back_pos;
    back_pos = back_pos_;
    if (gripper_closed_)
        back_pos[6] = closed_gripper_angle_;
    else
        back_pos[6] = open_gripper_angle_;
    go_to_pose(back_pos, spdParams_.gotoXSpeed);
}

void ManipArmIF::go_to_front() {
    armlib::js_vect front_pos;
    front_pos = front_pos_;
    if (gripper_closed_)
        front_pos[6] = closed_gripper_angle_;
    else
        front_pos[6] = open_gripper_angle_;
    go_to_pose(front_pos, spdParams_.gotoXSpeed);
    return;
}

void ManipArmIF::go_to_manip() {
    armlib::js_vect manip_pos;
    manip_pos = manip_pos_;
    if (gripper_closed_)
        manip_pos[6] = closed_gripper_angle_;
    else
        manip_pos[6] = open_gripper_angle_;
    go_to_pose(manip_pos, spdParams_.gotoXSpeed);
}

void ManipArmIF::go_to_offview() {
    armlib::js_vect offview_pos;
    offview_pos = offview_pos_;
    if (gripper_closed_)
        offview_pos[6] = closed_gripper_angle_;
    else
        offview_pos[6] = open_gripper_angle_;
    go_to_pose(offview_pos, spdParams_.gotoXSpeed);
}




/**
 * Unwind to right next to the kinect pole.
 */
bool ManipArmIF::go_from_back_to_front() {
    // starting position check attempt
    if (!is_joint_at(back_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""back"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper

    if (go_to_pose(front_pos_, spdParams_.windingSpeed) != 0)
        return 0;
    return 1;
}


/**
  * wind back
  */
bool ManipArmIF::go_from_front_to_back() {
    // starting position check attempt
    if (!is_joint_at(front_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""front"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper

    if (go_to_pose(back_pos_, spdParams_.windingSpeed) != 0)
        return 0;
    return 1;
}


/**
  * move to a point above the play area from which to play
  */
bool ManipArmIF::go_from_front_to_manip() {
    // starting position check attempt
    if (!is_joint_at(front_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""front"" position.");
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

    if (go_to_pose(manip_pos_, spdParams_.windingSpeed2) != 0)
        return 0;
    return 1;
}


/**
  * move to the near kinect pole position
  */
bool ManipArmIF::go_from_manip_to_front() {
    // starting position check attempt
    if (!is_joint_at(manip_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""manip"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper
//    armlib::dir_vect dir;
//    dir.push_back(armlib::R_POSITIVE);
//    dir.push_back(armlib::R_NEGATIVE);
//    dir.push_back(armlib::R_POSITIVE);
//    dir.push_back(armlib::R_SHORTEST);
//    dir.push_back(armlib::R_SHORTEST);
//    dir.push_back(armlib::R_SHORTEST);
//    dir.push_back(armlib::R_SHORTEST);
//    robot_->go_to(front_pos_, dir, spdParams_.windingSpeed2);
//    robot_->wait_until_stopped();

    if (go_to_pose(front_pos_, spdParams_.windingSpeed2) != 0)
        return 0;
    return 1;
}

bool ManipArmIF::go_from_manip_to_offview() {
    // starting position check attempt
    if (!is_joint_at(manip_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""manip"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper

    if (go_to_pose(offview_pos_, spdParams_.offviewSpeed) != 0)
        return 0;
    return 1;
}

bool ManipArmIF::go_from_offview_to_manip() {
    // starting position check attempt
    if (!is_joint_at(offview_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""offview"" position.");
        return false;
    }

    gripper_closed_ = false; //since front_pos opens the gripper

    if (go_to_pose(manip_pos_, spdParams_.offviewSpeed) != 0)
        return 0;
    return 1;
}


/**
  *  Get an IK solution for the proposed points, in some coordinates, and returns 0 if
  *  the proposed pose is feasible and an error code otherwise.
  */
// return error codes:
//
// 0: OK
// 1: no IK solution found
// 2: IK solutions require turning backwards
// 6: move gripper into ground
// 7: user stopped motion
// 8: other unknown error
int ManipArmIF::go_to_ikpose(tf::Pose gripper_pose, double vel)  {

    tf::Vector3 coords = gripper_pose.getOrigin();
    if (coords[2] < gripper_length_) {
        ROS_ERROR("[ManipArmIF] Illegal z-coord %f would put grippers through table_plane.", coords[2]);
        return 6;  // ERRORCODE 6 - move gripper into ground
    }
    tf::Stamped<tf::Pose> ps(gripper_pose, ros::Time(0), arm_ref_);

    // IK
    std::vector<armlib::js_vect> vsolutions;
    bool ikSuccess = arm_ik_for_pose(ps, vsolutions);

    double rangle, pangle, yangle;
    gripper_pose.getBasis().getRPY(rangle,pangle,yangle);
    ROS_INFO("go_to_ikpose");
    ROS_INFO("  input coord %f,%f,%f", coords[0], coords[1], coords[2]);
    ROS_INFO("  input rpy %f,%f,%f",rangle,pangle,yangle);
    ROS_INFO("  input vel %f",vel);

    // If no solutions at all were found, escape.
    if (!ikSuccess) {
        ROS_ERROR("[ManipArmIF] No IK found for %f,%f,%f,%f,%f,%f", \
                  coords[0], coords[1], coords[2], \
                  rangle,pangle,yangle);
        sleep(1);
        return 1;  // ERRORCODE 1 - no IK solution found
    }

    //set pos to best solution
    armlib::js_vect pos;
    arm_get_nearest_legal_solution(vsolutions, pos);

    //check if torso's backwards. No longer belongs in feasibility check,
    //but might get reinstated.
    bool torso_forward = torso_forward_p(pos); //pos getting assigned
    if (!torso_forward) {
        ROS_WARN("[ManipArmIF] No non-backwards IK found for %f,%f,%f,%f,%f,%f", \
                 coords[0], coords[1], coords[2], \
                 rangle,pangle,yangle);

        sleep(1);
        return 2;  // ERRORCODE 2 - IK solutions require turning backwards
    }

    //move to found solution
    armlib::js_vect current_pos;
    robot_->get_actual_joint_pos(current_pos);
    pos.push_back(current_pos[6]); // don't change the gripper

    if (go_to_pose(pos,vel) == 2)
        return 6; // ERRORCODE 6 - user stopped motion

    return 0;  // ERRORCODE 0 - all's well
}

int ManipArmIF::go_to_ikpose( \
        double rollangle, double pitchangle, double yawangle, \
        tf::Vector3 coords, double vel)  {
    tf::Quaternion q; q.setRPY(rollangle,pitchangle,yawangle);
    tf::Pose pose(q, coords);
    return go_to_ikpose(pose,vel);
}

// NOTE - Clamping roll and pitch angles of gipper, safer IK
int ManipArmIF::go_to_ikpose(double yawangle, tf::Vector3 coords, double vel) {
    tf::Quaternion q; q.setRPY(M_PI, 0.0 ,yawangle); // 1.0 < yawangle < 6.0 (ik algorithm limitation)
    tf::Pose pose(q, coords);
    return go_to_ikpose(pose, vel);
}


/**
  *  Stop pick or place motion, then go back to manip_pos.
  *  Try to go to prev_coord first then move to "manip_pos_".
  *  To skip prev_coord, give default value.
  */
// return error codes:
//
// 0: everything is good
// 1: going to prev_coord failed
int ManipArmIF::ppStop(double yawangle, tf::Vector3 coords) {
    sleep(1);
    int retval = 0;
    if (!(((coords[0] == coords[1]) == coords[2]) == 0))
        if (go_to_ikpose(yawangle,coords,spdParams_.gotoXSpeed) != 0) {
            retval = 1;
            ROS_WARN("[ManipArmIF] failed to reach prev_pose, moving to manip_pos after 1.0");
            sleep(1);
        }
    go_to_pose(manip_pos_,spdParams_.gotoXSpeed);

    return retval;
}


// return error codes:
//
// 0: everything is good
// 1: grasping failed
// 2: going above object failed
// 3: getting close to obejct float64failed
// 8: unknown error
//
// finish positions
//   grasping sucess - above object
//   grasping fail   - manip_pos (all errorcode except 8)
//   unknown error   - wherever motion ended
int ManipArmIF::pick_up_object(double yawangle, tf::Vector3 coords) {
    // starting position check attempt
    if (!is_joint_at(manip_pos_, one_degree_tolerances_)) {
        ROS_ERROR("[ManipArmIF] joints are not in ""manip"" position.");
        return false;
    }

    // Move gripper above the target object
    ROS_INFO("Move to above object");
    tf::Vector3 above_coords;
    above_coords[0] = coords[0];
    above_coords[1] = coords[1];
    above_coords[2] = move_height_;
    if (go_to_ikpose(yawangle, above_coords, spdParams_.hoverSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to above object");
        ppStop(yawangle, above_coords);
        return 2; // ERRORCODE: going above object failed
    }

    ROS_INFO("Opening gripper");
    open_gripper();

    ROS_INFO("Going down");
    int retval2 = go_to_ikpose(yawangle, coords, spdParams_.manipSpeed);
    if (retval2 != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to object");
        ppStop(yawangle, above_coords);
        return 3; // ERRORCODE: getting close to obejct failed
    }

    ROS_INFO("Closing gripper");
    close_gripper();
    if (!is_gripper_fully_closed()) {   // Success
    //if (true) {   // DEBUG
        ROS_INFO("Grasping success!");
        if (go_to_ikpose(yawangle, above_coords, spdParams_.manipSpeed) != 0) {
            ROS_ERROR("[ManipArmIF] Failed to go back to above object - object in gripper");
            return 8;
        }
    } else {                            // Failed
        ROS_INFO("Grasping failed...");
        open_gripper();
        if (go_to_ikpose(yawangle, above_coords, spdParams_.manipSpeed) != 0) {
            ROS_ERROR("[ManipArmIF] Failed go back to above object - no object in gripper");
            return 8;
        }
        go_to_pose(manip_pos_,spdParams_.gotoXSpeed); // slow backtrack
        return 1; // ERRORCODE: Grasping failed
    }
    return 0;
}

// return error codes:
//
// 0: everything is good
//
// 2: going above target position failed
// 3: going down to target position failed
// 8: unknown error
//
// finish position - manip_pos
int ManipArmIF::put_down_object(double yawangle, tf::Vector3 coords) {

    // Move gripper above the target place
    ROS_INFO("Move to above target");
    tf::Vector3 above_coords;
    above_coords[0] = coords[0];
    above_coords[1] = coords[1];
    above_coords[2] = move_height_;
    if (go_to_ikpose(yawangle, above_coords, spdParams_.hoverSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to above object");
        ppStop(yawangle,above_coords);
        return 2;
    }

    ROS_INFO("Going down");
    if (go_to_ikpose(yawangle, coords, spdParams_.manipSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to object");
        ppStop(yawangle,above_coords);
        return 3;
    }

    ROS_INFO("Opening gripper");
    open_gripper();
    if (go_to_ikpose(yawangle, above_coords, spdParams_.manipSpeed) != 0) {
        ROS_ERROR("Can't go back to above object");
        return 8;
    }
    go_to_pose(manip_pos_,spdParams_.hoverSpeed);

    return 0;
}

// return error codes:
//
// 0: everything is good
//
// 2: pick up phase failed
// 3: put down phase failed
// 8: unknown error
int ManipArmIF::move_object(double src_yangle, tf::Vector3 src_coords, double tgt_yangle, tf::Vector3 tgt_coords) {
    int retval = pick_up_object(src_yangle, src_coords);
    if (retval == 8) {
        ROS_ERROR("[ManipArmIF] UNKNOWN ERROR");
        return retval;
    } else if (retval != 0) {
        ROS_ERROR("[ManipArmIF] Pick up failed");
        return 2;
    }

    int retval2 = put_down_object(tgt_yangle, tgt_coords);
    if (retval2 != 0) {
        ROS_ERROR("[ManipArmIF] UNKNOWN ERROR");
        return retval;
    } else if (retval2 != 0) {
        ROS_ERROR("Put down failed");
        return 3;
    }

    return 0;
}

int ManipArmIF::move_object_to_left(double src_yangle, tf::Vector3 src_coords) {
    double tgt_angle = M_PI;
    tf::Vector3 tgt_coords(0.3,0.0,object_height_);
    return move_object(src_yangle,src_coords,tgt_angle,tgt_coords);
}

int ManipArmIF::move_object_to_right(double src_yangle, tf::Vector3 src_coords) {
    double tgt_angle = M_PI;
    tf::Vector3 tgt_coords(0.6,0.0,object_height_);
    return move_object(src_yangle,src_coords,tgt_angle,tgt_coords);
}

int ManipArmIF::move_object_to_offtable(double src_yangle, tf::Vector3 src_coords) {
    double tgt_angle = M_PI;
    tf::Vector3 tgt_coords(0.3,-0.35,object_height_);
    return move_object(src_yangle,src_coords,tgt_angle,tgt_coords);
}


// return error codes:

// 0: everything is good
// 2: going above object failed
// 3: going to object failed
// 4: going to target position failed
// 5: going to above object at target postion failed

// finish positions - manip_pos
int ManipArmIF::push_object(double src_yangle, tf::Vector3 src_coords, \
                double tgt_yangle, tf::Vector3 tgt_coords) {

    // Set gripper angle
    armlib::js_vect pos;
    robot_->get_actual_joint_pos(pos);
    pos[6] = 0.13; //max open is ~0.785 == 45 degrees
    gripper_closed_ = false;
    go_to_pose(pos);


    // Move gripper above the target place
    ROS_INFO("Move to above target");
    tf::Vector3 above_coords;
    above_coords[0] = src_coords[0];
    above_coords[1] = src_coords[1];
    above_coords[2] = move_height_;
    if (go_to_ikpose(src_yangle, above_coords, spdParams_.hoverSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to above object position");
        ppStop(src_yangle,above_coords);
        return 2;
    }

    ROS_INFO("Going down");
    if (go_to_ikpose(src_yangle, src_coords, spdParams_.manipSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to object");
        ppStop(src_yangle,above_coords);
        return 3;
    }

    ROS_INFO("Push start");
    if (go_to_ikpose(tgt_yangle, tgt_coords, spdParams_.manipSpeed) != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to the target position");
        ppStop(src_yangle,above_coords);
        return 4;
    }

    ROS_INFO("Go to target hover height");
    tf::Vector3 tgt_aboveCoords;
    tgt_aboveCoords[0] = tgt_coords[0];
    tgt_aboveCoords[1] = tgt_coords[1];
    tgt_aboveCoords[2] = move_height_;
    int retval4 = go_to_ikpose(tgt_yangle, tgt_aboveCoords, spdParams_.manipSpeed);
    if (retval4 != 0) {
        ROS_ERROR("[ManipArmIF] Can't go to above object at the target position");
        ppStop(src_yangle,above_coords);
        return 5;
    }
    go_to_pose(manip_pos_,spdParams_.hoverSpeed);

    return 0;
}

int ManipArmIF::push_object_left(tf::Vector3 coords) {
    coords[0] = coords[0] + 0.03;
    coords[2] = 0.1; // known height
    tf::Vector3 endCoords;
    endCoords[0] = coords[0] - 0.25;
    endCoords[1] = coords[1];
    endCoords[2] = coords[2] + 0.01;
    //endCoords[2] = 0.11; // known height
    return push_object(M_PI, coords, M_PI, endCoords);
}

int ManipArmIF::push_object_right(tf::Vector3 coords) {
    coords[0] = coords[0] - 0.03;
    coords[2] = 0.1; // known height
    tf::Vector3 endCoords;
    endCoords[0] = coords[0] + 0.25;
    endCoords[1] = coords[1];
    endCoords[2] = coords[2] + 0.01;
    //endCoords[2] = 0.11; // known height
    return push_object(M_PI, coords, M_PI, endCoords);
}

int ManipArmIF::push_object_down(tf::Vector3 coords) {
    coords[1] = coords[1] + 0.03;
    coords[2] = 0.1; // known height
    tf::Vector3 endCoords;
    endCoords[0] = coords[0];
    endCoords[1] = coords[1] - 0.3;
    endCoords[2] = coords[2] + 0.01;
    //endCoords[2] = 0.11; // known height
    return push_object(M_PI/2.0, coords, M_PI/2, endCoords);
}


BasicManipStateMachine::BasicManipStateMachine(ros::NodeHandle nh, SpeedParams spdParams, char startPosID)
    : ManipArmIF(nh, spdParams)
{
    // Initialize states
    BackState_ = boost::shared_ptr<BackState>(new BackState(this));
    FrontState_ = boost::shared_ptr<FrontState>(new FrontState(this));
    ManipReadyState_ = boost::shared_ptr<ManipReadyState>(new ManipReadyState(this));
    MoveGripperState_ = boost::shared_ptr<MoveGripperState>(new MoveGripperState(this));
    OffViewState_ = boost::shared_ptr<OffViewState>(new OffViewState(this));

    // Setting initial state
    if (startPosID == 'b') {
        if (!is_joint_at(back_pos_, one_degree_tolerances_)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{back}!");
            exit(-1);
        } else {
            state_ = BackState_;
            ROS_INFO("Setting start state to BACK");
        }
    } else if (startPosID == 'f') {
        if (!is_joint_at(front_pos_, one_degree_tolerances_)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{front}!");
            exit(-1);
        } else {
            state_ = FrontState_;
            ROS_INFO("Setting start state to FRONT");
        }
    } else if (startPosID == 'm') {
        if (!is_joint_at(manip_pos_, one_degree_tolerances_)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{manipready}!");
            exit(-1);
        } else {
            state_ = ManipReadyState_;
            ROS_INFO("Setting start state to MANIPREADY");
        }
    } else if (startPosID == 'v') {
        if (!is_joint_at(offview_pos_, one_degree_tolerances_)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{offview}!");
            exit(-1);
        } else {
            state_ = OffViewState_;
            ROS_INFO("Setting start state to OFFVIEW");
        }
    } else {
        ROS_ERROR_STREAM("Unknown input = " << startPosID);
        exit(-1);
    }

    // ROS related
    basic_movements_ = nh_global_.advertiseService("basic_movements", &BasicManipStateMachine::basic_movements_srv, this);
    move_gripper_ = nh_global_.advertiseService("move_gripper", &BasicManipStateMachine::move_gripper_srv, this);
    manip_object_simple_ = nh_global_.advertiseService("manip_object_simple", &BasicManipStateMachine::manip_object_simple_srv, this);
}

bool BasicManipStateMachine::moveToBack() { return state_->moveToBack(); }
bool BasicManipStateMachine::moveToFront() { return state_->moveToFront(); }
bool BasicManipStateMachine::moveToManipReady() { return state_->moveToManipReady(); }
bool BasicManipStateMachine::moveToOffView() { return state_->moveToOffView(); }
int BasicManipStateMachine::moveGripperGlobal(tf::Pose pose, double vel) {
    return state_->moveGripper(pose, vel);
}
int BasicManipStateMachine::manipObject(double yawangle, tf::Vector3 coords, int type) {
    return state_->manipObject(yawangle, coords, type);
}

bool BasicManipStateMachine::basic_movements_srv( \
        gambit_manip::BasicMovements::Request &req, \
        gambit_manip::BasicMovements::Response &resp)
{
    int movementID = req.movementID;
    resp.retval = resp.FAIL;

    if (movementID == req.BACK) {
        resp.retval = this->moveToBack();
        ROS_DEBUG_STREAM("req.BACK resp.retval = " << resp.retval);
    } else if (movementID == req.FRONT) {
        resp.retval = this->moveToFront();
        ROS_DEBUG_STREAM("req.FRONT resp.retval = " << resp.retval);
    } else if (movementID == req.MANIPREADY) {
        resp.retval = this->moveToManipReady();
        ROS_DEBUG_STREAM("req.MANIPREADY resp.retval = " << resp.retval);
    } else if (movementID == req.OFFVIEW) {
        resp.retval = this->moveToOffView();
        ROS_DEBUG_STREAM("req.OFFVIEW resp.retval = " << resp.retval);
    } else {
        std::cout << "unknown command = " << movementID << std::endl;
    }
    return true;
}

bool BasicManipStateMachine::move_gripper_srv( \
        gambit_manip::MoveGripper::Request &req, \
        gambit_manip::MoveGripper::Response &resp)
{
    tf::Quaternion q; q.setRPY(req.rollangle,req.pitchangle,req.yawangle);
    tf::Vector3 coords(req.to_x,req.to_y,req.to_z);
    tf::Pose pose(q, coords);

    resp.retval = moveGripperGlobal(pose,req.vel);
    ROS_DEBUG_STREAM("move_gripper_srv resp.retval = " << resp.retval);
    return (true);
}

bool BasicManipStateMachine::manip_object_simple_srv( \
        gambit_manip::ManipObjectSimple::Request &req, \
        gambit_manip::ManipObjectSimple::Response &resp)
{
    double lowLimit = 0.072;

    double z = req.from_z;
    if (z > move_height_) {
        ROS_WARN_STREAM("[BasicManipStateMachine] Input req.from_z = " << z << " higher then move_height_ " << move_height_);
        ROS_WARN_STREAM("[BasicManipStateMachine] setting z to move_height_ " << move_height_);
        z = move_height_ - 0.001;
    } else if (z < lowLimit) {
        ROS_WARN_STREAM("[BasicManipStateMachine] Input req.from_z = " << z << " too close to lower-limit " << lowLimit);
        ROS_WARN_STREAM("[BasicManipStateMachine] setting z to lower-limit " << lowLimit);
        z = lowLimit;
    }


    tf::Vector3 coords(req.from_x,req.from_y,z);
    ROS_INFO("Calling moveToManipReady()");
    moveToManipReady();
    ROS_INFO("Calling manipObject()");
    resp.retval = manipObject(req.yawangle,coords,req.type);
    ROS_DEBUG_STREAM("manipObject resp.retval = " << resp.retval);
    ROS_INFO("Calling moveToOffView()");
    moveToOffView();
    return (true);
}


// BackState
bool BackState::moveToFront() {
    int retval = machine_->go_from_back_to_front();
    if (retval)
        machine_->setState(machine_->getFrontState());
    return retval;
}

// FrontState
bool FrontState::moveToBack() {
    int retval = machine_->go_from_front_to_back();
    if (retval)
        machine_->setState(machine_->getBackState());
    return retval;
}

bool FrontState::moveToManipReady() {
    int retval = machine_->go_from_front_to_manip();
    if (retval)
        machine_->setState(machine_->getManipReadyState());
    return retval;
}

// ManipReadyState
bool ManipReadyState::moveToFront() {
    int retval = machine_->go_from_manip_to_front();
    if (retval)
        machine_->setState(machine_->getFrontState());
    return retval;
}

bool ManipReadyState::moveToOffView() {
    int retval = machine_->go_from_manip_to_offview();
    if (retval)
        machine_->setState(machine_->getOffViewState());
    return retval;
}

int ManipReadyState::moveGripper(tf::Pose pose, double vel) {
    int retval = machine_->go_to_ikpose(pose, vel);
    if (retval == 0)
        machine_->setState(machine_->getMoveGripperState());
    return retval;
}

int ManipReadyState::manipObject(double yawangle, tf::Vector3 coords, int type) {
    int retval = -1;
    switch (type) {
    case 0:
        retval = machine_->move_object_to_left(yawangle, coords);
        break;
    case 1:
        retval = machine_->move_object_to_right(yawangle, coords);
        break;
    case 2:
        retval = machine_->move_object_to_offtable(yawangle, coords);
        break;

    case 3:
        retval = machine_->push_object_left(coords);
        break;
    case 4:
        retval = machine_->push_object_right(coords);
        break;
    case 5:
        retval = machine_->push_object_down(coords);
        break;

    default:
        retval = -1;
        ROS_INFO_STREAM("Unknown type " << type);
    }

    return retval;
}

// MoveGripperState - gripper can be located in any workspace
// NO SAFTY CHECK
bool MoveGripperState::moveToManipReady() {
    machine_->go_to_manip();
    machine_->setState(machine_->getManipReadyState());
    return true;
}

int MoveGripperState::moveGripper(tf::Pose pose, double vel) {
    return machine_->go_to_ikpose(pose, vel);
}


// OffViewState
// NO SAFTY CHECK
bool OffViewState::moveToManipReady() {
    machine_->go_from_offview_to_manip();
    machine_->setState(machine_->getManipReadyState());
    return true;
}
