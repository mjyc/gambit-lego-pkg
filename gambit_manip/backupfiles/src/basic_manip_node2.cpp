/ boost
#include <boost/thread.hpp>
// local
#include <armlib/gambit.h>
#include <gambit_manip/basic_manip_node2.h>

typedef boost::mutex::scoped_lock Synchronize;
boost::mutex mutex_;

BasicManipStateMachine::BasicManipStateMachine (ros::NodeHandle nh, char startPosID) :
    nh_global_(nh),
    nh_local_("~"),
    listener_(),
    aif_obj_(nh)
{
    // initialize states
    BackState_ = boost::shared_ptr<BackState>(new BackState(this));
    FrontState_ = boost::shared_ptr<FrontState>(new FrontState(this));
    ManipReadyState_ = boost::shared_ptr<ManipReadyState>(new ManipReadyState(this));
    MoveGripperState_ = boost::shared_ptr<MoveGripperState>(new MoveGripperState(this));

    // SETTING INITIAL STATE
    //check joint conditions
    if (startPosID == 'b') {
        if (!aif_obj_.is_joint_at(aif_obj_.back_pos, aif_obj_.one_degree_tolerances)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{back}!");
            exit(-1);
        } else {
            state_ = BackState_;
            ROS_INFO("Setting start state to BACK");
        }
    } else if (startPosID == 'f') {
        if (!aif_obj_.is_joint_at(aif_obj_.front_pos, aif_obj_.one_degree_tolerances)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{front}!");
            exit(-1);
        } else {
            state_ = FrontState_;
            ROS_INFO("Setting start state to FRONT");
        }
    } else if (startPosID == 'm') {
        if (!aif_obj_.is_joint_at(aif_obj_.manip_pos, aif_obj_.one_degree_tolerances)) {
            ROS_ERROR("Gambit is NOT positioned at starting position{manipready}!");
            exit(-1);
        } else {
            state_ = ManipReadyState_;
            ROS_INFO("Setting start state to MANIPREADY");
        }
    } else {
        ROS_ERROR_STREAM("Unknown input = " << startPosID);
        exit(-1);
    }

    // robot
    robot_ = aif_obj_.get_armlib_obj();

    // ROS related
    // services provided here
    basic_movements_ = nh_global_.advertiseService("basic_movements", &BasicManipStateMachine::basic_movements_srv, this);
    move_gripper_ = nh_global_.advertiseService("move_gripper", &BasicManipStateMachine::move_gripper_srv, this);
    // parameter server - various counters, default values, etc.
    nh_local_.param<std::string>("arm_ref",arm_ref_,default_arm_ref_);
    nh_local_.param<std::string>("table_plane_ref",table_plane_ref_,default_table_plane_ref_);
}

bool BasicManipStateMachine::moveToBack() {
    return state_->moveToBack();
}
bool BasicManipStateMachine::moveToFront() {
    return state_->moveToFront();
}
bool BasicManipStateMachine::moveToManipReady() {
    return state_->moveToManipReady();
}
int BasicManipStateMachine::moveGripperGlobal(double rollangle, double pitchangle, double yawangle, \
                                        tf::Vector3 coords) {
    return state_->moveGripper(rollangle, pitchangle, yawangle, coords);
}

bool BasicManipStateMachine::basic_movements_srv( \
        gambit_manip::BasicMovements::Request &req, \
        gambit_manip::BasicMovements::Response &resp)
{
    int movementID = req.movementID;
    //---- BasicMovement services ----//
    if (movementID == req.BACK) {
        Synchronize sync(mutex_);
        resp.retval = this->moveToBack();
        ROS_DEBUG_STREAM("req.BACK resp.retval = " << resp.retval);
    } else if (movementID == req.FRONT) {
        Synchronize sync(mutex_);
        resp.retval = this->moveToFront();
        ROS_DEBUG_STREAM("req.FRONT resp.retval = " << resp.retval);
    } else if (movementID == req.MANIPREADY) {
        Synchronize sync(mutex_);
        resp.retval = this->moveToManipReady();
        ROS_DEBUG_STREAM("req.MANIPREADY resp.retval = " << resp.retval);
    } else {
        std::cout << "unknown command = " << movementID << std::endl;
    }

    return (true);
}

bool BasicManipStateMachine::move_gripper_srv( \
        gambit_manip::MoveGripper::Request &req, \
        gambit_manip::MoveGripper::Response &resp)
{
    Synchronize sync();
    tf::Vector3 coord(req.to_x,req.to_y,req.to_z);
    resp.retval = this->moveGripperGlobal( \
                req.gripper_rollangle, \
                req.gripper_pitchangle, \
                req.gripper_yawangle, \
                coord);
    ROS_DEBUG_STREAM("move_gripper_srv resp.retval = " << resp.retval);

    return (true);
}


/*
 *  Get an IK solution for the proposed points, in some coordinates, and returns 0 if
 *  the proposed pose is feasible and an error code otherwise.
 */
int BasicManipStateMachine::go_to_table_plane_coords( \
        double rollangle, double pitchangle, double yawangle, \
        tf::Vector3 coords, armlib::js_vect& pos)  {
    // return error codes:
    //
    // 0: OK
    // 1: no IK solution found
    // 6: move gripper into ground
    // 8: other unknown error

    if (coords[2] < gripper_length_) {
        ROS_ERROR("Illegal z-coord %f would put grippers through table_plane.", coords[2]);
        return 6;  // ERRORCODE 6 - move gripper into ground
    }

    try {
        // get transformation and desired pose in new coordinate frame
        ros::Time latest;
        listener_.getLatestCommonTime(table_plane_ref_, arm_ref_, latest, NULL);

        // The quaternion is translated into roll/pitch/yaw
        //Pi's to rotate around the Z axis so hand points down at table_plane
        //tf::Quaternion q; q.setRPY(rollangle, 0.0, -M_PI);
        //tf::Quaternion q; q.setRPY(rollangle, pitchangle, yawangle);
        // ONLY CONTROL YAW - THE OTHER TWO MIGHT REQUIRE GROSS ROTATION
        tf::Quaternion q; q.setRPY(M_PI, 0.0, yawangle);
        tf::Pose pose(q, coords);

        // transform things
        tf::Stamped<tf::Pose> ps(pose, latest, table_plane_ref_);
        tf::Stamped<tf::Pose> ps_to;
        listener_.transformPose(arm_ref_, ps, ps_to);

        // IK
        std::vector<armlib::js_vect> vsolutions;
        bool ikSuccess = aif_obj_.arm_ik_for_pose(ps_to, vsolutions);

        ROS_INFO("input coord %f,%f,%f", coords[0], coords[1], coords[2]);
        ROS_INFO("input rpy %f,%f,%f", rollangle,pitchangle,yawangle);

        // If no solutions at all were found, escape.
        if (!ikSuccess) {
            ROS_ERROR("No IK found for %f,%f,%f,%f,%f,%f", coords[0], coords[1], coords[2], \
                      rollangle, pitchangle, yawangle);
            sleep(1);
            return 1;  // ERRORCODE 1 - no IK solution found
        }

        //set pos to best solution
        aif_obj_.arm_get_best_legal_solution(vsolutions, pos);

        //check if torso's backwards. No longer belongs in feasibility check,
        //but might get reinstated.
        bool torso_forward = aif_obj_.torso_forward_p(pos); //pos getting assigned
        if (!torso_forward) {
            ROS_WARN("No non-backwards IK found for %f,%f,%f,%f,%f,%f", coords[0], coords[1], coords[2], \
                     rollangle, pitchangle, yawangle);

            sleep(1);
            return 2;  // ERRORCODE 2 - IK solutions require turning backwards
        }

        //move to found solution
        armlib::js_vect current_pos;
        robot_->get_actual_joint_pos(current_pos);
        pos.push_back(current_pos[6]);

        // [2012.05.13 MJYC] TODO: safety check before move!
        aif_obj_.go_to_pose(pos);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return 8; // ERRORCODE 8 - other unknown error
    }
    return 0;  // ERRORCODE 0 - all's well
}


//---- BackState
bool BackState::moveToFront() {
    int retval = machine_->goFromBackToFront();
    if (retval)
        machine_->setState(machine_->getFrontState());
    return retval;
}

//---- FrontState
bool FrontState::moveToBack() {
    int retval = machine_->goFromFrontToBack();
    if (retval)
        machine_->setState(machine_->getBackState());
    return retval;
}

bool FrontState::moveToManipReady() {
    int retval = machine_->goFromFrontToManipReady();
    if (retval)
        machine_->setState(machine_->getManipReadyState());
    return retval;
}

//---- ManipReadyState
bool ManipReadyState::moveToFront() {
    int retval = machine_->goFromManipReadyToFront();
    if (retval)
        machine_->setState(machine_->getFrontState());
    return retval;
}

int ManipReadyState::moveGripper(double rollangle, double pitchangle, double yawangle, tf::Vector3 coords) {
    armlib::js_vect pos;
//    int retval = machine_->go_to_table_plane_coords( \
//                rollangle, pitchangle, yawangle, coords, pos);
    int retval = machine_->pick_up_object(\
                rollangle, pitchangle, yawangle, coords, pos);
    if (retval == 0) {
        machine_->setState(machine_->getMoveGripperState());
        ROS_DEBUG("Succesfully moved to MoveGripper State");
    }
    return retval;
}

//---- MoveGripperState
// NO SAFTY CHECK
bool MoveGripperState::moveToManipReady() {
    machine_->goToManipReady();
    machine_->setState(machine_->getManipReadyState());
    return true;
}

int MoveGripperState::moveGripper(double rollangle, double pitchangle, double yawangle, tf::Vector3 coords) {
    armlib::js_vect pos;
    return machine_->go_to_table_plane_coords( \
                rollangle, pitchangle, yawangle, coords, pos);
}



void usage(char** argv) {
    using namespace std;
    cerr << "usage: " << argv[0] << " <posID>" << endl;
    cerr << "    posID options:" << endl;
    cerr << "        0 = back" << endl;
    cerr << "        1 = front" << endl;
    cerr << "        2 = manipready" << endl;
    exit(-1);
}

int main (int argc, char** argv)
{
    // Argument parsing
    std::string arg;
    if (argc > 1)
        arg = std::string (argv[1]);
    if (arg == "--help" || arg == "-h")
    {
        usage (argv);
        return 1;
    }
    char startPosID = 'b'; // default start is b
    if (argc > 1)
        startPosID = argv[1][0];
    ROS_INFO_STREAM("[parsing] posID = " << startPosID);


    // ROS node
    ros::init (argc, argv, "basic_manip_state_machine");
    ros::NodeHandle nh;

    std::cout << "Starting BasicManipStateMachine in 2sec..." << std::endl;
    sleep(2);
    BasicManipStateMachine state_machine(nh,startPosID);
    ros::spin();
    return 0;
}



    \
