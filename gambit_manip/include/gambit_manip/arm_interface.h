#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

#include <math.h>
#include <string>
#include <tf/tf.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <armlib/arm.h>


class ArmIF
{
protected:
    ros::NodeHandle nh_global_;

    boost::shared_ptr<armlib::Arm> robot_;
    boost::mutex stop_flag_mutex_;
    bool user_stopped_;
    bool gripper_closed_;

    double speed_;
    std::string arm_ref_;


public:
    // Constant variables
    static const double max_speed_ = 1.5;
    static const double min_speed_ = 0.1;
    static const int num_joints_ = 7;
    static const double closed_gripper_angle_ = -1.045;   //max close is  -1.047 == -60 degrees
    //static const double open_gripper_angle_ = 0.785;    //max open is    0.785 == 45 degrees
    static const double open_gripper_angle_ = -0.131;     //half open is  -0.131 == -7.5 degrees

    static armlib::js_vect make_pos_vector (float, float, float, float, float, float, float);
    static double speed_range_check(double speed);

    ArmIF (ros::NodeHandle nh);
    virtual ~ArmIF();

    bool arm_ik_for_pose(tf::Stamped<tf::Pose> ps_to, std::vector<armlib::js_vect>& vsolutions);
    bool arm_get_nearest_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos);
    bool is_joint_at(armlib::js_vect desired_pos, armlib::js_vect tolerances);
    int go_to_pose(armlib::js_vect pos, double vel);
    int go_to_pose(armlib::js_vect pos);
    bool torso_forward_p(armlib::js_vect& pos);
    void stop_motion();

    void open_gripper();
    void close_gripper();
    bool is_gripper_fully_closed();
    bool is_gripper_set_to_closed();

    void go_to_zero(); // WARN - NO SAFETY CHECK

    inline bool get_user_stopped_flag() { boost::lock_guard<boost::mutex> lock(stop_flag_mutex_); return user_stopped_; }
    inline void set_user_stopped_flag(bool flag) { boost::lock_guard<boost::mutex> lock(stop_flag_mutex_); user_stopped_ = flag; }
    inline double get_speed() { return speed_; }
    inline boost::shared_ptr<armlib::Arm> get_armlib_obj() { return robot_; }
}; // end class


struct SpeedParams {
    double gotoXSpeed;      // any unsafe goto
    double windingSpeed;    // back  <-> front
    double windingSpeed2;   // front <-> manip
    double hoverSpeed;      // move above object
    double manipSpeed;      // go down to object / come up from object
    double offviewSpeed;    // manip <-> offview
};


class ManipArmIF : public ArmIF
{
protected:
    SpeedParams spdParams_;
    double gripper_length_;
    double move_height_;
    // NOTE - ICRA2013 specific
    double object_height_;

public:
    // hardcoded joint poses
    static armlib::js_vect back_pos_;
    static armlib::js_vect front_pos_;
    static armlib::js_vect manip_pos_;
    static armlib::js_vect offview_pos_;
    // hardcoded tolerances
    static armlib::js_vect one_degree_tolerances_;

    ManipArmIF(ros::NodeHandle nh, SpeedParams spdParams);
    virtual ~ManipArmIF();

    // WARN - NO SAFETY CHECKS!
    void go_to_back();
    void go_to_front();
    void go_to_manip();
    void go_to_offview();

    // Script
    bool go_from_back_to_front();
    bool go_from_front_to_back();
    bool go_from_front_to_manip();
    bool go_from_manip_to_front();
    bool go_from_manip_to_offview();
    bool go_from_offview_to_manip();


    // WARN - be careful with the end point rotation, found solutions may require huge joint rotations.
    int go_to_ikpose(tf::Pose gripper_pose, double vel);
    int go_to_ikpose(double rollangle, double pitchangle, double yawangle, \
                     tf::Vector3 coords, double vel);
    int go_to_ikpose(double yawangle, tf::Vector3 coords, double vel); // only control yaw, much safe

    // Pick and place
    int ppStop(double yawangle, tf::Vector3 coords); // stop any pick, place motions - attempt to go prevCoord first
    int pick_up_object(double yawangle, tf::Vector3 coords);
    int put_down_object(double yawangle, tf::Vector3 coords);
    int move_object(double src_yangle, tf::Vector3 src_coords, double tgt_yangle, tf::Vector3 tgt_coords);
    int manip_pos_grasp(double yawangle);
    // NOTE - ICRA2013 specific
    int move_object_to_left(double src_yangle, tf::Vector3 src_coords);
    int move_object_to_right(double src_yangle, tf::Vector3 src_coords);
    int move_object_to_offtable(double src_yangle, tf::Vector3 src_coords);
    // NOTE - HRI2014 specific
    int grasp_n_put_object(double tgt_yangle, tf::Vector3 tgt_coords);
    int grasp_n_put_object_fast(double tgt_yangle, tf::Vector3 tgt_coords);

    // push
    int push_object(double src_yangle, tf::Vector3 src_coords, \
                    double tgt_yangle, tf::Vector3 tgt_coords);
    int push_object_left(tf::Vector3 coords);
    int push_object_right(tf::Vector3 coords);
    int push_object_down(tf::Vector3 coords);

};
// WARN - gripper is always open
armlib::js_vect ManipArmIF::back_pos_  = ArmIF::make_pos_vector(-3.1260, -3.10, 3.08,   -1.5391, -0.1772, 0.0460, ArmIF::open_gripper_angle_);
armlib::js_vect ManipArmIF::front_pos_ = ArmIF::make_pos_vector( 1.0722, -3.10, 3.08,   -1.4521,  0.0,    0.0460, ArmIF::open_gripper_angle_);
armlib::js_vect ManipArmIF::manip_pos_ = ArmIF::make_pos_vector( 0.2443,  0.90, 2.2024,  0.0,    -0.0376, 0.2446, ArmIF::open_gripper_angle_);
armlib::js_vect ManipArmIF::offview_pos_ = ArmIF::make_pos_vector( -1.2443,  0.90, 2.2024,  0.0,    -0.0376, 0.2446, ArmIF::open_gripper_angle_);

armlib::js_vect ManipArmIF::one_degree_tolerances_ = ManipArmIF::make_pos_vector(0.0174532925,0.0174532925,0.0174532925,0.0174532925,0.0174532925,0.0174532925,0.0174532925);





#include <boost/noncopyable.hpp>
#include "gambit_manip/BasicMovements.h"
#include "gambit_manip/MoveGripper.h"
#include "gambit_manip/ManipObjectSimple.h"


class BasicManipStateMachine : public ManipArmIF {
protected:
    // Manipulation State
    boost::shared_ptr<class State> state_;
    boost::shared_ptr<State> BackState_;
    boost::shared_ptr<State> FrontState_;
    boost::shared_ptr<State> ManipReadyState_;
    boost::shared_ptr<State> MoveGripperState_;
    boost::shared_ptr<State> OffViewState_;

    // ROS related
    ros::ServiceServer basic_movements_;
    ros::ServiceServer move_gripper_;
    ros::ServiceServer manip_object_simple_;


public:
    BasicManipStateMachine(ros::NodeHandle nh, SpeedParams spdParams, char startPosID);

    // State transition related

    boost::shared_ptr<State> getBackState() { return BackState_; }
    boost::shared_ptr<State> getFrontState() { return FrontState_; }
    boost::shared_ptr<State> getManipReadyState() { return ManipReadyState_; }
    boost::shared_ptr<State> getMoveGripperState() { return MoveGripperState_; }
    boost::shared_ptr<State> getOffViewState() { return OffViewState_; }
    void setState(boost::shared_ptr<State> newState) { state_ = newState; }

    bool moveToBack();
    bool moveToFront();
    bool moveToManipReady();
    bool moveToOffView();
    int moveGripperGlobal(tf::Pose pose, double vel);
    int manipObject(double yawangle, tf::Vector3 coords, int type);

    // ROS related
    bool basic_movements_srv(gambit_manip::BasicMovements::Request &req, \
                            gambit_manip::BasicMovements::Response &resp);
    bool move_gripper_srv(gambit_manip::MoveGripper::Request &req, \
                            gambit_manip::MoveGripper::Response &resp);
    bool manip_object_simple_srv(gambit_manip::ManipObjectSimple::Request &req, \
                            gambit_manip::ManipObjectSimple::Response &resp);
};


// State Interface

class State : boost::noncopyable {
public:
    virtual bool moveToBack() { std::cout << "moveToBack() not defined" << std::endl; return 0; }         // errorcode 0 - fail
    virtual bool moveToFront() { std::cout << "moveToFront() not defined" << std::endl; return 0; }       // errorcode 0 - fail
    virtual bool moveToManipReady() { std::cout << "moveToManip() not defined" << std::endl; return 0; }  // errorcode 0 - fail
    virtual bool moveToOffView() { std::cout << "moveToOffView() not defined" << std::endl; return 0; }  // errorcode 0 - fail
    virtual int moveGripper(tf::Pose pose, double vel) { std::cout << "moveGripper() not defined" << std::endl; return -1; }  // errorcode -1 - fail
    virtual int manipObject(double yawangle, tf::Vector3 coords, int type) { std::cout << "manipObject() not defined" << std::endl; return -1; }  // errorcode -1 - fail
    virtual std::string toString() = 0;
};

// Concrete State Implementations

class BackState : public State {
public:
    BackState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToFront();
    virtual std::string toString() { return "BackState"; }
private:
    BasicManipStateMachine* machine_;
};

class FrontState : public State {
public:
    FrontState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToBack();
    virtual bool moveToManipReady();
    virtual std::string toString() { return "FrontState"; }
private:
    BasicManipStateMachine* machine_;
};

class ManipReadyState : public State {
public:
    ManipReadyState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToFront();
    virtual bool moveToOffView();
    virtual int moveGripper(tf::Pose pose, double vel);
    virtual int manipObject(double yawangle, tf::Vector3 coords, int type);
    virtual std::string toString() { return "ManipReadyState"; }
private:
    BasicManipStateMachine* machine_;
};

class MoveGripperState : public State {
public:
    MoveGripperState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToManipReady(); // WARN - NO SAFTY CHECK
    virtual int moveGripper(tf::Pose pose, double vel);
    virtual std::string toString() { return "MoveGripperState"; }
private:
    BasicManipStateMachine* machine_;
};

class OffViewState : public State {
public:
    OffViewState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToManipReady(); // WARN - NO SAFTY CHECK
    virtual std::string toString() { return "OffView"; }
private:
    BasicManipStateMachine* machine_;
};

#endif // ARM_INTERFACE_H
