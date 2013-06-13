#ifndef TABLE_MANIP_H
#define TABLE_MANIP_H

#include <iostream>
#include <string>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <gambit_manip/arm_interface.h>
//#include <gambit_manip/BasicMovements.h>
//#include <gambit_manip/MoveGripper.h>


class TableManipStateMachine : public TableArmIF {
protected:
    //---- Manipulation State ----//
    boost::shared_ptr<class State> state_;
    boost::shared_ptr<State> BackState_;
    boost::shared_ptr<State> FrontState_;
    boost::shared_ptr<State> ManipReadyState_;
    boost::shared_ptr<State> MoveGripperState_;

    //---- ROS related ----//
    ros::NodeHandle nh_global_;
    ros::NodeHandle nh_local_;
    tf::TransformListener listener_;

    //---- service servers ----//
//    ros::ServiceServer basic_movements_;
//    ros::ServiceServer move_gripper_;

    //--- parameters ----//
//    std::string arm_ref_;
//    std::string table_plane_ref_;


public:
    typedef boost::shared_ptr<TableManipStateMachine> Ptr;

    static const double gripper_length_ = 0.07;
    static const double move_height_ = 0.2;
//    static const std::string default_arm_ref_;
//    static const std::string default_table_plane_ref_;

    TableManipStateMachine(ros::NodeHandle nh, char startPosID);

    //---- State transition related ----//
    bool moveToBack();
    bool moveToFront();
    bool moveToManipReady();
    int moveGripperGlobal(double rollangle, double pitchangle, double yawangle, \
                    tf::Vector3 coords);
    // current state setter
    void setState(boost::shared_ptr<State> newState) { state_ = newState; }
    // state getters
    boost::shared_ptr<State> getBackState() { return BackState_; }
    boost::shared_ptr<State> getFrontState() { return FrontState_; }
    boost::shared_ptr<State> getManipReadyState() { return ManipReadyState_; }
    boost::shared_ptr<State> getMoveGripperState() { return MoveGripperState_; }

    //---- ROS ----//
    //actually can be located outside of this class.
    bool basic_movements_srv(gambit_manip::BasicMovements::Request &req, \
                            gambit_manip::BasicMovements::Response &resp);
    bool move_gripper_srv(gambit_manip::MoveGripper::Request &req, \
                            gambit_manip::MoveGripper::Response &resp);

    //---- Robot control functionalities ----//
    void goToManipReady() { aif_obj_.go_to_manip(); } // NO SAFTY CHECK
    bool goFromBackToFront() { return aif_obj_.go_from_back_to_front(); }
    bool goFromFrontToBack() { return aif_obj_.go_from_front_to_back(); }
    bool goFromFrontToManipReady() { return aif_obj_.go_from_front_to_manip(); }
    bool goFromManipReadyToFront() { return aif_obj_.go_from_manip_to_front(); }

    int go_to_table_plane_coords( double rollangle, double pitchangle, double yawangle, \
            tf::Vector3 coords, armlib::js_vect& pos);


    int pick_up_object(double rollangle, double pitchangle, double yawangle, tf::Vector3 coords, armlib::js_vect &pos) {

        // Move gripper above the object
        tf::Vector3 aboveCoords(coords.getX(), coords.getY(), hover_height_);
        armlib::js_vect pos2 = pos;
        int retval = go_to_table_plane_coords(rollangle, pitchangle, yawangle, aboveCoords, pos2);
        if (retval != 0) {
            std::cerr << "Failed to go above the object" << std::endl;
            return retval;
        }

        // Opening a gripper
        std::cout << "Closing the" << std::endl;
        aif_obj_.open_gripper();

        // Go down...
        retval = go_to_table_plane_coords(rollangle, pitchangle, yawangle, coords, pos2);
        aif_obj_.close_gripper();

        if (!aif_obj_.is_gripper_fully_closed()) {
            std::cout << "grasping success" << std::endl;
            retval = go_to_table_plane_coords(rollangle, pitchangle, yawangle, aboveCoords, pos2);
        } else {
            std::cout << "grasping failed" << std::endl;
            aif_obj_.open_gripper();
            retval = go_to_table_plane_coords(rollangle, pitchangle, yawangle, aboveCoords, pos2);
        }
        return retval;
    }














    // when holding a object, raise how high before moving it
    double move_height_;
    // don't let gripper-tip go below board-plane
    double min_putdown_height_;

    int moveGripperGlobal(double rollangle, double pitchangle, double yawangle, \
                    tf::Vector3 coords);
    // current state setter
    void setState(boost::shared_ptr<State> newState) { state_ = newState; }
    // state getters
    boost::shared_ptr<State> getBackState() { return BackState_; }
    boost::shared_ptr<State> getFrontState() { return FrontState_; }
    boost::shared_ptr<State> getManipReadyState() { return ManipReadyState_; }
    boost::shared_ptr<State> getMoveGripperState() { return MoveGripperState_; }

};


// State Interface
class State : boost::noncopyable {
public:
    virtual bool moveToBack() { std::cout << "moveToBack() not defined" << std::endl; return 0; }
    virtual bool moveToFront() { std::cout << "moveToFront() not defined" << std::endl; return 0; }
    virtual bool moveToManipReady() { std::cout << "moveToManipReady() not defined" << std::endl; return 0; }
    virtual int moveGripper(double rollangle, double pitchangle, double yawangle, \
                            tf::Vector3 coords) { std::cout << "moveGripper() not defined" << std::endl; return 0; }
//    virtual int pickUp() {}
//    virtual int placeDown() {}
//    virtual int push() {}
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
    virtual int moveGripper(double rollangle, double pitchangle, double yawangle, \
                            tf::Vector3 coords);
    virtual std::string toString() { return "ManipReadyState"; }
private:
    BasicManipStateMachine* machine_;
};

class MoveGripperState : public State {
public:
    MoveGripperState(BasicManipStateMachine* machine) { machine_ = machine; }

    virtual bool moveToManipReady(); // NO SAFTY CHECK
    virtual int moveGripper(double rollangle, double pitchangle, double yawangle, \
                            tf::Vector3 coords);
    virtual std::string toString() { return "MoveGripperState"; }
private:
    BasicManipStateMachine* machine_;
};


#endif // TABLE_MANIP_H
