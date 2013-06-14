#ifndef _arm_interface_h_
#define _arm_interface_h_

#include <string>
#include <tf/tf.h>
#include <armlib/arm.h>


class Arm_IF
{
protected:
    ros::NodeHandle nh_global_;

    //publishers
    ros::Publisher markerPublisher_;

    int markerId_;
    std::string frameId_;
    double speed_;
    double windingspeed_;

    static const int num_joints_ = 7;

    armlib::Arm *robot_;

public:

    bool gripper_closed_;

    static const double closed_gripper_angle_ = -1.045; // -1.045; //max open is ~ -1.047 == -60 degrees
    static const double open_gripper_angle_ = 0.785; // 0.785; //max open is ~0.785 == 45 degrees

    Arm_IF (ros::NodeHandle nh);

    ~Arm_IF();

    void go_to_zero();
    bool go_from_watch_to_move();
    bool go_from_move_to_watch();

    bool torso_forward_p(armlib::js_vect& pos);

    void mark_coord(tf::Stamped<tf::Pose> pos);
    bool arm_ik_for_pose(tf::Stamped<tf::Pose> ps_to, std::vector<armlib::js_vect>& vsolutions);
    bool arm_get_nearest_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos);
    bool arm_get_best_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos);
    armlib::js_vect make_pos_vector (float, float, float, float, float, float, float);
    int go_to_pose(armlib::js_vect pos);

    void open_gripper();
    void close_gripper();
    bool is_gripper_fully_closed();
    bool is_gripper_set_to_closed();

    inline armlib::Arm *get_armlib_obj() { return robot_; }



}; // end class

#endif // _arm_interface_h_
