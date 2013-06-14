#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

#include <math.h>
#include <string>
#include <tf/tf.h>
#include <armlib/arm.h>

// This class offers easy way to keep track of the gripper
// status and using IK.
// [WARNING] using robot_ to control arm will not gaurantee
// correct tracking of the gripper status
class Arm_IF
{
protected:
    ros::NodeHandle nh_global_;
    ros::Publisher markerPublisher_;

    int markerId_;
    std::string frameId_;
    double startspeed_;
    double speed_;
    double windingspeed_;

    static const int num_joints_ = 7;

    armlib::Arm *robot_;

public:

    bool gripper_closed_;

    // Constant variables
    static const double closed_gripper_angle_ = -1.045; //max close is ~ -1.047 == -60 degrees
    static const double open_gripper_angle_ = 0.785; //max open is ~0.785 == 45 degrees
    //hand coded positions
    static armlib::js_vect back_pos;
    static armlib::js_vect front_pos;
    static armlib::js_vect manip_pos;
    //hand coded tolerances
    static armlib::js_vect one_degree_tolerances;

    Arm_IF (ros::NodeHandle nh);
    ~Arm_IF();

    // Go to hand coded position functions
    // has specific checks, specific way of moving
    // [WARNING] some of them does NOT have safety checks!
    void go_to_zero();
    void go_to_start();
    void go_to_front();
    void go_to_manip();
    bool go_from_back_to_front();
    bool go_from_front_to_manip();
    bool go_from_manip_to_front();
    bool go_from_front_to_back();
    //scripts
    bool unwind();
    bool wind();

    // Utility functions
    static armlib::js_vect make_pos_vector (float, float, float, float, float, float, float);

    void mark_coord(tf::Stamped<tf::Pose> pos);
    bool arm_ik_for_pose(tf::Stamped<tf::Pose> ps_to, std::vector<armlib::js_vect>& vsolutions);
    bool arm_get_nearest_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos);
    bool arm_get_best_legal_solution(std::vector<armlib::js_vect> vsolutions, armlib::js_vect& pos);
    bool is_joint_at(armlib::js_vect desired_pos, armlib::js_vect tolerances);
    int go_to_pose(armlib::js_vect pos, double vel);
    int go_to_pose(armlib::js_vect pos);

    bool torso_forward_p(armlib::js_vect& pos);

    void open_gripper();
    void close_gripper();
    bool is_gripper_fully_closed();
    bool is_gripper_set_to_closed();

    inline armlib::Arm *get_armlib_obj() { return robot_; }

}; // end class

//--- Hand coded variables ----//
//joint pos - gripper is always open
armlib::js_vect Arm_IF::back_pos = Arm_IF::make_pos_vector(-3.1259665489196777, -3.10, 3.08, -1.539094090461731, -0.17719179391860962, 0.046019721776247025, Arm_IF::open_gripper_angle_);
armlib::js_vect Arm_IF::front_pos = Arm_IF::make_pos_vector(1.0722, -3.10, 3.08, -1.4521, 0.0, 0.0460, Arm_IF::open_gripper_angle_);
armlib::js_vect Arm_IF::manip_pos = Arm_IF::make_pos_vector(0.2443, 0.9000, 2.2024, 0.0, -0.0376, 0.2446, Arm_IF::open_gripper_angle_);
//tolerance
armlib::js_vect Arm_IF::one_degree_tolerances = Arm_IF::make_pos_vector(0.0174532925,0.0174532925,0.0174532925,0.0174532925,0.0174532925,0.0174532925, Arm_IF::open_gripper_angle_);


#endif // ARM_INTERFACE_H
