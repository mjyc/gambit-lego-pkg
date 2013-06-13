#ifndef __armlib_hybrid_h_
#define __armlib_hybrid_h_

#include <ros/callback_queue.h>
#include <armlib/arm.h>
#include <gambit_msgs/JointState.h>
#include <gambit_msgs/JointTargets.h>


namespace armlib {

/**
 * @brief High-level control interface for the Barrett WAM.
 */
class Hybrid : public Arm {
    public:
    Hybrid();
    virtual bool check_joint_limits(js_vect &position);
    virtual bool check_arm_joint_limits(js_vect &position);
    virtual bool check_manip_joint_limits(js_vect &position);

    virtual bool inverse_kinematics(double *position, double *orientation,
        std::vector<js_vect> &solutions);

    protected:
    virtual bool get_encoder_pos(js_vect &position);
    virtual bool set_target_pos(js_vect &position);

    /**
     * @brief ROS callback function to receive JointState messages.
     *
     * This information will be used to read the present encoder values.
     */
    void joint_encoder_cb(const gambit_msgs::JointStateConstPtr &joints);

    bool _data_valid;
    js_vect _encoder_position;              /** The most recently received encoder values */
   
    ros::CallbackQueue _cb_queue;           /** Callback queue for hybrid subscribers */
    ros::AsyncSpinner _spinner;             /** Asynchronous spinner for hybrid subscribers */
    ros::Publisher _joint_target_pub;       /** Publisher for new joint targets */
    ros::Subscriber _joint_encoder_sub;     /** Subscriber for joint encoder values */
};

}

#endif // __armlib_hybrid_h_
