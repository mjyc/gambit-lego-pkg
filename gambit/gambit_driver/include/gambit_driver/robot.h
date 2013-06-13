/**
 * @file
 * @author Brian D. Mayton <bmayton@bdm.cc>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (c) 2010, Intel Labs Seattle
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - Neither the name of Intel Labs Seattle nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * Robot base class.
 */

#ifndef __gambit_driver_robot_h_
#define __gambit_driver_robot_h_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gambit_driver/DOF.h>
#include <gambit_driver/actuator_group.h>
#include <gambit_driver/stamped_target.h>

#include <gambit_msgs/JointState.h>
#include <gambit_msgs/JointTargets.h>
#include <gambit_msgs/SetDOFProperty.h>
#include <gambit_msgs/GetArmInfo.h>
#include <sensor_msgs/JointState.h>

#include <string>
#include <vector>
#include <queue>

#include <pthread.h>

namespace gambit_driver {

typedef void(*loop_callback)(void*);

typedef struct {
    loop_callback func;
    void *arg;
} loop_callback_entry;

class Robot {
public:
    Robot(std::string name);
    virtual ~Robot();

    void add_actuator_group(ActuatorGroup *ag);
    void initialize();
    void pub_state(const ros::TimerEvent &event);
    void pub_dof_state(const ros::TimerEvent &event);
    void register_callback(loop_callback fn, void *arg);

    virtual void process_pending_targets();
    virtual void step();
    virtual void driver_loop();

    /* Services */
    bool set_DOF_property(SetDOFProperty::Request &req, SetDOFProperty::Response &resp);
    bool get_arm_info(GetArmInfo::Request &req, GetArmInfo::Response &resp);

    /* Subscriptions */
    void targets_cb(const JointTargetsConstPtr &msg);

protected:
    bool initialized;
    std::vector <ActuatorGroup*> actuator_groups;
    std::vector <DOF*> DOFs;
    std::string robot_name;
    int next_dof_index;
    std::vector <loop_callback_entry> callbacks;

    double pub_timer_duration;
    bool use_client_timestamps;

    std::queue <StampedTarget> target_queue;
    pthread_mutex_t target_queue_lock;

    ros::AsyncSpinner joint_target_spinner;
    ros::CallbackQueue joint_target_callback_queue;

    ros::Publisher joint_state_pub;
    ros::Publisher ros_joint_state_pub;
    ros::Publisher dof_state_pub;
    ros::Timer state_pub_timer;
    ros::Timer dof_state_pub_timer;
    ros::ServiceServer get_arm_info_srv;
    ros::ServiceServer set_DOF_property_srv;
    ros::Subscriber targets_sub;
};

}

#endif // __gambit_driver_robot_h_

