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
 * A group of actuators comprising a part of the robot that works together, such
 * as the wrist.
 */

#ifndef __gambit_driver_actuator_group_h_
#define __gambit_driver_actuator_group_h_

#include <ros/ros.h>

#include <vector>

#include <gambit_driver/actuator.h>
#include <gambit_driver/DOF.h>
#include <sensor_msgs/JointState.h>

namespace gambit_driver {

class ActuatorGroup {
    public:

    ActuatorGroup();

    /**
     * Initializes the actuator group.
     */
    virtual void initialize();

    virtual bool check_connectivity();

    virtual void control();
   
    /**
     * Reads the status of all of the actuators that comprise the group.  Normally this is
     * called from the control loop.
     */
    virtual void read_status();
    
    /**
     * Sends control information (e.g. new positions) to the actuators.  Normally this is 
     * called from the control loop.
     */
    virtual void send_control();

    /**
     * When called once on each iteration of the control loop, collects timing statistics
     * and occasionally prints them out as ROS_DEBUG messages
     */
    void control_loop_timing_monitor();

    /**
     * Returns the list of DOFs in this group.
     *
     * @return a pointer to the vector of DOFs
     */
    inline std::vector<DOF*> *get_DOFs() { return &DOFs; }

    /**
     * Adds information about the DOFs in this ActuatorGroup to a ROS sensor_msgs::JointState
     * message.
     *
     * @param js a sensor_msgs::JointState to which the DOF state will be appended
     */
    virtual void get_ros_jointstate(sensor_msgs::JointState &js);

    /**
     * Loads calibration parameters (joint offsets) from the ROS parameter server.
     */
    virtual void load_calibration_parameters();

    protected:
    std::vector<Actuator*> actuators;
    std::vector<DOF*> DOFs;

    ros::Time last_control_time;
    ros::Time last_status_print_time;
    double avg_control_duration;
};

}

#endif // __gambit_driver_actuator_group_h_

