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
 * High-level definiton of the gambit wrist.
 */

#ifndef __gambit_driver_wrist_h_
#define __gambit_driver_wrist_h_

#include <ros/ros.h>

#include <gambit_driver/actuator_group.h>

#include <bus/interface.h>

namespace gambit_driver {

/**
 * @brief High-level definition of the newarm wrist.
 * 
 * This class defines the high-level structure of the newarm wrist.  It constructs the DOFs
 * and actuators and defines the relationships between them.  It also implements the
 * control loop for all of these DOFs/actuators.
 */
class Wrist : public ActuatorGroup {
    public:
    /**
     * Constructs a new Wrist.
     *
     * @param interface the communication bus used for communication with all of the wrist
     *  actuators
     */
    Wrist(bus::Interface *interface);

    /**
     * Default destructor.
     */
    ~Wrist();

    /**
     * Performs a connectivity check with all of the actuators that comprise the wrist.
     * Return status is only true if all actuators are present and responding.  ROS error
     * messages are printed out if actuators do not respond.
     *
     * @return whether all actuators are present on the bus and responsive to commands
     */
    bool check_connectivity();

    /**
     * Sets up the initial state of the actuators.  Verifies that all actuators start up
     * disabled, and that all of their commanded positions are initialized to their actual
     * positions.
     */
    void initialize();

    void get_ros_jointstate(sensor_msgs::JointState &js);

    /**
     * Performs one iteration of the control loop.
     */
    void control();

    protected:
    const static double GRIPPER_LINKAGE_LENGTH = 0.030;
    const static double GRIPPER_HUB_RADIUS = 0.015;

    bus::Interface *iface;
};

}

#endif // __gambit_driver_wrist_h_

