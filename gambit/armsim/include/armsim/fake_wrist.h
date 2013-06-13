/**
 * @file
 * @author brian d. mayton <bmayton@bdm.cc>
 * @version 1.0
 *
 * @section license
 *
 * copyright (c) 2010, intel labs seattle
 * all rights reserved.
 *
 * redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - neither the name of intel labs seattle nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  this software is provided by the copyright holders and contributors "as is"
 *  and any express or implied warranties, including, but not limited to, the
 *  implied warranties of merchantability and fitness for a particular purpose
 *  are disclaimed. in no event shall the copyright holder or contributors be
 *  liable for any direct, indirect, incidental, special, exemplary, or
 *  consequential damages (including, but not limited to, procurement of
 *  substitute goods or services; loss of use, data, or profits; or business
 *  interruption) however caused and on any theory of liability, whether in
 *  contract, strict liability, or tort (including negligence or otherwise)
 *  arising in any way out of the use of this software, even if advised of the
 *  possibility of such damage.
 *
 * @section description
 *
 * Actuator group for fake wrist dofs.
 */

#include <ros/ros.h>

#include <gambit_driver/actuator_group.h>
#include <gambit_driver/actuator_dummy.h>
#include <gambit_driver/DOF.h>

#include <sensor_msgs/JointState.h>

namespace gambit_driver {
class FakeWrist : public ActuatorGroup {
public:
    const static double GRIPPER_LINKAGE_LENGTH = 0.030;
    const static double GRIPPER_HUB_RADIUS = 0.015;

    FakeWrist();
    
    bool check_connectivity();
    void control();

    void get_ros_jointstate(sensor_msgs::JointState &js);
};

}

