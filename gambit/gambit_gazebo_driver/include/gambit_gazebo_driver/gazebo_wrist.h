
#include <ros/ros.h>

#include <gazebo.hh>
//#include <common/common.hh>
//#include <common/Events.hh>
//#include <common/PID.hh>
//#include <physics/physics.hh>

#include <sensor_msgs/JointState.h>

#include <gambit_driver/actuator_group.h>
#include <gambit_driver/DOF.h>
#include <gambit_gazebo_driver/actuator_gazebo.h>
#include <gambit_gazebo_driver/actuator_gazebo_gripper.h>

namespace gambit_driver {
class GazeboWrist : public ActuatorGroup {
public:
    const static double GRIPPER_LINKAGE_LENGTH = 0.030;
    const static double GRIPPER_HUB_RADIUS = 0.015;

    GazeboWrist(gazebo::physics::ModelPtr model);
    
    bool check_connectivity();
    void control();

    void get_ros_jointstate(sensor_msgs::JointState &js);

    void update();
};

}

