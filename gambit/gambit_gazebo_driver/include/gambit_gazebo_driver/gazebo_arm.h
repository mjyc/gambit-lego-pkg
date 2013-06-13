
#include <ros/ros.h>

#include <gazebo.hh>
//#include <common/common.hh>
//#include <common/Events.hh>
//#include <common/PID.hh>
//#include <physics/physics.hh>

#include <gambit_driver/actuator_group.h>
#include <gambit_driver/DOF.h>
#include <gambit_gazebo_driver/actuator_gazebo.h>

namespace gambit_driver {
class GazeboArm : public ActuatorGroup {
    public:

    GazeboArm(gazebo::physics::ModelPtr model);
    
    bool check_connectivity();
    void control();
	void update();

};

}

