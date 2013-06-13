#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

#include <gazebo.hh>
#include <common/common.hh>
#include <common/Events.hh>
//#include <common/PID.hh>
//#include <physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

//#include <gambit_driver/robot.h>
#include <gambit_gazebo_driver/gazebo_arm.h>
#include <gambit_gazebo_driver/gazebo_wrist.h>
#include <gambit_gazebo_driver/robot4gazebo.h>

namespace gazebo {

class ROSModelPlugin : public ModelPlugin {

public:
    ROSModelPlugin() {

        // Start up ROS
        //std::string name = "ros_model_plugin_node";
        std::string name = "arm";
        int argc = 0;
        ros::init(argc, NULL, name);

    }
    ~ROSModelPlugin() {
        delete node_;
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr) {
        model_ = _model;

        // Initialize ROS
        node_ = new ros::NodeHandle("~");
        //sub_ = node_->subscribe<std_msgs::Float64>("x", 1000, &ROSModelPlugin::ROSCallback, this );

        // Initialize robot class
        //robot_ = boost::make_shared<gambit_driver::Robot>("gambit");
        robot_ = boost::make_shared<gambit_driver::Robot4Gazebo>("gambit");
        gambit_driver::GazeboArm *arm = new gambit_driver::GazeboArm(model_);
        gambit_driver::GazeboWrist *wrist = new gambit_driver::GazeboWrist(model_);
        robot_->add_actuator_group(arm);
        robot_->add_actuator_group(wrist);
        robot_->initialize();
        robot_->enable_dofs();

        this->update_connection_ = event::Events::ConnectWorldUpdateStart(
                    boost::bind(&ROSModelPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate() {
        robot_->process_pending_targets();
        robot_->step();
        ros::spinOnce();
    }


protected:
    static const int NUM_JOINTS_ = 8;
    static std::string JOINT_NAMES_[];

    // Gazebo related
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;

    // ROS related
    ros::NodeHandle* node_;
    //ros::Subscriber sub_;

    // Robot
    //boost::shared_ptr<gambit_driver::Robot> robot_;
    boost::shared_ptr<gambit_driver::Robot4Gazebo> robot_;
};
std::string ROSModelPlugin::JOINT_NAMES_[] = {"waist","shoulder","elbow","wrist_roll","wrist_pitch","wrist_yaw","gripper_left","gripper_right"};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}



