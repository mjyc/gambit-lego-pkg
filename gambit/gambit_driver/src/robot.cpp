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

#include <gambit_driver/robot.h>

namespace gambit_driver {

Robot::Robot(std::string name) :
    joint_target_spinner(1, &joint_target_callback_queue)
{
    initialized = false;
    robot_name = name;
    next_dof_index = 0;

    pub_timer_duration = 0.02;
}

Robot::~Robot() {
    for(unsigned int i=0; i<actuator_groups.size(); i++) {
        delete actuator_groups[i];
    }
}

void Robot::add_actuator_group(ActuatorGroup *ag) {
    if(initialized) {
        ROS_ERROR("Attempting to add an actuator group after initialization");
        return;
    }

    actuator_groups.push_back(ag);
    
    std::vector<DOF*> *ag_dofs = ag->get_DOFs();
    for(unsigned int i=0; i<ag_dofs->size(); i++) {
        DOF *d = ag_dofs->at(i);
        d->set_index(next_dof_index);
        DOFs.push_back(d);
        ROS_INFO(" [%d] %s", next_dof_index, d->get_name().c_str());
        next_dof_index++;
    }
}

void Robot::initialize() {
    for(unsigned int i=0; i<actuator_groups.size(); i++) {
        actuator_groups[i]->initialize();
        actuator_groups[i]->check_connectivity();
    }
    
    ros::NodeHandle nh("~");

    /* Parameters */
    use_client_timestamps = false;
    nh.getParam("use_client_timestamps", use_client_timestamps);
    
    /* Publishers */
    joint_state_pub = nh.advertise<JointState>("joint_state", 1);
    ros_joint_state_pub = nh.advertise<sensor_msgs::JointState>("ros_joint_state", 1);
    dof_state_pub = nh.advertise<DOFProperty>("dof_state", 1);

    /* Timers */
    state_pub_timer = nh.createTimer(ros::Duration(pub_timer_duration),
        &Robot::pub_state, this);
    dof_state_pub_timer = nh.createTimer(ros::Duration(2.0),
        &Robot::pub_dof_state, this);

    /* Services */
    get_arm_info_srv = nh.advertiseService("get_arm_info", &Robot::get_arm_info, this);
    set_DOF_property_srv = nh.advertiseService("set_DOF_property", 
        &Robot::set_DOF_property, this);

    /* Subscriptions */
    ros::NodeHandle joint_target_nh("~");
    joint_target_nh.setCallbackQueue(&joint_target_callback_queue);
    targets_sub = joint_target_nh.subscribe("joint_targets", 1, &Robot::targets_cb, this);
    joint_target_spinner.start();

    initialized = true;
}

void Robot::pub_state(const ros::TimerEvent &event) {
    JointState js;
    sensor_msgs::JointState rjs;
    
    for(unsigned int i=0; i<DOFs.size(); i++) {
        DOF *d = DOFs[i];
        js.indices.push_back(i);
        js.enabled.push_back(d->get_enabled());
        js.positions.push_back(d->get_position());
        js.targets.push_back(d->get_cmd_position());

    }
    joint_state_pub.publish(js);

    rjs.header.stamp = ros::Time::now();
    rjs.header.frame_id = "/wrist";
    
    for(unsigned int i=0; i<actuator_groups.size(); i++)
        actuator_groups[i]->get_ros_jointstate(rjs);
    
    ros_joint_state_pub.publish(rjs);
}

void Robot::pub_dof_state(const ros::TimerEvent &event) {
    std::vector<DOFProperty> properties;
    for(unsigned int i=0; i<DOFs.size(); i++) {
        DOF *d = DOFs[i];
        d->get_properties(properties);
    }

    for(unsigned int i=0; i<properties.size(); i++) {
        dof_state_pub.publish(properties[i]);
    }
}

void Robot::register_callback(loop_callback fn, void *arg) {
    loop_callback_entry e;
    e.func = fn;
    e.arg = arg;
    callbacks.push_back(e);
}

bool Robot::set_DOF_property(SetDOFProperty::Request &req, SetDOFProperty::Response &resp) {
    unsigned int i = req.p.DOF_index;
    if(i >= DOFs.size()) {
        ROS_WARN("Ignoring request to set property for invalid DOF index %d", i);
        return false;
    }
    dof_state_pub.publish(req.p);
    return DOFs[i]->set_property(req.p);
}

bool Robot::get_arm_info(GetArmInfo::Request &req, GetArmInfo::Response &resp) {
    resp.name = robot_name;
    for(unsigned int i=0; i<DOFs.size(); i++) {
        DOF *d = DOFs[i];
        DOFInfo info;
        d->get_DOFInfo(info);
        resp.DOFs.push_back(info);
    }
    return true;
}

void Robot::targets_cb(const JointTargetsConstPtr &msg) {
    static ros::Duration offset;

    ros::Time timestamp;
    ros::Time now = ros::Time::now();
    
    if(use_client_timestamps) {
        ros::Duration new_offset = now - msg->header.stamp;
        ros::Duration delta = new_offset - offset;
        if( fabs(delta.toSec()) > 0.1 ) {
            offset = new_offset;
            ROS_DEBUG("New time offset %f", new_offset.toSec());
        } else {
            offset = offset * 0.8 + new_offset * 0.2;
        }

        timestamp = msg->header.stamp + offset;
    } else {
        timestamp = now;
    }

    if(msg->indices.size() != msg->targets.size()) {
        ROS_WARN("Invalid joint targets message; lengths of indices and targets differ");
        return;
    }

    for(unsigned int n=0; n<msg->indices.size(); n++) {
        unsigned int i = msg->indices[n];
        if(i >= DOFs.size()) {
            ROS_WARN("Ignoring target for invalid DOF index %d", i);
            continue;
        }

        StampedTarget t;
        t.index = i;
        t.timestamp = timestamp;
        t.target = msg->targets[n];
        pthread_mutex_lock(&target_queue_lock);
        target_queue.push(t);
        pthread_mutex_unlock(&target_queue_lock);
    }
}

void Robot::process_pending_targets() {
    pthread_mutex_lock(&target_queue_lock);
    while(!target_queue.empty()) {
        StampedTarget &t = target_queue.front();
        //std::cout << t.index << " " << t.target << " " << DOFs.size() << std::endl;
        DOFs[t.index]->set_cmd_position(t.target);
        target_queue.pop();
    }
    pthread_mutex_unlock(&target_queue_lock);
}

void Robot::step() {
    for(unsigned int i=0; i<actuator_groups.size(); i++) {
        actuator_groups[i]->control();
    }
    for(unsigned int i=0; i<callbacks.size(); i++) {
        callbacks[i].func(callbacks[i].arg);
    }
}


void Robot::driver_loop() {
    ros::Rate loop_rate(50);
    while(ros::ok()) {
        process_pending_targets();
        step();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

}

