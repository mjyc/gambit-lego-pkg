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
 * High-level definiton of the Gambit 3-DOF arm.
 */

#include <gambit_driver/arm.h>
#include <math.h>
#include <limits.h>
#include <sys/time.h>

namespace gambit_driver {

Arm::Arm() {
    control_started = false;
    control_thread_rate = 100.0;
    last_sync_time = 0;

    ros::NodeHandle nh("~");

    int lag_time = 150;
    nh.getParam("lag_time", lag_time);

    /* AMPLIFIER INITIALIZATION */
    ActuatorCopley *m1;
    ActuatorCopley *m2;
    ActuatorCopley *m3;
    try {
        ROS_DEBUG("Initializing amplifier 1");
        m1 = new ActuatorCopley(0x01, 800000);
        ROS_DEBUG("Initializing amplifier 2");
        m2 = new ActuatorCopley(0x02, 800000);
        ROS_DEBUG("Initializing amplifier 3");
        m3 = new ActuatorCopley(0x03, 400000);
    } catch(const char *e) {
        ROS_FATAL("Arm initialization failed.");
        ROS_FATAL("Check that all arm DOFs are present and powered on.");
        exit(1);
    }

    /* TORQUE CONSTANTS: Nm/A */
    m1->set_torque_constant(2.9);
    m2->set_torque_constant(2.9);
    m3->set_torque_constant(1.3);

    /* DOF/SMOOTHER SETUP */
    SmoothedDOF *j1 = new SmoothedDOF("waist", lag_time);
    j1->set_limits(-M_PI, M_PI);
    j1->add_actuator((Actuator*)m1, 1.0);
    j1->set_continuous_rotation(true);
    m1->add_DOF((DOF*)j1, 1.0);

    SmoothedDOF *j2 = new SmoothedDOF("shoulder", lag_time);
    j2->set_limits(-M_PI, M_PI);
    j2->add_actuator((Actuator*)m2, 1.0);
    j2->set_continuous_rotation(true);
    m2->add_DOF((DOF*)j2, 1.0);
    
    SmoothedDOF *j3 = new SmoothedDOF("elbow", lag_time);
    j3->set_limits(-M_PI, M_PI);
    j3->add_actuator((Actuator*)m3, 1.0);
    j3->set_continuous_rotation(true);
    m3->add_DOF((DOF*)j3, 1.0);

#ifdef REALTIME
    j1->set_realtime(true);
    j2->set_realtime(true);
    j3->set_realtime(true);
#endif

    actuators.push_back(m1);
    DOFs.push_back(j1);
    actuators.push_back(m2);
    DOFs.push_back(j2);
    actuators.push_back(m3);
    DOFs.push_back(j3);

    load_calibration_parameters();

    ROS_DEBUG("Arm base configured with %d actuators in %d DOFs", actuators.size(),
        DOFs.size());
}

void Arm::initialize() {
    ActuatorCopley *j1 = (ActuatorCopley*)actuators[0];
    ActuatorCopley *j2 = (ActuatorCopley*)actuators[1];
    ActuatorCopley *j3 = (ActuatorCopley*)actuators[2];

    if(j1->is_homed() && j2->is_homed() && j3->is_homed()) {
        ROS_DEBUG("All actuators already homed, no need to do so");
    } else {
        ROS_DEBUG("Homing actuator 3");
        j3->home(true);
        ROS_DEBUG("Homing actuator 2");
        j2->home(true);
        ROS_DEBUG("Homing actuator 1");
        j1->home(false);
    }
}

bool Arm::check_connectivity() {
    // If we've passed the initialization in the constructor, the arm must be connected
    // okay, so just return true
    return true;
}

void Arm::set_control_rate(double rate) {
    control_thread_rate = rate;
}

void Arm::control() {
    if(!control_started) {
        reset_timing_stats();
        timing_avg_latency = 0.0;
        timing_last_exec = get_time_usec();
        last_status_print_time = ros::Time::now();
#ifdef REALTIME
        ROS_DEBUG("Starting internal arm control thread (realtime)");
        rt_task_create(&rt_internal_thread, "arm_control", 0, 50, 0);
        rt_task_start(&rt_internal_thread, &Arm::internal_control, this);
#else
        ROS_DEBUG("Starting internal arm control thread");
        pthread_create(&internal_thread, NULL, &Arm::internal_control, this);
#endif
        control_started = true;
    }

    if((ros::Time::now() - last_status_print_time).toSec() > 10.0) {
        print_timing_stats();
        reset_timing_stats();
        last_status_print_time = ros::Time::now();
    }
}

void Arm::sync_drives() {
    //ROS_DEBUG("Syncing drives");
    copleySendSyncSignal();
    for(unsigned int i=0; i < actuators.size(); i++) {
        ActuatorCopley *a = (ActuatorCopley*)this->actuators[i];
        a->sync();
    }
}

/* control thread */
#ifdef REALTIME
void Arm::internal_control(void *arg) {
#else
void *Arm::internal_control(void *arg) {
#endif

    Arm *self = (Arm*)arg;
#ifdef REALTIME
    rt_task_set_periodic(NULL, TM_NOW, 1.0/self->control_thread_rate * 1000000000);
#else
    ros::Rate loop_rate(self->control_thread_rate);
#endif

    for(;;) {
        self->update_timing_stats();
        int status;

        uint64_t now = self->get_time_usec();
        if(now - self->last_sync_time > 1000000) {
            self->sync_drives();
            self->last_sync_time = now;
        }
        
        for(unsigned int i=0; i < self->actuators.size(); i++) {
            ActuatorCopley *a = (ActuatorCopley*)self->actuators[i];
            if(!a->get_trajectory_active() && a->get_DOF_enabled()) {
                // Associated DOF has been enabled; start a new trajectory
                a->reset();
                a->start_interpolated_move(self->control_thread_rate);
            } else if(a->get_trajectory_active() && !a->get_DOF_enabled()) {
                // A trajectory is running, but the DOF has been disabled.  Execute a
                // stop to the interpolated move.
                a->stop_interpolated_move();
            } else if(a->get_trajectory_active() && a->get_DOF_enabled()) {
                // A trajectory is active and the DOF is enabled; send a new target
                int target = a->get_cmd_position() * a->counts_per_rad;
                status = a->set_interpolated_target(target);
                if(status == ERROR_MOVE_NOT_ACTIVE) {
                    ROS_WARN("Restarting trajectory for arm actuator %d", i);
                    a->start_interpolated_move(self->control_thread_rate);
                }
            }

            // Read encoders.
            a->send_updated_parameters();
            a->read_status();
        }
        
#ifdef REALTIME
        rt_task_wait_period(NULL);
#else
        loop_rate.sleep();
#endif
    }

    ROS_ERROR("Internal control thread exiting!");
#ifndef REALTIME
    return NULL;
#endif
}

uint64_t Arm::get_time_usec() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

void Arm::reset_timing_stats() {
    timing_max_latency = INT_MIN;
    timing_min_latency = INT_MAX;
}

void Arm::print_timing_stats() {
    ROS_DEBUG("Arm control latency: avg: %.03f usec, min: %d usec, max %d usec", 
        timing_avg_latency, timing_min_latency, timing_max_latency);
}

void Arm::update_timing_stats() {
    uint64_t now = get_time_usec();
    unsigned int target_period = 1.0/control_thread_rate * 1000000;
    unsigned int actual_period = now - timing_last_exec;
    int latency = actual_period - target_period;

    if(latency > timing_max_latency) timing_max_latency = latency;
    if(latency < timing_min_latency) timing_min_latency = latency;
    timing_avg_latency = (timing_avg_latency * (control_thread_rate - 1) + latency) / 
        control_thread_rate;
    
    timing_last_exec = now;
}


}

