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
 * An implementation of Actuator for Copley AccelNet drives.
 */

#include <ros/ros.h>
#include <gambit_driver/actuator_copley.h>
#include <sys/errno.h>

namespace gambit_driver {

ActuatorCopley::ActuatorCopley(unsigned int node_id, unsigned int counts_per_rev) {
    id = node_id;
    position = 0.0;
    torque_constant = 2.9;
    trajectory_active = true;
    parameters_updated = false;
    counts_per_rad = counts_per_rev / (M_PI * 2.0);
    int status = copleyAmpInit(node_id);
    if(status != ERROR_NONE) {
        ROS_FATAL("Failed to initialize amplifier id 0x%X: %s", node_id,
            copleyGetErrorString(status));
        if(status == ERROR_TIMEOUT) {
            ROS_FATAL("System error: %s", sys_errlist[errno]);
        }
        throw "Amplifier initialization failed";
    }

    status = copleyGetCurrentLimits(id, &peak_current_limit, &peak_current_time,
        &continuous_current_limit);
    if(status != ERROR_NONE) {
        ROS_ERROR("Failed to read current limits for amplifer id 0x%X: %s", node_id,
            copleyGetErrorString(status));
    }
}

ActuatorCopley::~ActuatorCopley() {
    copleyQuickStop(id);
}

double ActuatorCopley::get_position() {
    return position;
}

double ActuatorCopley::get_velocity() {
    return 0.0;
}

double ActuatorCopley::get_torque() {
    return 0.0;
}

bool ActuatorCopley::ping() {
    return true;
}

void ActuatorCopley::set_torque_enabled(bool enabled) {
    // We don't do anything here; the control loop will decide whether torque should
    // be enabled.
}

bool ActuatorCopley::is_homed() {
    bool is_homed = false;
    copleyCheckIfHomed(id, &is_homed);
    return is_homed;
}

void ActuatorCopley::home(bool direction) {
    copleyHomeByHomeSwitch(id, direction, counts_per_rad * 2 * M_PI + 100000);
}

void ActuatorCopley::sync() {
    copleyDoSync(id);
}

void ActuatorCopley::stop_interpolated_move() {
    copleyQuickStop(id);
    trajectory_active = false;
}

int ActuatorCopley::start_interpolated_move(double rate) {
    unsigned int interval = round(1.0/rate * 1000);
    ROS_DEBUG("Starting trajectory at %dms interval for amplifier id 0x%X", interval, id);
    int status = copleyStartInterpPosMove(id, 2, interval);
    if(status != ERROR_NONE) {
        ROS_WARN("Error  when starting interpolated move for amplifier id 0x%X: %s", status,
            copleyGetErrorString(status));
    } else {
        trajectory_active = true;
    }
    return status;
}

int ActuatorCopley::set_interpolated_target(int target) {
    int status = copleySetInterpTargPos(id, target);
    if(status != ERROR_NONE) {
        ROS_WARN("Error when setting interpolation target for amplifier id 0x%X: %s", 
            id, copleyGetErrorString(status));
    }
    return status;
}

void ActuatorCopley::read_status() {
    int32_t encoder_pos;
    int status = copleyGetCurPos(id, &encoder_pos);
    if(status != ERROR_NONE) {
        ROS_WARN("Error reading status from amplifier id 0x%X: %s", id,
            copleyGetErrorString(status));
    } else {
        position = (double)encoder_pos / counts_per_rad;
    }
}

void ActuatorCopley::get_properties(std::vector<DOFProperty> &properties) {
    unsigned int idx = DOFs[0]->get_index();

    DOFProperty p_torque_limit;
    p_torque_limit.DOF_index = idx;
    p_torque_limit.property_name="TORQUE_LIMIT";
    p_torque_limit.display_name="Torque";
    p_torque_limit.type="float";
    p_torque_limit.rw="rw";
    p_torque_limit.float_value = ((double)continuous_current_limit / 100.0) * 
        torque_constant;
    properties.push_back(p_torque_limit);
    
    DOFProperty p_peak_torque;
    p_peak_torque.DOF_index = idx;
    p_peak_torque.property_name="PEAK_TORQUE";
    p_peak_torque.display_name="Pk. Torque";
    p_peak_torque.type="float";
    p_peak_torque.rw="rw";
    p_peak_torque.float_value = ((double)peak_current_limit / 100.0) * 
        torque_constant;
    properties.push_back(p_peak_torque);
}

bool ActuatorCopley::set_property(DOFProperty &p) {
    if(p.property_name == "TORQUE_LIMIT") {
        int32_t current = ((int)round((p.float_value / torque_constant) * 100.0));
        if(current < 0) current = 0;
        if(current > 32767) current = 32767;
        continuous_current_limit = current;
        parameters_updated = true;
        return true;
    } else if(p.property_name == "PEAK_TORQUE") {
        int32_t current = ((int)round((p.float_value / torque_constant) * 100.0));
        if(current < 0) current = 0;
        if(current > 32767) current = 32767;
        peak_current_limit = current;
        parameters_updated = true;
        return true;
    }
    return false;
}

void ActuatorCopley::send_updated_parameters() {
    if(parameters_updated) {
        int status = copleySetCurrentLimits(id, peak_current_limit, peak_current_time,
            continuous_current_limit);
        if(status != ERROR_NONE) {
            ROS_ERROR("Error when setting torque limits for amplifier 0x%X: %s", id,
                copleyGetErrorString(status));
        }
        parameters_updated = false;
    }
}

void ActuatorCopley::load_calibration_parameters() {
    double value;
    ros::NodeHandle nh("~");
    if(nh.getParam(primary_name + std::string("_peak_torque"), value)) {
        int32_t current = ((int)round((value / torque_constant) * 100.0));
        if(current < 0) current = 0;
        if(current > 32767) current = 32767;
        peak_current_limit = current;
        parameters_updated = true;
        ROS_DEBUG("Setting peak torque limit for %s to %f", primary_name.c_str(), value);
    }

    if(nh.getParam(primary_name + std::string("_continuous_torque"), value)) {
        int32_t current = ((int)round((value / torque_constant) * 100.0));
        if(current < 0) current = 0;
        if(current > 32767) current = 32767;
        continuous_current_limit = current;
        parameters_updated = true;
        ROS_DEBUG("Setting continuous torque limit for %s to %f", primary_name.c_str(), 
            value);
    }
}

}


