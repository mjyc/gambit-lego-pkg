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
 * Top-level driver node for the Gambit arm.
 */

#include <ros/ros.h>

#include <gambit_driver/robot.h>
#include <gambit_driver/arm.h>
#include <gambit_driver/wrist.h>

#include <bus/interface_tty.h>
#include <safetyboard/safetyboard.h>

#include <string>
#include <termios.h>

#ifdef REALTIME
#include <sys/mman.h>
#endif

#include <copley/copleyCmds.h>
#include <copley/copleySockCanSdo.h>
#include <copley/copleyDictionary.h>

void check_safety_cb(void *arg) {
    SafetyBoard *safety = (SafetyBoard*)arg;
    int state = safety->get_state();
    if(state == STATE_ESTOPPED) {
        int fault = safety->get_fault();
        ROS_FATAL("Emergency stop detected");
        if(fault == FAULT_ESTOP) {
            ROS_FATAL("E-stop button pressed");
        } else if(fault == FAULT_HEARTBEAT) {
            ROS_FATAL("Heartbeat fault detected");
        }
        ros::shutdown();
        exit(1);
    }
}

int main(int argc, char **argv) {
#ifdef REALTIME
    mlockall(MCL_CURRENT|MCL_FUTURE);
#endif

    ROSCONSOLE_AUTOINIT;

    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::init(argc, argv, "arm");

    gambit_driver::Robot *robot = new gambit_driver::Robot("arm");

    ros::NodeHandle nh("~");

    /* RS485 BUS SETUP */
    std::string rs485_interface("/dev/tty81a");
    nh.getParam("rs485_interface", rs485_interface);

    bus::InterfaceTTY *rs485;
    try {
        rs485 = new bus::InterfaceTTY(rs485_interface.c_str(), B115200);
    } catch(int e) {
        ROS_FATAL("Failed to open RS-485 interface");
        return 1;
    }

    /* ENABLE SAFETY RELAY */
    SafetyBoard *safety = new SafetyBoard(rs485, 10);
    ROS_DEBUG("Enabling motor bus power");

    unsigned int n_retries = 0;
    while(true) {
        safety->set_state(STATE_ENABLED);
        
        /* WAIT FOR POWER TO RISE */
        usleep(500000); 
        
        /* CONSUME RS-485 BUS NOISE */
        uint8_t buf[255];
        int n = rs485->read(buf, 255);
        ROS_DEBUG("Consumed %d extraneous bytes on RS-485 interface", n);
        
        if(safety->get_state() == STATE_ENABLED) {
            break;
        } else if(n_retries++ > 10) {
            ROS_FATAL("Failed to enable motor bus voltage; check E-stops");
            exit(1);
        }
    }

    
    /* CANBUS SETUP */
    std::string can_interface("can0");
    nh.getParam("can_interface", can_interface);

    ROS_DEBUG("Initializing SocketCAN interface %s", can_interface.c_str());
    int status = socketCanInit(can_interface.c_str(), 999999);
    if(status != ERROR_NONE) {
        ROS_FATAL("Failed to initialize SocketCAN interface %s: %s", 
            can_interface.c_str(), copleyGetErrorString(status));
        return 1;
    }

    /* SET UP ROBOT */
    gambit_driver::Arm *arm = new gambit_driver::Arm;
    gambit_driver::Wrist *wrist = new gambit_driver::Wrist(rs485);

    robot->add_actuator_group(arm);
    robot->add_actuator_group(wrist);

    robot->initialize();

    robot->register_callback(check_safety_cb, (void*)safety);
   
    ROS_DEBUG("Starting main loop");
    safety->set_state(STATE_ACTIVE);
    robot->driver_loop();
    
    ROS_DEBUG("Disabling motor bus power");
    safety->set_state(STATE_RESET);

    ROS_DEBUG("Exiting");
    return 0;
}

