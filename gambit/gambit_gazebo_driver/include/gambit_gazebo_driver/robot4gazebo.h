#ifndef ROBOT4GAZEBO_H
#define ROBOT4GAZEBO_H

#include <vector>
#include <gambit_driver/robot.h>
#include <gambit_driver/stamped_target.h>

namespace gambit_driver {

class Robot4Gazebo : public gambit_driver::Robot {

public:
    Robot4Gazebo(std::string name) : Robot(name) {}

    virtual ~Robot4Gazebo() {}

    void enable_dofs() {
        for(unsigned int i=0; i<DOFs.size(); i++) {
            DOF *d = DOFs[i];
            d->set_enabled(true);
        }
    }

//    void step() {
//        for(unsigned int i=0; i<DOFs.size(); i++) {
//            DOF *d = DOFs[i];
//            d->set_enabled(true);
//        }
//        std::cout << "step here1" << std::endl;
//        pthread_mutex_lock(&target_queue_lock);
//        while(!target_queue.empty()) {
//            StampedTarget &t = target_queue.front();
//            std::cout << t.index << " " << t.target << std::endl;
//            DOFs[t.index]->set_cmd_position(t.target);
//            target_queue.pop();
//        }
//        pthread_mutex_unlock(&target_queue_lock);
//        std::cout << "step here3" << std::endl;
//        for(unsigned int i=0; i<actuator_groups.size(); i++) {
//            actuator_groups[i]->control();
//        }
//        std::cout << "step here3" << std::endl;
//        for(unsigned int i=0; i<callbacks.size(); i++) {
//            callbacks[i].func(callbacks[i].arg);
//        }
//        //std::cout << "step here4" << std::endl;
//        ros::spinOnce();
//        //std::cout << "step here5" << std::endl;
//    }
};

}
#endif // ROBOT4GAZEBO_H
