#!/usr/bin/env python
import roslib; roslib.load_manifest('armsim')
import rospy
from gambit_msgs.msg import *
from gambit_msgs.srv import *

## [2012.04.04 Mike Chung]
## To enable "simulated actuataors", need to call set_DOF_properties.
## from "armsim/src/arm_sim_node.cpp", it DOES NOT enable the actuators from the start!

def set_DOF_property_client(p):
    rospy.wait_for_service('set_DOF_property');
    try:
        set_DOF_property = rospy.ServiceProxy('set_DOF_property', SetDOFProperty)
        resp1 = set_DOF_property(p)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":

    GambitDOFlist = [0,1,2,3,4,5,6]
    for i in GambitDOFlist:
        p = DOFProperty()
        p.DOF_index = i
        p.property_name = 'DOF_ENABLED'
        p.display_name = 'Enabled'
        p.type = 'bool'
        p.rw = 'rw'
        p.string_value = ''
        p.int_value = 1
        p.float_value = 0.0

        print 'sending:'
        print p
        set_DOF_property_client(p);
    
