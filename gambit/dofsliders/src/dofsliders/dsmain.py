#!/usr/bin/env python

import roslib; roslib.load_manifest('dofsliders')
import rospy

import gtk
import sys

from dof import *
from gambit_msgs.srv import *
from gambit_msgs.msg import *

ARM_INFO_SERVICE = "get_arm_info"
SET_DOF_PROPERTY_SERVICE = "set_DOF_property"
ARM_STATE_TOPIC = "joint_state"
DOF_STATE_TOPIC = "dof_state"
JOINT_TARGET_TOPIC = "joint_targets"

class DOFColumn:
    
    def __init__(self, name, id, colid):
        self.name = name
        self.id = id
        self.colid = colid
        self.name_label = gtk.Label("<b>%s</b>" % self.name)
        self.name_label.set_use_markup(True)

class DOFSlidersMain:

    def __init__(self):
        self.ui = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.ui.connect("destroy", lambda event: gtk.main_quit())
        self.columns = []
        self.dofs = []
        self.cur_colid = 0
        self.table = gtk.Table(1,1)
        self.ui.add(self.table)
        self.add_default_columns()

        rospy.init_node('dofsliders')
        
        rospy.wait_for_service(ARM_INFO_SERVICE)
        rospy.wait_for_service(SET_DOF_PROPERTY_SERVICE)
        self.ros_get_arm_info = rospy.ServiceProxy(ARM_INFO_SERVICE, 
            GetArmInfo)
        self.set_dof_property = rospy.ServiceProxy(SET_DOF_PROPERTY_SERVICE, 
            SetDOFProperty)
        rospy.Subscriber(ARM_STATE_TOPIC, JointState, self.state_cb)
        rospy.Subscriber(DOF_STATE_TOPIC, DOFProperty, self.dof_state_cb)
        self.target_pub = rospy.Publisher(JOINT_TARGET_TOPIC, JointTargets)

        self.setup_dofs()
        self.ui.show_all()

    def state_cb(self, data):
        gtk.gdk.threads_enter()
        for n in range(len(data.indices)):
            i = data.indices[n]
            if i >= len(self.dofs):
                continue
            d = self.dofs[i]
            p = d.get_parameter("DOF_POSITION")
            p.get_control().set_value_nice(data.positions[n]* 180. / 3.14)
            en = d.get_parameter("DOF_ENABLED")
            #en.get_control().set_active(data.enabled[n])
        gtk.gdk.threads_leave()

    def dof_state_cb(self, data):
        gtk.gdk.threads_enter()
        if data.DOF_index > len(self.dofs):
            return
        d = self.dofs[data.DOF_index]
        p = d.get_parameter(data.property_name)
        if p is None:
            return
        if data.type == "bool" or data.type == "int":
            p.set_value(data.int_value)
        elif data.type == "float":
            p.set_value(data.float_value)
        elif data.type == "string":
            p.set_value(data.string_value)
        gtk.gdk.threads_leave()


    def add_default_columns(self):
        self.add_column("Name", "DOF_NAME")
        self.add_column("", "DOF_MIN_ANGLE")
        self.add_column("Position", "DOF_POSITION")
        self.add_column("", "DOF_MAX_ANGLE")
        self.add_column("Enable", "DOF_ENABLED")

    def parameter_changed(self, dofindex, id, type, value):
        print (dofindex, id, type, value)
        if id == "DOF_POSITION":
            targs = JointTargets()
            targs.indices.append(dofindex)
            targs.header.stamp = rospy.rostime.get_rostime()
            targs.targets.append(value * 3.14159 / 180.)
            self.target_pub.publish(targs)
        else:
            p = DOFProperty()
            p.DOF_index = dofindex
            p.property_name = id
            p.type = type
            if type == "bool":
                if value:
                    p.int_value = 1
                else:
                    p.int_value = 0
            elif type == "int":
                p.int_value = value
            elif type == "float":
                p.float_value = value
            elif type == "string":
                p.string_value = value
            self.set_dof_property(p)
            

    def setup_dofs(self):
        info = self.ros_get_arm_info()
        self.ui.set_title("Robot DOF Sliders: %s" % info.name)

        index = 0
        for dofinfo in info.DOFs:
            d = DOF(index, dofinfo.display_name, dofinfo.limit_min, dofinfo.limit_max,
                self.parameter_changed)
            d.get_parameter("DOF_POSITION").get_control().set_size_request(400,0)
            self.dofs.append(d)
            for p in dofinfo.properties:
                if d.get_parameter(p.property_name) is None:
                    if p.rw == "r":
                        if p.type == "int":
                            d.add_parameter(p.property_name, p.display_name, p.int_value,
                                p.type, "readonly")
                        elif p.type == "float":
                            d.add_parameter(p.property_name, p.display_name, p.float_value,
                                p.type, "readonly")
                    elif p.type == "bool":
                        d.add_parameter(p.property_name, p.display_name, p.int_value, 
                            p.type, "checkbox")
                    elif p.type == "float":
                        d.add_parameter(p.property_name, p.display_name, p.float_value,
                            p.type, "entry")
                    elif p.type == "int":
                        d.add_parameter(p.property_name, p.display_name, p.int_value,
                            p.type, "entry")
                else:
                    param = d.get_parameter(p.property_name)
                    if p.type == "bool" or p.type == "int":
                        param.set_value(p.int_value)
                    elif p.type == "float":
                        param.set_value(p.float_value)
                    elif p.type == "string":
                        param.set_value(p.string_value)
                    
            index += 1
        
        self.table.resize(len(self.dofs)+1, len(self.columns))
        for i in xrange(len(self.dofs)):
            dof = self.dofs[i]
            for p in dof.parameters:
                colid = self.get_column_index(p.id, p.displayname)
                xoptions =0 
                if p.controltype == "slider":
                    xoptions = gtk.FILL | gtk.EXPAND;
                self.table.attach(p.get_control(), colid, colid+1, i+1, i+2, xpadding=4, 
                    ypadding=4, xoptions=xoptions)

    def get_column_index(self, id, name=""):
        for i in xrange(len(self.columns)):
            if(self.columns[i].id == id):
                return i
        col = self.add_column(name, id)
        print "dynamically adding column %s" % name
        return col.colid

    def add_column(self, name, id):
        col = DOFColumn(name, id, self.cur_colid)
        self.cur_colid += 1
        self.columns.append(col)
        self.table.resize(len(self.dofs)+1, len(self.columns))
        self.table.attach(col.name_label, col.colid, col.colid+1, 0, 1, xpadding=8,
            ypadding=4, xoptions=0)
        return col

if __name__ == "__main__":
    gtk.gdk.threads_init()
    gtk.gdk.threads_enter()
    app = DOFSlidersMain()
    gtk.main()
    gtk.gdk.threads_leave()
