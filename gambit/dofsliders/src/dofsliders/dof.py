import gtk
from controls import DOFEntry, DOFSlider
from math import pi

class DOFParameter:
    
    def __init__(self, dof, id, displayname, value, type, controltype, lower=0.0, upper=1.0):
        self.id = id
        self.dof = dof
        self.value = value
        self.type = type
        self.control = None
        self.controltype = controltype
        self.displayname = displayname
        self.upper = upper
        self.lower = lower

        if controltype == "readonly":
            if type == "int":
                self.control = gtk.Label("%d" % value)
            elif type == "float":
                self.control = gtk.Label("%0.02f" % value)
            elif type == "string":
                self.control = gtk.Label("%s" % value)
            else:
                self.control = gtk.Label("???")
        elif controltype == "slider":
            step = (self.upper - self.lower) / 1024.0
            adj = gtk.Adjustment(value, self.lower, self.upper, step, step * 10, step * 10)
            self.control = DOFSlider(adj)
            self.control.set_draw_value(False)
            self.control.set_size_request(200,-1)
            self.control.connect("change-value", self.user_changed_cb)
            self.control.connect("move-slider", self.user_changed_cb)
        elif controltype == "checkbox":
            self.control = gtk.CheckButton()
            self.control.set_active(value)
            self.control.connect("clicked", self.user_changed_cb)
        elif controltype == "entry":
            self.control = DOFEntry(type=type)
            if type == "int":
                self.control.set_size_request(48, -1)
                self.control.set_alignment(1.0)
                self.control.format = "%d"
            elif type == "float":
                self.control.set_size_request(48, -1)
                self.control.set_alignment(1.0)
                self.control.format = "%0.02f"
            elif type == "string":
                self.control.format = "%s"
            else:
                self.control.format = "%s"
            self.control.connect("editing_complete", self.user_changed_cb)

            self.control.set_value(value)
        self.set_value(value)

    def get_control(self):
        return self.control
        
    def set_value(self, value):
        controltype = self.controltype
        type = self.type
        if controltype == "readonly":
            if type == "int":
                self.control.set_text("%d" % value)
            elif type == "float":
                self.control.set_text("%0.02f" % value)
            elif type == "string":
                self.control.set_text("%s" % value)
        elif controltype == "slider":
            self.control.set_value_nice(value)
        elif controltype == "checkbox":
            self.control.set_active(value)
        elif controltype == "entry":
            self.control.set_value(value)
        

    def user_changed_cb(self, widget, arg1=None, arg2=None, arg3=None):
        value = None
        if self.controltype == "checkbox":
            value = self.control.get_active()
        elif self.controltype == "slider":
            value = self.control.get_value()
        elif self.controltype == "entry":
            value = self.control.get_value()
        self.dof.param_changed(self.id, self.type, value)
        return False

class DOF:
    
    def __init__(self, index, name, min_limit, max_limit, callback):
        self.index = index
        self.callback = callback
        self.parameters = []
        
        min_limit = min_limit * 180.0 / pi
        max_limit = max_limit * 180.0 / pi

        self.add_parameter("DOF_NAME", "Name", name, "string", "readonly")
        self.add_parameter("DOF_MIN_ANGLE", "", round(min_limit), "int", "readonly")
        self.add_parameter("DOF_POSITION", "Position", 0, "float", "slider", min_limit, max_limit)
        self.add_parameter("DOF_MAX_ANGLE", "", round(max_limit), "int", "readonly")
        self.add_parameter("DOF_ENABLED", "Enable", False, "bool", "checkbox")

    def add_parameter(self, id, displayname, value, type, controltype, lower=0.0, upper=1.0):
        param = DOFParameter(self, id, displayname, value, type, controltype, lower, upper)
        self.parameters.append(param)

    def get_parameter(self, id):
        for p in self.parameters:
            if p.id == id:
                return p
        return None

    def param_changed(self, id, type, value):
        if self.callback is not None:
            self.callback(self.index, id, type, value);
