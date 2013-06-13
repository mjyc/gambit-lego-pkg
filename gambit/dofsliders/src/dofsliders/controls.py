import gtk
import gobject

class DOFEntry(gtk.Entry):

    __gsignals__ = {
        'editing_complete': (gobject.SIGNAL_RUN_LAST, gobject.TYPE_NONE, ()),
    }

    def __init__(self, format="%d", type="float"):
        gtk.Entry.__init__(self)
        self.value = 0
        self.type = type
        self.connect("changed", self.edited_cb)
        self.connect("activate", self.activate_cb)
        self.send_edit_events = True
        self.editing = False
        
    def get_value(self):
        return self.value

    def convert_value(self, value):
        try:
            if self.type == "float":
                return float(value) 
            elif self.type == "int":
                return int(value)
            else:
                return value
        except:
            print "invalid value for control"
            return self.value

    def set_value(self, value):
        if not self.editing:
            self.value = self.convert_value(value)
            self.send_edit_events = False
            self.set_text(self.format % self.value)
            self.send_edit_events = True

    def edited_cb(self, widget):
        if self.send_edit_events:
            self.editing = True
            self.modify_base(gtk.STATE_NORMAL, gtk.gdk.color_parse("#ffaaaa"))
    
    def activate_cb(self, widget):
        self.modify_base(gtk.STATE_NORMAL, gtk.gdk.color_parse("#ffffff"))
        self.editing = False
        self.set_value(self.get_text())
        self.emit("editing_complete")
        print "edc"

gobject.type_register(DOFEntry)

class DOFSlider(gtk.HScale):

    def __init__(self, adj):
        gtk.HScale.__init__(self, adj)
        self.dragging = False
        self.connect("button-press-event", self.mousedown_cb)
        self.connect("button-release-event", self.mouseup_cb)

    def mousedown_cb(self, widget, event):
        self.dragging = True

    def mouseup_cb(self, widget, signal_id):
        self.dragging = False

    def set_value_nice(self, value):
        if not self.dragging:
            self.set_value(value)
