class TableEvent(object):
    def __init__(self, type, time, obj, occfrac=None, occmask=None):
        self.eventType = type
        self.timestep = time
        self.object = obj
        
        if occfrac != None:
            self.occFrac = occfrac
        if occmask != None:
            self.occMask = occmask

    
    def __str__(self):
        return 'E(%i %s %s)' % (self.timestep, self.eventType, self.object.name)


TOUCH_EVENT = 'touch'
UNTOUCH_EVENT = 'untouch'
OCCLUDE_EVENT = 'occlude'
UNOCCLUDE_EVENT = 'unocclude'
CREATE_EVENT = 'create'
TAKEAWAY_EVENT = 'takeaway'
HAND_IN_EVENT = 'hand in'
HAND_OUT_EVENT = 'hand out'
