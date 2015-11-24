#ifndef TABLEEVENT_H
#define TABLEEVENT_H

#include <gambit_perception/tableobject.h>

class TableEvent {
public: // Forward declaration required
    enum EvType
    {
        HANDIN,
        HANDOUT,
        CREATE,
        REMOVE,
        REAPPEAR
    };

private:
    EvType eventType_;
    LegoTableObject::Ptr curObj_;  // somehow fix back TableObject later

public:
    typedef boost::shared_ptr<TableEvent> Ptr;

    TableEvent(EvType eventType, LegoTableObject::Ptr curObj)
        : eventType_(eventType),
          curObj_(curObj)
    {}

    EvType getType() {
        return eventType_;
    }

    LegoTableObject::Ptr getObj() {
        return curObj_;
    }
};


//class LegoTableEvent {

//public:

//    LegoTableEvent(EvType eventType, LegoTableObject::Ptr curObj)
//        : TableEvent(eventType, curObj)
//    {}

//};


#endif // TABLEEVENT_H
