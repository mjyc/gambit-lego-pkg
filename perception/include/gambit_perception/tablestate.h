#ifndef TABLESTATE_H
#define TABLESTATE_H

#include <vector>
#include <map>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <gambit_perception/tableevent.h>
#include <gambit_perception/tableobject.h>


// TODOs:
// 1. Currently I'm using LegoTableObject everywhere, make interface or somehow organize it
// 2. Really make sure what kind of assumptions I'm making (1 block at a time, etc...)
//    make "update" part of code very clear.
// 3. Organize what I'm publishing


// Global variables
static const int MAX_NUM_OBJ = 10;


// Classes
class TableState {

protected:
    TableParams params_;
    image_geometry::PinholeCameraModel* model_; // dont' modify
    int max_num_obj_;

    // objects on the table
    std::map<std::string, LegoTableObject::Ptr> objects_; // TODO: back to TableObject or something

    // table mask property variables
    cv::Mat baseTable_;
    cv::Mat tableModel_;
    cv::Mat mask_;

    // can be used for multiple constuctors
    void init(cv::Mat baseTable, cv::Mat tableModel, cv::Mat mask, \
              TableParams params, \
              image_geometry::PinholeCameraModel* model,\
              int max_num_obj = MAX_NUM_OBJ) {
        params_ = params;
        model_ = model;

        baseTable_ = baseTable.clone();
        tableModel_ = tableModel.clone();
        mask_ = mask.clone();

        max_num_obj_ = max_num_obj;
    }


public:
    typedef boost::shared_ptr<TableState> Ptr;
    typedef std::pair<std::string, LegoTableObject::Ptr> str_tobj_p;

    // Constructors
    TableState(cv::Mat baseTable, cv::Mat tableModel, cv::Mat mask, \
               TableParams params, \
               image_geometry::PinholeCameraModel* model, \
               int max_num_obj) {
        init(baseTable, tableModel, mask, params, model, max_num_obj);
    }

    virtual ~TableState() {}

    virtual void clear() {
        objects_.clear();
        tableModel_ = baseTable_.clone();
        NUM_OBJECTS = 0;
    }

    virtual std::vector<TableEvent> update(cv::Mat& img, cv::Mat& dep) {
        std::vector<TableEvent> events;

        // Detect removed objects
        BOOST_FOREACH(str_tobj_p p, objects_) {
            LegoTableObject::Ptr objPtr = p.second;
            if (objPtr->getOffTable()) // skip objects that are offtable
                continue;

            if (objPtr->isRemoved(dep)) {
                // no need to update TableObject image
                objPtr->setOffTable(true);                                   // update TableObject offtable
                cutTableModel(baseTable_,objPtr->getROI(),tableModel_);      // update TableModel
                events.push_back(TableEvent(TableEvent::REMOVE, objPtr));    // add an event

                //std::cout << objPtr->getName() + " is removed" << std::endl;
            }
        }

        // Find objects
        std::vector< std::vector<cv::Point> > objContours = \
                findObjectContours(dep,tableModel_,mask_, \
                                   params_.table_noise_threshold, \
                                   params_.min_objsize_frac);

        // BIG ASSUMPTION - ONLY ONE OBJECT APPEARS AT ONE TIME!
        if (objContours.size() > 1)
            std::cout << "[WARN] More than one object appeared at once." << std::endl;

        // Check for reappeared objects
        // BIG ASSUMPTION - ONLY ONE OBJECT OFF-THE-TABLE AT ONE TIME!
        BOOST_FOREACH(str_tobj_p p, objects_) {
            LegoTableObject::Ptr objPtr = p.second;

            if (objPtr->getOffTable() && objContours.size() > 0) {
                std::vector<cv::Point> objContour = objContours[0];

                objPtr->updateImages(img,dep,objContour);              // update TableObject image
                objPtr->setLocState(LegoTableObject::ONTABLE);             // update TableObject locState
                augmentTableModel(dep,objPtr->getROI(),tableModel_);   // update tableModel
                events.push_back(TableEvent(TableEvent::REAPPEAR, objPtr)); // add an event
                //std::cout << objPtr->getName() + "reappeared" << std::endl;

                // clear detected object list
                // * prevent adding "reappearaed" objs to "new" obj list
                // * a.k.a. prevent double counting
                objContours.clear();
            }
        }

        // Create NEW objects
        BOOST_FOREACH(std::vector<cv::Point> newObjContour, objContours) {
            if (NUM_OBJECTS < max_num_obj_) {

                // Create new obj
//                TableObject::Ptr newObj =  boost::make_shared<TableObject>( \
//                            TableObject::getNextObjectName(), \
//                            TableObject::ONTABLE,img,dep,newObjContour, \
//                            params_, model_);
                LegoTableObject::Ptr newObj =  boost::make_shared<LegoTableObject>( \
                            LegoTableObject::getNextObjectName(), \
                            LegoTableObject::ONTABLE,img,dep,newObjContour, \
                            params_, model_);

                objects_[newObj->getName()] = newObj;                      // add new TableObject
                augmentTableModel(dep,newObj->getROI(),tableModel_);       // update tableModel
                events.push_back(TableEvent(TableEvent::CREATE, newObj));  // add an event
            }
        }


        // Update complex (2-objects) events
        BOOST_FOREACH(TableEvent event, events) {
            BOOST_FOREACH(str_tobj_p p, objects_) {
                LegoTableObject::Ptr objPtr = p.second;

                if (event.getObj()->getName().compare(objPtr->getName()) != 0)
                    event.getObj()->getRelativeProperty(objPtr);

            }
        }

        return events;
    }


    // Getter and Setters

    std::map<std::string, LegoTableObject::Ptr> getTableObjects() {
        return objects_;
    }

    std::vector<LegoTableObject::Ptr> getTableObjectVector() {
        std::vector<LegoTableObject::Ptr> objVector;
        BOOST_FOREACH(str_tobj_p p, objects_)
            objVector.push_back(p.second);
        return objVector;
    }
};

#endif // TABLESTATE_H




