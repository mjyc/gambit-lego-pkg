#ifndef TABLESTATE_H
#define TABLESTATE_H

#include <vector>
#include <map>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <gambit_perception/icra2013/tableobject.h>
#include <gambit_perception/tableevent_detection.h>
#include <gambit_perception/tableevent.h>


// Global variables
static const int MAX_NUM_OBJ = 4;


// Classes
class TableState {

protected:
    TableParams params_;
    int max_num_obj_;

    std::map<std::string, TableObject::Ptr> objects_;
    std::vector<TableEvent> events_; // events at current frame

    cv::Mat baseTable_;
    cv::Mat tableModel_;
    cv::Mat mask_;
    std::vector< std::vector<cv::Point> > areaContours_; // used for which ot two area the object is located at


    // can be used for multiple constuctors
    void init(cv::Mat baseTable, cv::Mat tableModel, cv::Mat mask, \
              std::vector< std::vector<cv::Point> > areaContours, \
              TableParams params, \
              int max_num_obj = MAX_NUM_OBJ) {
        params_ = params;

        baseTable_ = baseTable.clone();
        tableModel_ = tableModel.clone();
        mask_ = mask.clone();
        areaContours_ = areaContours;

        max_num_obj_ = max_num_obj;
    }


public:
    typedef boost::shared_ptr<TableState> Ptr;
    typedef std::pair<std::string, TableObject::Ptr> str_tobj_p;

    // Constructors
    TableState(cv::Mat baseTable, cv::Mat tableModel, cv::Mat mask, \
               std::vector< std::vector<cv::Point> > areaContours, \
               TableParams params, \
               int max_num_obj) {
        init(baseTable, tableModel, mask, areaContours, params, max_num_obj);
    }

    virtual ~TableState() {}


    virtual void update(cv::Mat& img, cv::Mat& dep) {
        events_.clear(); // empty out previous events


        // Detect removed objects
        BOOST_FOREACH(str_tobj_p p, objects_) {
            TableObject::Ptr objPtr = p.second;
            if (objPtr->getOffTable()) // skip objects that are offtable
                continue;

            if (objPtr->isRemoved(dep)) {
                // no need to update TableObject image
                objPtr->setOffTable(true);                                   // update TableObject offtable
                cutTableModel(baseTable_,objPtr->getROI(),tableModel_);      // update TableModel
                events_.push_back(TableEvent(TableEvent::REMOVE, objPtr));   // add an event

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
            TableObject::Ptr objPtr = p.second;

            if (objPtr->getOffTable() && objContours.size() > 0) {
                std::vector<cv::Point> objContour = objContours[0];

                // Check for location
                int locID = objContour2AreaIdx(objContour);
                TableObject::loc_state_t locState = TableState::areaIdx2LocState(locID);

                objPtr->updateImages(img,dep,objContour);              // update TableObject image
                objPtr->setLocState(locState);                         // update TableObject locState
                augmentTableModel(dep,objPtr->getROI(),tableModel_);   // update tableModel
                events_.push_back(TableEvent(TableEvent::REAPPEAR, objPtr)); // add an event
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

                // Check for location
                int areaIdx = objContour2AreaIdx(newObjContour);
                TableObject::loc_state_t locState = TableState::areaIdx2LocState(areaIdx);

                // Create new obj
                TableObject::Ptr newObj =  boost::make_shared<TableObject>( \
                            TableObject::getNextObjectName(), \
                            locState,img,dep,newObjContour, \
                            params_);

                objects_[newObj->getName()] = newObj;                      // add new TableObject
                augmentTableModel(dep,newObj->getROI(),tableModel_);       // update tableModel
                events_.push_back(TableEvent(TableEvent::CREATE, newObj)); // add an event
            }
        }
    }


    virtual void clear() {
        objects_.clear();
        events_.clear();
        tableModel_ = baseTable_.clone();
        NUM_OBJECTS = 0;
    }

    // @params posPt: query 2D point
    // @return: area index (for areaCountour_), or -1 if the point belong
    //          unknown area.
    // NOTE: areaContours_ shouldn't be overlapping each other. If they are,
    //       return value will be the last area index the query point belong to.
    int posPt2AreaIdx(cv::Point2f posPt) {
        int retval = -1;
        for (int i = 0; i < areaContours_.size(); ++i) {
            if (cv::pointPolygonTest(areaContours_[i],posPt,false) > 0)
                retval = i;
        }
        return retval;
    }

    int objContour2AreaIdx(std::vector<cv::Point> objContour) {
        // Compute 2D centroid
        cv::Moments mu = cv::moments( objContour, false );
        cv::Point2f mc ( mu.m10/mu.m00, mu.m01/mu.m00 );
        return posPt2AreaIdx(mc);
    }


    // Getter and Setters

    std::map<std::string, TableObject::Ptr> getTableObjects() {
        return objects_;
    }

    std::vector<TableObject::Ptr> getTableObjectVector() {
        std::vector<TableObject::Ptr> objVector;
        BOOST_FOREACH(str_tobj_p p, objects_)
            objVector.push_back(p.second);
        return objVector;
    }

    std::vector<TableEvent> getEvents() {
        return events_;
    }


    // Static functions

    // Given locationID, returns TableObject::ObjLoc type
    static TableObject::loc_state_t areaIdx2LocState(int locID) {
        TableObject::loc_state_t retval = TableObject::UNKNOWN;
        switch (locID) {
        case 0:
            retval = TableObject::LEFT;
            break;
        case 1:
            retval = TableObject::RIGHT;
            break;
        }
        return retval;
    }
};

#endif // TABLESTATE_H
