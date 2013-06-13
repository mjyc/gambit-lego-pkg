#ifndef STATE_H
#define STATE_H

#include <iostream>

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

using namespace std;

// I like this state machine since it requires less places to modify than that of head first

class StateMachine {
public:
    StateMachine();
    void moveToBack();
    void moveToFront();

private:
    friend class State; //define friendship
    class State* state_; //
    void setState (State* newState);
};


class State : boost::noncopyable {
public:
    virtual void moveToBack(StateMachine&) { cout << "default moveToBack()" << endl; }
    virtual void moveToFront(StateMachine&) { cout << "default moveToBack()" << endl; }
protected:
    void setState (StateMachine& machine, State* newState) { machine.setState(newState); }
};


class BackState : public GambitManipState {
public:
    static boost::shared_ptr<BackState> getInstance();
    ~BackState() {/*weak ptr does not have to be nulled*/}

    virtual void moveToBack(StateMachine);

private:
    BackState() { }

    static boost::weak_ptr<BackState> singleton_;
};

boost::weak_ptr<BackState> BackState::singleton_;

boost::shared_ptr<BackState> BackState::getInstance() {
    boost::shared_ptr<BackState> instance = singleton_.lock();
    if (!instance) {
        instance.reset(new BackState());
        singleton_ = instance;
    }
    return instance;
}


BasicManipStateMachine::BasicManipStateMachine () :
    state_(BackState::getInstance().get()) { }

void BasicManipStateMachine::moveToBack() {
    state_->moveToBack(*this);
}
void BasicManipStateMachine::moveToFront() {
    state_->moveToFront(*this);
}
void BasicManipStateMachine::moveToManip() {
    state_->moveToManip(*this);
}
void BasicManipStateMachine::setState (GambitManipState* newState) {
    state_ = newState;
}

#endif // STATE_H
