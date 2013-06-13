#ifndef SMOOTHER_H 
#define SMOOTHER_H 

#include <deque>
#include <assert.h>
#include <pthread.h>
#include <stdint.h>

namespace smoothing {

struct PendingTargPos
{
  volatile bool bPending;
  int64_t timeStamp;
  double targPos;
};

class Smoother
{
  public:
    enum SA_State { INVALID, VALID, PENDING_RESET };

    // TIGHT is guaranteed to go through all targets 
    // LOOSE is only guaranteed to go through the last target, but is smoother
    enum SA_SplineType { STRICT, LOOSE };
  
    //ctor, dtor
    Smoother();
    Smoother(double startPosVec);
    ~Smoother();

    //
    // non-real-time interface
    //

    // accessors
    void setRealtime(bool value);
    void setSplineType(SA_SplineType value);
    void setLagTime(int lagTimeUSec);
    void invalidate();
    bool isValid();

    // automatically assign the timestamp
    void setCoarseTarg(double targPos);
    // explicitly assign timestamp (must be monotonic)
    void setCoarseTarg(int64_t timestamp, double targPos);

    //
    // real-time interface
    //

    // automatically assign the timestamp
    void reset(double startPos);
    // explicitly assign timestamp
    void reset(int64_t startTime, double startPos);

    // automatically assign the timestamp
    void getSmoothedPVA(double* smoothPos,
                        double* smoothVel = NULL,
                        double* smoothAccel = NULL);
    // explicitly assign timestamp, must be monotonic and >= (time of most recent target -  lagTime)
    void getSmoothedPVA(int64_t curTime,
                        double* smoothPos,
                        double* smoothVel = NULL,
                        double* smoothAccel = NULL);

    void splineDump(char color);
    void debugDump();
  protected:
    //
    // methods
    //

    int64_t getCurTimeUSec();

    void processPendingCoarseTarg();

    void createSpline();

    void evaluateSpline(int64_t time, double* pPos, double* pVel, double* pAccel=NULL);  

    // spline type 1: starting at time 0, go to a new position, and get there with zero slope 
    void createSplineSegmentType1(double pInitial, double vInitial, 
                                  double tFinal, double pFinal);

    // spline type 2 strict:  a spline that starts at pInitial, vInitial, and goes 
    // through (tMid, pMid) and (tFinal, pFinal)
    void createSplineSegmentType2Strict(double pInitial, double vInitial, 
                                        double tMid, double pMid,
                                        double tFinal, double pFinal);

    // spline type 2 loose:  a spline that starts at pInitial, vInitial, and goes 
    // through (tFinal, pFinal) with minimum integrated accel^2 
    void createSplineSegmentType2Loose(double pInitial, double vInitial, 
                                       double tMid, double pMid,
                                       double tFinal, double pFinal);

    //
    // data
    //

    bool bRealtime;

    SA_State state;
    SA_SplineType splineType;
    PendingTargPos pendingCoarseTarg;


    // time stuff
    int lagTime;
    std::deque<int64_t> timeBuf;
    // target positions
    std::deque<double> posBuf;

    // spline coeff's
    double C0;
    double C1;
    double C2;
    double C3;

    // the spline window start state (this is the lower temporal limit)
    int64_t splineWindowStartTime;
    double splineWindowStartPos;
    double splineWindowStartVel;

    // spline features
    int64_t splineWindowEndTime; // the current spline is valid until this time
    double maxAbsPosDelta;
    double overshootDangerFraction;
    double overshootDangerThreshold; 

};

}

#endif

