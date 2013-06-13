// Algorithm description:
// 1) At startup:
//  - The initial position and lag time are specified.
//  - The target point queue is empty.  
//  -  A trivial constant=posInit spline is constructed.
// 
// 2) When we need the current smoothed state:
//  - It is verified that the spline is valid for (current time).
//    -  If no, the first point in the queue is popped, and the spline is updated,
//       and we try again
//    - if yes, the spline is evaluated at (current time) and the state
//      is returned
// 
// 3) When a new target point arrives:
//  - It is placed in the target queue,  time stamped with the current
//    time + lagTime. I.e. the spline goes through the target in lagTime from now
//  - (curTime)  is a lower limit to the spline window.  However,
//    to avoid blocking the real-time thread, processing of new targets is 
//    can be delayed, and a higher lower limit may be established via step 2.  
//    If it is determined that (curTime) is actually a new lower limit
//    then the new start window is set.  Either way, the spline in re-created
//
// 4) When we need to up update the spline:
//  a)
//     - If there is only one point in the queue, then the spline is
//       a trivial constant spline that is valid forever
//  b)
//      - If there are two points in the queue, we create a spline that
//        goes from the current state to the second point in the queue,
//        ending with zero slope.
//      - This spline is valid to the time of the second point.
//  c)
//      - If there are more than two points in the queue, we create 
//        a spline that go from the current state, and that uses 
//        the points in the queue.  
//      - Dangerous splines that oveshoot the target points are detected 
//        and eliminated.
//      - This spline is valid to the time of the second point in the queue.

//#define DEBUG

#include "smoothing/Smoother.h"
#include <assert.h>
#include <limits.h>
#include <string>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#define ROSBUILD
#ifdef ROSBUILD
#include <ros/ros.h>
#else
#define ROS_WARN printf
#define ROS_DEBUG printf
#endif 

#include <unistd.h>

namespace smoothing {

Smoother::Smoother() {
  state = INVALID;
  splineType = LOOSE;
  lagTime = 150000; //uSec
  bRealtime = false;
}


Smoother::Smoother(double startPos) {
  splineType = LOOSE;
  lagTime = 150000; //uSec
  reset(startPos);
  bRealtime = false;
}

void Smoother::setSplineType(SA_SplineType value) {
  state = INVALID;
  splineType = value;
}

void Smoother::setRealtime(bool value) {
  bRealtime = value;
}


void Smoother::reset(double startPos) {
  int64_t startTime = getCurTimeUSec();
  reset(startTime, startPos);
}


void Smoother::reset(int64_t startTime, double startPos) {

  timeBuf.clear();
  posBuf.clear();

  // initialize pendingCoarseTarg
  pendingCoarseTarg.bPending = false;
  pendingCoarseTarg.targPos = startPos;

  // overshoot stuff
  maxAbsPosDelta = 0.0;
  overshootDangerFraction = 0.1;
  overshootDangerThreshold = maxAbsPosDelta*overshootDangerFraction;

  // spline stuff
  C0 = startPos;
  C1 = 0.0;
  C2 = 0.0;
  C3 = 0.0;

  // the most recent smooth state 
  splineWindowStartTime = startTime; 
  splineWindowStartPos = startPos;
  splineWindowStartVel = 0.0;
  splineWindowEndTime = LLONG_MAX; 

  state=VALID;
}

Smoother::~Smoother() {
}


void Smoother::getSmoothedPVA( double* pSmoothPos,
                               double* pSmoothVel,
                               double* pSmoothAccel ) {

  int64_t curTime = getCurTimeUSec();
  getSmoothedPVA( curTime, pSmoothPos, pSmoothVel, pSmoothAccel );
}


void Smoother::getSmoothedPVA( int64_t curTime,
                               double* pSmoothPos,
                               double* pSmoothVel,
                               double* pSmoothAccel ) {

  // this could be dangerous if we are not in a valid state
  // TODO do something besides just crash (what?) 
  if (state != VALID) {
    throw "No previously held position";
  }

  processPendingCoarseTarg();

  // verify that timestamp is in a valid range
  if (curTime < splineWindowStartTime) {
    // this is should not happen, but if it does, 
    // issue a warning and return the earliest value
    ROS_WARN("Attempted to getSmoothedPVA with non-monotomic time");
    curTime = splineWindowStartTime;
  }

  double vel;
  if (pSmoothVel == NULL) {
    pSmoothVel = &vel;
  }

  double accel;
  if (pSmoothAccel == NULL) {
    pSmoothAccel = &accel;
  }

  evaluateSpline(curTime, pSmoothPos, pSmoothVel, pSmoothAccel);
}


void Smoother::invalidate() {
  state = INVALID;
}


bool Smoother::isValid() {
  return state == VALID;
}


void Smoother::setLagTime(int lagTimeUSec) {

  //early out 
  if (state != INVALID) {
    ROS_WARN("Attempted to set lagTime on a non-stopped smoother\n");
    ROS_WARN("You must stop the smoother to make this setting change\n");
    return;
  }

  lagTime = lagTimeUSec;
}


void Smoother::setCoarseTarg(int64_t timestamp, double targPos) {
  //early out 
  if (state == INVALID) {
    ROS_WARN("Attempted to setCoarseTarg on an INVALID smoother\n");
    return;
  }
  

  if(bRealtime) {
    while (pendingCoarseTarg.bPending == true) {
      // wait for real time thread to consume the pendingCoarseTarg
      // it shouldn't take long...
      usleep(100);
    }
  } else {
    if(pendingCoarseTarg.bPending == true)
      ROS_DEBUG("Smashing old target...\n");
  }
    
  
  pendingCoarseTarg.targPos = targPos;
  // the actual targ is lagged by lagTime
  pendingCoarseTarg.timeStamp = timestamp;

  #ifdef DEBUG //LLL
  printf("%%setCoarseTarg time: %lld, pos:%f\n", timestamp, targPos);
  #endif


  pendingCoarseTarg.bPending = true;
}

void Smoother::setCoarseTarg(double targPos) {
  int64_t curTime = getCurTimeUSec();
  setCoarseTarg(curTime, targPos);
}


void Smoother::processPendingCoarseTarg() {

  // early out if pendingCoarseTarg is invalid
  if (pendingCoarseTarg.bPending == false) {
    return;
  }

  int64_t targetTime =  pendingCoarseTarg.timeStamp + lagTime; // the target is lagged

  // skip this pendingCoarseTarg if its targetTime doesn't come after the last targetTime 
  if ( timeBuf.size() > 0 && targetTime <= timeBuf.back() ) {
    // not a good idea to print in a real time thread
    //printf("Warning: Smoother::processPendingCoarseTarg does not come after last targ %lld <= %lld\n", 
    //                                                              pendingCoarseTarg.timeStamp, timeBuf.back());
    pendingCoarseTarg.bPending = false;
    return;
  }

  // back push pendingCoarseTarg onto buffers 
  timeBuf.push_back(targetTime);
  posBuf.push_back(pendingCoarseTarg.targPos);

  // overshoot danger threshold scaling 
  if (posBuf.size() >= 2) {
    double absPosDelta = posBuf.back() - posBuf.at(posBuf.size() - 2);
    if (absPosDelta < 0) absPosDelta = -absPosDelta;
    if (absPosDelta > maxAbsPosDelta) {
      maxAbsPosDelta = absPosDelta;
      overshootDangerThreshold = maxAbsPosDelta*overshootDangerFraction;
    }
  }

  int64_t laggedTime = pendingCoarseTarg.timeStamp;

  // laggedTime is a lower limit to the required buffering.
  if (laggedTime > splineWindowStartTime) {
    // move the splineWindowStart forward to laggedTime
    evaluateSpline(laggedTime, &splineWindowStartPos, &splineWindowStartVel);
    splineWindowStartTime = laggedTime;
  }

  createSpline();

  pendingCoarseTarg.bPending = false;
} 



int64_t Smoother::getCurTimeUSec() {
  timeval curtv;
  gettimeofday( &curtv, NULL );
  int64_t curTimeUSec = (int64_t)(curtv.tv_sec)*1000000 + (int64_t)(curtv.tv_usec);
  return curTimeUSec;
}



void Smoother::evaluateSpline(int64_t time, double* pPos, double* pVel, double* pAccel) {

  if (time <  splineWindowStartTime) {
    ROS_WARN("Attempted to evaluateSpline at a time that is earlier than allowed.  Falling back to allowed time");
    time = splineWindowStartTime;
  }

  if ( time > splineWindowEndTime) {

    // slide down and recurse
    evaluateSpline(splineWindowEndTime, &splineWindowStartPos, &splineWindowStartVel);
    splineWindowStartTime = splineWindowEndTime; // slide splineWindowStartTime down to the splineWindowEndTime
    createSpline(); // build a new spline (resulting in a new splineWindowEndTime)
    // try again
    evaluateSpline(time, pPos, pVel, pAccel);
 
  } else {

    // this is easy.  just evaluate the cubic
    int64_t t = time - splineWindowStartTime;  
    assert(t >= 0);
    *pPos = C0 + (C1 + (C2 + C3 * t) * t) * t;
    *pVel = C1 + (2*C2 + 3*C3 * t) * t;
    if (pAccel != NULL) {
      *pAccel = 2*C2 + 6*C3*t;
    }
    
  } 
}



// create a spline given:
//  -  where we are now (timeBuf.at(0), pos0, vel0)   
//  -  the other (subsequent) targets (timeBuf.at(i), posBuf.at(i))  
// Note: In general, the spline is not required to go through all of the targets,
//       but it is required to go through the current position.  Thus we assume 
//       timeBuf.at(0) represents the current time, and we require specified 
//       (as opposed to inferred) pos0 and vel0 arguments.
void Smoother::createSpline() {

  assert( timeBuf.size() == posBuf.size() );
  
  // early out if there are no target points
  if (timeBuf.size() == 0) {
    return;
  }

  // remove all of the buffered targets that come before splineWindowStartTime 
  if ( splineWindowStartTime >= timeBuf.at(0) ) {
    // front pop all of the times that come before splineWindowStartTime 
    // (within a tolerance to avoid points that are too close to each other)
    // The tolerance will be the time between samples divided by tolDen.
    int tolDen = 100;
    int64_t tol;
    if (timeBuf.size() > 1) {
      tol = (timeBuf.at(1) - timeBuf.at(0)) / tolDen;
    } else {
      tol = 0;
    }
    while ( !timeBuf.empty()  && (timeBuf.at(0) <= splineWindowStartTime + tol)) {
      if (timeBuf.size() > 1) { 
        tol = ( timeBuf.at(1) - timeBuf.at(0) ) / tolDen;
      } else {
        tol = 0;
      }
      timeBuf.pop_front();
      posBuf.pop_front();
    }
  }

  assert(posBuf.size() == timeBuf.size());

  #ifdef DEBUG //LLL
  printf("%%createSpline pos0: %f, numTargPts: %d\n", pos0, posBuf.size());
  #endif

  unsigned int numTargPts = posBuf.size();

  double t1;
  if (numTargPts == 0) {

    C0 = splineWindowStartPos;
    C1 = 0.0;
    C2 = 0.0;
    C3 = 0.0;
    t1 = 0.0;
    splineWindowEndTime = LLONG_MAX; 

  } else if (numTargPts == 1) {
     
    // create a spline from splineWindowStartPos, splineWindowStartVel @ splineWindowStartTime to p1 @ t1 ending in zero slope 
    t1 = (double) (timeBuf.at(0) - splineWindowStartTime);
    assert(t1 >= 0);
    double p1 = posBuf.at(0);
    createSplineSegmentType1(splineWindowStartPos, splineWindowStartVel, t1, p1); 
    splineWindowEndTime = timeBuf.at(0); 

  } else {

    assert(numTargPts > 1);

    // create a spline from pos0, vel0 though p1 at t1 and p2 at t2 
    t1 = (double) (timeBuf.at(0) - splineWindowStartTime);
    assert(t1 >= 0);
    double p1 = posBuf.at(0);

    if (splineType == STRICT) {
   
      double t2 = (double) (timeBuf.at(1) - splineWindowStartTime);
      double p2 = posBuf.at(1);
 
      createSplineSegmentType2Strict(splineWindowStartPos, splineWindowStartVel, t1, p1, t2, p2); 
      splineWindowEndTime = timeBuf.at(0); 

    } else if (splineType == LOOSE) {

      double tf = (double) (timeBuf.back() - splineWindowStartTime);
      double pf = posBuf.back();

      createSplineSegmentType2Loose(splineWindowStartPos, splineWindowStartVel, t1, p1, tf, pf); 
      splineWindowEndTime = timeBuf.at(0); 

      // modify mid points to be consistent with spline  
      for (unsigned int idx = 0; idx < (posBuf.size() - 1);  idx++) {
        double tidx = timeBuf.at(idx) - splineWindowStartTime;
        posBuf.at(idx) = C0+(C1+(C2+C3*tidx)*tidx)*tidx;
      }

    } else {  // undefined splineType

      // this should never happen
      assert(false);
    }
 
  }
}


void Smoother::createSplineSegmentType1(double pInitial, double vInitial, 
                                        double tFinal, double pFinal ) {
  // type 1: starting at time 0, go to a new position, and get there with zero slope 

  C0 = pInitial;
  C1 = vInitial;
  C2 = ((3*(pFinal-pInitial)/tFinal) - (2*vInitial))/tFinal;
  C3 = (2*(pInitial-pFinal)/tFinal + vInitial)/tFinal/tFinal;

  // we guard against big overshoots
  // maxima occur at d/dt(C0+C1*t+C2*t^2+C3*t^3) = 0
  // tmaxima1 = 1/6/C3*(-2*C2+2*sqrt(C2^2-3*C3*C1))
  // tmaxima2 = 1/6/C3*(-2*C2-2*sqrt(C2^2-3*C3*C1))
  bool bDanger = false;
  double discriminant = C2*C2-3.0*C3*C1;
  double tmax;
  if (discriminant >= 0 && C3 != 0.0 ) {
    // the plus case
    tmax =  (-2.0*C2 + 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tFinal) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pFinal-pInitial)*tmax/tFinal);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
    // the minus case
    tmax =  (-2.0*C2 - 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tFinal) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pFinal-pInitial)*tmax/tFinal);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
  }
  if (bDanger) {
    // fall back to linear 
    C0 = pInitial;
    C1 = (pFinal-pInitial)/tFinal;
    C2 = 0;
    C3 = 0;
  }
}

void Smoother::createSplineSegmentType2Strict(double pInitial, double vInitial, 
                                              double tMid, double pMid,
                                              double tFinal, double pFinal) {
    
  // type 2: starting at time 0, go to a new position, 
  // with consideration of where we are going in the future
  C0 = pInitial;
  C1 = vInitial;

  // a spline that starts at pInitial, with slope vInital
  // and goes through (tMid, pMid) and (tFinal,pFinal)
  double tFinal2 = (double)tFinal * (double)tFinal;
  double tFinal3 = tFinal2 * (double)tFinal;
  double tMid2 = (double)tMid * (double)tMid;
  double tMid3 = tMid2 * (double)tMid;
  double den = tMid2*tFinal2*(double)(tMid-tFinal);

  //a0 = -((v0*t2-p2+p0)*t1^3-t2^3*v0*t1+(p1-p0)*t2^3)/t2^2/t1^2/(t1-t2);
  //*pC2 = -((v0*t2-p2+p0)*t1^3 + (p1-p0-v0*t1)*t2^3)/den;
  C2 = -((vInitial*tFinal-pFinal+pInitial)*tMid3 + (pMid-pInitial-vInitial*tMid)*tFinal3)/den;

  //a0 = ((p1-p0-v0*t1)*t2^2+t1^2*v0*t2+(-p2+p0)*t1^2)/t2^2/t1^2/(t1-t2); 
  //*pC3 = ((p1-p0-v0*t1)*t2^2  + (-p2+p0+v0*t2)*t1^2)/den; 
  C3 = ((pMid-pInitial-vInitial*tMid)*tFinal2  + (-pFinal+pInitial+vInitial*tFinal)*tMid2)/den; 


  // we guard against big overshoots
  // maxima occur at d/dt(C0+C1*t+C2*t^2+C3*t^3) = 0
  // tmaxima1 = 1/6/C3*(-2*C2+2*sqrt(C2^2-3*C3*C1))
  // tmaxima2 = 1/6/C3*(-2*C2-2*sqrt(C2^2-3*C3*C1))
  bool bDanger = false;
  double discriminant = C2*C2-3.0*C3*C1;
  double tmax;
  if (discriminant >= 0 && C3 != 0.0 ) {
    // the plus case
    tmax =  (-2.0*C2 + 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
    // the minus case
    tmax =  (-2.0*C2 - 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
  }
  if (bDanger) {
    // fall back to type 1
    createSplineSegmentType1(pInitial, vInitial, tMid, pMid); 
  }
}

void Smoother::createSplineSegmentType2Loose(double pInitial, double vInitial, 
                                             double tMid, double pMid,
                                             double tFinal, double pFinal) {
  //     Create spline from the current position and velocity, through the last point in the queue
  //     with minimum integrated accel^2.
    
  // type 2: starting at time 0, go to a new position, 
  // with consideration of where we are going in the future
  C0 = pInitial;
  C1 = vInitial;

  // a spline that goes through (tFinal,pFinal) with minimum integrated acceleration^2
  C2=-3.0*(-pFinal+pInitial+vInitial*tFinal)/2.0/tFinal/tFinal;
  C3=(pFinal-pInitial-vInitial*tFinal-C2*tFinal*tFinal)/tFinal/tFinal/tFinal;

  // we guard against big overshoots
  // maxima occur at d/dt(C0+C1*t+C2*t^2+C3*t^3) = 0
  // tmaxima1 = 1/6/C3*(-2*C2+2*sqrt(C2^2-3*C3*C1))
  // tmaxima2 = 1/6/C3*(-2*C2-2*sqrt(C2^2-3*C3*C1))
  bool bDanger = false;
  double discriminant = C2*C2-3.0*C3*C1;
  double tmax;
  if (discriminant >= 0 && C3 != 0.0 ) {
    // the plus case
    tmax =  (-2.0*C2 + 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
    // the minus case
    tmax =  (-2.0*C2 - 2.0 * sqrt(discriminant))/6.0/C3;
    if (tmax > 0 && tmax < tMid) {
      double pmax =  C0 + (C1 + (C2 + C3*tmax) * tmax) * tmax;
      double dangerMeasure = pmax - (pInitial + (pMid-pInitial)*tmax/tMid);
      if (dangerMeasure < 0) dangerMeasure = -dangerMeasure;
      if (dangerMeasure > overshootDangerThreshold) bDanger = true;
    }
  }
  if (bDanger) {
    // fall back to type 1
    createSplineSegmentType1(pInitial, vInitial, tMid, pMid); 
  }
}


void Smoother::debugDump() {

  printf("%%Smoother Debug dump\n");
  printf("%%-------------------------------\n");

  printf("%%Arm times: \n");
  printf("%%lagTime: %d\n", lagTime);
  for (unsigned int i=0; i<timeBuf.size(); i++) {
    printf("%%timeBuf.at(%d): %lld\n", i, timeBuf.at(i));
  }
  printf("\n");

  printf("%%Joint target positions: \n");
  for (unsigned int i=0; i<posBuf.size(); i++) {
    printf("%%posBuf.at(%d): %f\n", i, posBuf.at(i));
  }
  printf("\n");

  printf("%%Joint spline: \n");
  printf("%%spline data: C0 %e C1 %e C2 %e C3 %e\n", C0, C1, C2, C3);
  printf("\n");
}



void Smoother::splineDump(char color) {

  printf("[t,p]=plotSpline(%lld, %lld, %e, %e, %e, %e);\n", 
                  timeBuf.at(0), timeBuf.at(1), 
                                           C0, C1, C2, C3); 
  printf("plot(t,p,'%c');\n", color); 
  printf("hold on;\n\n"); 
  printf("plot(%lld, %f, '%cx');\n", timeBuf.at(1), posBuf.at(1), color);
  printf("plot(%lld, %f, '%cx');\n", timeBuf.back(), posBuf.back(), color);
  printf("\n");
}

} // end namespace

