#include "copley/copleyDictionary.h"
#include "copley/copleySockCanSdo.h"
#include "copley/copleyCmds.h"

#include <assert.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define DEBUG

/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#define INTERP_POS_COBID_BASE  0x200 //PDO for interp position move

#define RIGID_ASSUMPTIONS false

// globals
int socketHandle;
uint32_t moveDoneTimeArray_usec[128];
uint8_t lagDepthArray[128];
uint8_t sampTimeArray_msec[128];
uint8_t segIntegrityCounter[128];

uint32_t syncTime_usec;

//
//
// Private Functions
//
//

char bitsString[255];
char* getBits(uint32_t inputData) {
  int nibbleIdx, nibbleSubIdx, bitIdx, charIdx;
  uint32_t mask;
  charIdx = 0;
  for (nibbleIdx = 7; nibbleIdx >= 0; nibbleIdx--) {
    for (nibbleSubIdx = 3; nibbleSubIdx >= 0; nibbleSubIdx--) {
      bitIdx = nibbleIdx * 4 + nibbleSubIdx;
      mask = 1 << bitIdx;
      if ((inputData & mask) == 0) {
        bitsString[charIdx] = '0';
      } else {
        bitsString[charIdx] = '1';
      } 
      charIdx++;
    }
    bitsString[charIdx] = ' ';
    charIdx++;
  }
  // null termination
  bitsString[charIdx] = 0;  
  return bitsString;
}


int getLocalTime(uint32_t* pCurTime_usec) {
  // returns the current time in usec
  uint64_t result64;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  
  result64 = tv.tv_sec*1000000 + tv.tv_usec;
  *pCurTime_usec = (uint32_t) (result64 & 0xffffffff);
  return ERROR_NONE;
}


int setInterpTargPos(uint8_t id, int32_t targPos, uint8_t msec) {

  assert(id < 128);

  #if 0
  // SDO-based code
  // interp position target
  uint32_t targPosData = (uint32_t)targPos;
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, INTERPOLATION_DATA_RECORD, 1, 4, targPosData, RIGID_ASSUMPTIONS ) );
  

  // This is the way the variable-time should be set according to the copley canopen programmers manual
  // but it doesnt work.  See note below
  //RETURN_ON_ERROR( writeSdoData( socketHandle, id, INTERPOLATION_DATA_RECORD, 2, 1, msec, RIGID_ASSUMPTIONS ) );

  // The copley canopen programmers manual is very clear about how this is supposed to 
  // work (i.e. in linear interp, variable time mode, you set sub-indices 1 and 2 of 0x60c1)
  // However, after much thrashing, I have concluded that this just doesn't work.  Instead,
  // the variable time is set via the 0x60c2 index.  This is probably a firmware bug on the 
  // copley box
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, INTERPOLATION_CONSTANT_TIME, 1, 1, msec, RIGID_ASSUMPTIONS ) );
  #endif

  #if 0
  // pdo to INTERPOLATION_DATA_RECORD and INTERPOLATION_CONSTANT_TIME
  RETURN_ON_ERROR( writePdoData(socketHandle, INTERP_POS_COBID_BASE+id, 5, (uint32_t)targPos,  (uint32_t)msec) );
  #endif

  # if 1
  // pdo to IP_MOVE_SEGMENT_COMMAND
  uint32_t word1, word2;
  //word1 = [ headerByte msec pos0 pos1 ] 
  //word2 = [ pos2 pos3 0 0 ]
  uint8_t headerByte = (0x5<<3) + (segIntegrityCounter[id] & 0x7);
  uint8_t pos0 = targPos & 0xff;
  uint8_t pos1 = (targPos & 0xff00) >> 8;
  uint8_t pos2 = (targPos & 0xff0000) >> 16;
  uint8_t pos3 = (targPos & 0xff000000) >> 24;
  word1 = headerByte + (msec << 8) + (pos0 << 16) + (pos1 << 24);
  word2 = pos2 + (pos3 << 8);
  RETURN_ON_ERROR( writePdoData(socketHandle, INTERP_POS_COBID_BASE+id, 8, word1, word2) );
  segIntegrityCounter[id]++;

  // DEBUG
  #if 0
  uint32_t data;
  uint16_t numBuf;
  RETURN_ON_ERROR( readSdoData( socketHandle, id, TRAJECTORY_BUFFER_FREE_COUNT, 0, &data, RIGID_ASSUMPTIONS ) ); 
  numBuf = 0x20 - (0x0000ffff & data);
  printf("numBuf : %d\n", numBuf);
  RETURN_ON_ERROR( readSdoData( socketHandle, id, TRAJECTORY_BUFFER_STATUS, 0, &data, RIGID_ASSUMPTIONS ) ); 
  printf("LLL tbs: 0x%x: %s\n", data, getBits(data));
  RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &data, RIGID_ASSUMPTIONS ) ); 
  printf("status word: 0x%x: %s\n", data & 0xffff, getBits(data & 0xffff));
  RETURN_ON_ERROR( readSdoData( socketHandle, id, CONTROL_WORD, 0, &data, RIGID_ASSUMPTIONS ) ); 
  printf("control word: 0x%x: %s\n", data & 0xffff, getBits(data & 0xffff));
  RETURN_ON_ERROR( readSdoData( socketHandle, id, MFG_STATUS_REGISTER, 0, &data, RIGID_ASSUMPTIONS ) ); 
  printf("mfg status: 0x%x: %s \n", data, getBits(data) );
  printf("segIntegrityCounterBits: 0x%x\n\n", segIntegrityCounter[id] & 0x7);
  #endif

  #endif

  return ERROR_NONE;

}

int enableInterpPosPdo(uint8_t id) {

  // follow procedure on pg 28

  #if 0
  //
  // this code uses INTERPOLATION_DATA_RECORD and INTERPOLATION_CONSTANT_TIME
  //

  // disable pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 0, 1, 0, RIGID_ASSUMPTIONS ) );
 
  // set interp pos cobid
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_COM_PARAM_0, 1, 4, INTERP_POS_COBID_BASE+id, RIGID_ASSUMPTIONS ) );

  // immediate response to pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_COM_PARAM_0, 2, 1, 0xff, RIGID_ASSUMPTIONS ) );
  
  // map 4 bytes to index 0x60c1 (INTERPOLATION_DATA_RECORD), subindex 1
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 1, 4, 0x60c10120, RIGID_ASSUMPTIONS ) );

  // map 1 byte to index 0x60c2 (INTERPOLATION_CONSTANT_TIME), subindex 2
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 2, 4, 0x60c20108, RIGID_ASSUMPTIONS ) );

  // enable pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 0, 1, 2, RIGID_ASSUMPTIONS ) );
  #endif


  //
  // this code uses IP_MOVE_SEGMENT_COMMAND
  //
  // disable pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 0, 1, 0, RIGID_ASSUMPTIONS ) );
 
  // set interp pos cobid
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_COM_PARAM_0, 1, 4, INTERP_POS_COBID_BASE+id, RIGID_ASSUMPTIONS ) );

  // immediate response to pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_COM_PARAM_0, 2, 1, 0xff, RIGID_ASSUMPTIONS ) );
  
  // map 8 bytes to index 2010 (IP_MOVE_SEGMENT_COMMAND), subindex 0
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 1, 4, 0x20100040, RIGID_ASSUMPTIONS ) );

  // enable pdo
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, RX_PDO_MAPPING_PARAMETERS0, 0, 1, 1, RIGID_ASSUMPTIONS ) );

  return ERROR_NONE;

}

#define MODE_PROFILE_POSITION        1
#define MODE_PROFILE_VELOCITY        3
#define MODE_PROFILE_TORQUE          4
#define MODE_HOMING                  6
#define MODE_INTERPOLATED_POSITION   7
int setMode(uint8_t id, uint8_t mode) {
  //mode (pg 59)
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, MODE_OF_OPERATION, 0, 1, mode, RIGID_ASSUMPTIONS ) );

  // verify mode
  int curTry = 0;
  int maxTry = 10;
  uint32_t curMode = 0;
  for (curTry=0; curTry < maxTry; curTry++) {
    RETURN_ON_ERROR( readSdoData( socketHandle, id, MODE_OF_OPERATION_DISPLAY, 0, &curMode, RIGID_ASSUMPTIONS ) ); 
    if ((uint8_t)curMode == mode) break;
  }
  if (curTry == maxTry) {
    RETURN_ON_ERROR( ERROR_BAD_DATA );
  } 

  return ERROR_NONE;
}

//
//
// Public Functions
//
//
  

int getSocketHandle(int* pSkt) {
  *pSkt = socketHandle;
  return ERROR_NONE;
}


int socketCanInit(const char* interfaceName, uint32_t timeoutUsec) {

  /* Create the socket */
  socketHandle = socket( PF_CAN, SOCK_RAW, CAN_RAW );
  #ifdef DEBUG
  printf("socket created\n");
  #endif

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = timeoutUsec;
  if ((setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))) == -1) {
    perror("catch - setsockopt SO_RCVTIMEO failed");
    return ERROR_FAILED_SOCKET;
  }
  if ((setsockopt(socketHandle, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout))) == -1) {
    perror("catch - setsockopt SO_RCVTIMEO failed");
    return ERROR_FAILED_SETSOCKOPT;
  }

  /* Locate the interface you wish to use */
  struct ifreq ifr;
  strcpy(ifr.ifr_name, interfaceName);
  ioctl(socketHandle, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled 
                                 * with that device's index */
  #ifdef DEBUG
  printf("interface located\n");
  #endif

  /* Select that CAN interface, and bind the socket to it. */
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind( socketHandle, (struct sockaddr*)&addr, sizeof(addr) );
  #ifdef DEBUG
  printf("socket bind\n");
  #endif

  return ERROR_NONE;
}


int copleyAmpInit( uint8_t id ) {

  RETURN_ON_ERROR( resetNode( socketHandle, id ) ); 

  int i, n;

  // disable all PDO's
  uint16_t mapIndex[] = { RX_PDO_MAPPING_PARAMETERS0, RX_PDO_MAPPING_PARAMETERS1, 
                          RX_PDO_MAPPING_PARAMETERS2, RX_PDO_MAPPING_PARAMETERS3, 
                          RX_PDO_MAPPING_PARAMETERS4, RX_PDO_MAPPING_PARAMETERS5, 
                          RX_PDO_MAPPING_PARAMETERS6, RX_PDO_MAPPING_PARAMETERS7,
                          TX_PDO_MAPPING_PARAMETERS0, TX_PDO_MAPPING_PARAMETERS1, 
                          TX_PDO_MAPPING_PARAMETERS2, TX_PDO_MAPPING_PARAMETERS3, 
                          TX_PDO_MAPPING_PARAMETERS4, TX_PDO_MAPPING_PARAMETERS5, 
                          TX_PDO_MAPPING_PARAMETERS6, TX_PDO_MAPPING_PARAMETERS7 };
  n = 16; // number of entries in index

  for (i=0; i<n; i++) {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, mapIndex[i], 0x0, 1, 0x0, false ) );
  }
  flushCanReadBuffer(socketHandle);
/*
  // now invalidate all PDO (just to make sure)
  uint16_t comIndex[] = { RX_PDO_COM_PARAM_0, RX_PDO_COM_PARAM_1, 
                          RX_PDO_COM_PARAM_2, RX_PDO_COM_PARAM_3, 
                          RX_PDO_COM_PARAM_4, RX_PDO_COM_PARAM_5, 
                          RX_PDO_COM_PARAM_6, RX_PDO_COM_PARAM_7, 
                          TX_PDO_COM_PARAM_0, TX_PDO_COM_PARAM_1, 
                          TX_PDO_COM_PARAM_2, TX_PDO_COM_PARAM_3, 
                          TX_PDO_COM_PARAM_4, TX_PDO_COM_PARAM_5, 
                          TX_PDO_COM_PARAM_6, TX_PDO_COM_PARAM_7 };
  n = 16; // number of entries in index

  for (i=0; i<n; i++) {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, comIndex[i], 0x0, 1, 0x80000000, false ) );
  }
  flushCanReadBuffer(socketHandle);
*/

  // now configure the InterpPosPdo 
  RETURN_ON_ERROR( enableInterpPosPdo(id) );

  //write desired state
  //set desired state to 30 (0x1e) servo mode, canOpen per pg 60
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, DESIRED_STATE, 0, 2, 0x1e, RIGID_ASSUMPTIONS ) );

  RETURN_ON_ERROR( startNode( socketHandle, id ) ); 

  return ERROR_NONE;
}

int copleyGetCurrentLimits( uint8_t id, int16_t* pPeakLimit, int16_t* pPeakLimitTime, int16_t* pContinuousLimit ) {
  uint32_t peakLimit;
  uint32_t peakLimitTime;
  uint32_t continuousLimit;
  RETURN_ON_ERROR( readSdoData( socketHandle, id, USER_PEAK_CURRENT_LIMIT, 0, &peakLimit, RIGID_ASSUMPTIONS ) ); 
  RETURN_ON_ERROR( readSdoData( socketHandle, id, USER_CONTINUOUS_CURRENT_LIMIT, 0, &continuousLimit, RIGID_ASSUMPTIONS ) ); 
  RETURN_ON_ERROR( readSdoData( socketHandle, id, USER_PEAK_CURRENT_LIMIT_TIME, 0, &peakLimitTime, RIGID_ASSUMPTIONS ) ); 
  *pPeakLimit = peakLimit;
  *pPeakLimitTime = peakLimitTime;
  *pContinuousLimit = continuousLimit;
  return ERROR_NONE;
}

int copleySetCurrentLimits( uint8_t id, int16_t peakLimit, int16_t peakLimitTime, uint16_t continuousLimit ) {
  if (peakLimit > 0) {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, USER_PEAK_CURRENT_LIMIT, 0, 2, peakLimit, RIGID_ASSUMPTIONS ) );
  }
  if (continuousLimit > 0) {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, USER_CONTINUOUS_CURRENT_LIMIT, 0, 2, continuousLimit, RIGID_ASSUMPTIONS ) );
  }
  if (peakLimitTime > 0 && peakLimitTime <= 10000) { 
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, USER_PEAK_CURRENT_LIMIT_TIME, 0, 2, peakLimitTime, RIGID_ASSUMPTIONS ) );
  }
  return ERROR_NONE;
}

int copleySendSyncSignal( void ) {
  RETURN_ON_ERROR( syncNode( socketHandle ) ); 
  getLocalTime(&syncTime_usec);
  return ERROR_NONE;
}


int copleyDoSync( uint8_t id ) {
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, HIGH_RESOLUTION_TIMESTAMP, 0, 4, syncTime_usec, true ) );
  return ERROR_NONE;
}


int copleyHomeIsCurPos(uint8_t id) {

  //set homing method (pg 157)
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, HOMING_METHOD, 0, 1, 0x0, RIGID_ASSUMPTIONS ) );

  RETURN_ON_ERROR( setMode(id, MODE_HOMING) );

  // homing move is started by setting bit 4 of control word 0x6040 (see pg 139)
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, CONTROL_WORD, 0, 2, 0x001f, RIGID_ASSUMPTIONS ) );

  //read status word, verify homed (bit 12 is set) 
  int curTry = 0;
  int maxTry = 10;
  uint32_t curStatus = 0;
  for (curTry=0; curTry < maxTry; curTry++) {
    RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &curStatus, RIGID_ASSUMPTIONS ) ); 
    if (curStatus & 0x1000) break;
  }
  if (curTry == maxTry) {
    RETURN_ON_ERROR( ERROR_BAD_DATA );
  } 

  return ERROR_NONE;
}

int copleyCheckIfHomed(uint8_t id, bool *pIsHomed) {
    RETURN_ON_ERROR( setMode(id, MODE_HOMING) );

    uint32_t curStatus; 
    RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &curStatus, RIGID_ASSUMPTIONS ) ); 
    *pIsHomed = (curStatus & 0x1000);
    
    return ERROR_NONE;
}

int copleyHomeByHomeSwitch(uint8_t id, int bSearchFwd, uint32_t searchSizeCounts) {

  // set curpos to 0
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, POSITION_ACTUAL_VALUE, 0x0, 4, 0x0, RIGID_ASSUMPTIONS ) );

  //set homing method (pg 157)
  if (bSearchFwd) {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, HOMING_METHOD, 0, 1, 19, RIGID_ASSUMPTIONS ) );
  } else {
    RETURN_ON_ERROR( writeSdoData( socketHandle, id, HOMING_METHOD, 0, 1, 21, RIGID_ASSUMPTIONS ) );
  }

  RETURN_ON_ERROR( setMode(id, MODE_HOMING) );

  // homing move is started by setting bit 4 of control word 0x6040 (see pg 139)
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, CONTROL_WORD, 0, 2, 0x001f, RIGID_ASSUMPTIONS ) );

  // now wait for homing success (or failure) 
  struct timeval tv;
  uint64_t startTime_usec, curTime_usec, timeoutTime_usec;
  gettimeofday(&tv, NULL);
  startTime_usec = tv.tv_sec*1000000 + tv.tv_usec;
  curTime_usec = startTime_usec;
  timeoutTime_usec = startTime_usec + 30000000; // time out in 30 seconds
  int32_t curPos;

  uint32_t curStatus; 
  while (1) {
    RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &curStatus, RIGID_ASSUMPTIONS ) ); 
   
    if (curStatus & 0x1000) { // bit 12 (homing attained)
      // success
      return ERROR_NONE;
    }
    gettimeofday(&tv, NULL);
    curTime_usec = tv.tv_sec*1000000 + tv.tv_usec;
    if ( curTime_usec > timeoutTime_usec) {
      copleyQuickStop(id);
      return ERROR_TIMEOUT;  
    }
    
    RETURN_ON_ERROR( copleyGetCurPos(id, &curPos) );
    if (bSearchFwd) {
      if (curPos > (int32_t)searchSizeCounts) {
        copleyQuickStop(id);
        return ERROR_HOME_NOT_FOUND;
      }
    } else {
      if (curPos < -((int32_t)searchSizeCounts)) {
        copleyQuickStop(id);
        return ERROR_HOME_NOT_FOUND;
      }
    }
    usleep(5000);
  }
  
  assert(false); // we should never be here
  return ERROR_NONE;
}


int copleyGetCurPos(uint8_t id, int32_t* pCurPos) {
  uint32_t curPosData;
  RETURN_ON_ERROR( readSdoData( socketHandle, id, POSITION_ACTUAL_VALUE, 0, &curPosData, RIGID_ASSUMPTIONS ) ); 
  // reinterpret as a signed value;
  *pCurPos = (int32_t)curPosData;
  return ERROR_NONE;
}



int copleyStartInterpPosMove(uint8_t id, uint8_t lagDepth, uint8_t sampTime_msec) {

  assert(id<128);

  // save lagDepth 
  lagDepthArray[id] = lagDepth;
  // save sampTime
  sampTimeArray_msec[id] = sampTime_msec;

  //#interpolated position mode pg59
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, MODE_OF_OPERATION, 0, 1, 7, RIGID_ASSUMPTIONS ) );

  //#set tracking error window
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, TRACKING_ERROR_WINDOW, 0, 4, 0x1fffffff, RIGID_ASSUMPTIONS ) );

  //#interpolation submode (linear interp with variable time) pg 189
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, INTERPOLATION_SUBMODE_SELECT, 0, 2, 0x0000ffff, RIGID_ASSUMPTIONS ) );

  // get ready to plunk down the current position as the 1st target
  int32_t curPos;
  RETURN_ON_ERROR( copleyGetCurPos(id, &curPos) ); 
 
  // clear the buffer 
  RETURN_ON_ERROR( writePdoData(socketHandle, INTERP_POS_COBID_BASE+id, 8, 0x80, 0x0) );

  // clear the errors
  RETURN_ON_ERROR( writePdoData(socketHandle, INTERP_POS_COBID_BASE+id, 8, 0xff82, 0x0) );

  // set the segIntegrityCounter
  uint32_t tbs;
  RETURN_ON_ERROR( readSdoData( socketHandle, id, TRAJECTORY_BUFFER_STATUS, 0, &tbs, RIGID_ASSUMPTIONS ) ); 
  segIntegrityCounter[id] = tbs & 0x7;


  // now do the plunking 
  // we have to plunk down 2 more samples than our desired lag depth for some reason
  uint8_t i;
  for (i=0; i<lagDepthArray[id]+2; i++) {
    RETURN_ON_ERROR( setInterpTargPos(id, curPos, sampTimeArray_msec[id]) );
  }
  
  // kick off
  RETURN_ON_ERROR( writeSdoData( socketHandle, id, CONTROL_WORD, 0, 2, 0x8f, RIGID_ASSUMPTIONS ) );

  RETURN_ON_ERROR( writeSdoData( socketHandle, id, CONTROL_WORD, 0, 2, 0x1f, RIGID_ASSUMPTIONS ) );

  // modify the moveDoneTime...
  uint32_t curTime_usec;
  getLocalTime(&curTime_usec);
  moveDoneTimeArray_usec[id] =  curTime_usec + (sampTimeArray_msec[id] * 1000 * lagDepthArray[id]); 

  return ERROR_NONE;
}


#define SYNCPERIOD 1000000
int copleySetInterpTargPos(uint8_t id, int32_t targPos) {

  uint32_t curTime_usec;

  assert(id<128);

  getLocalTime(&curTime_usec);

  // this sample should take us lagDepth sampTimes into the future
  uint32_t targDoneTime_usec = curTime_usec + (sampTimeArray_msec[id] * 1000 * lagDepthArray[id]);
  // determine the length of this segment
  // we are already sheduled till moveDoneTime
  uint32_t deltaTime_usec = targDoneTime_usec - moveDoneTimeArray_usec[id];
  uint32_t deltaTime_msec = deltaTime_usec/1000;
  if (ERROR_VERBOSITY_LEVEL+0 > 1) {
    printf("copleySetInterpTargPos deltaTime_msec: %d\n", deltaTime_msec);
  }
  

  // now do the plunking 
  if (deltaTime_msec == 0) {
    // TODO quickstop
    return ERROR_TOO_FAST;
  } else if (deltaTime_msec > 255) {
    // TODO quickstop
    return ERROR_TOO_SLOW;
  }

  // new done time... 
  moveDoneTimeArray_usec[id] = moveDoneTimeArray_usec[id] + deltaTime_msec * 1000;

  RETURN_ON_ERROR( setInterpTargPos(id, targPos, (uint8_t)deltaTime_msec) );

  // verify that the move is still active
  uint32_t statusWord;
  RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &statusWord, RIGID_ASSUMPTIONS ) ); 
  if ((statusWord & 0x4000) == 0) {
    return ERROR_MOVE_NOT_ACTIVE;
  } else {
    return ERROR_NONE;
  }
}  


int copleyStopInterpPosMove(uint8_t id) { 
  // sent a zero time point to the buffer
  RETURN_ON_ERROR( setInterpTargPos(id, 0, 0) );
  return ERROR_NONE;
}


int copleyQuickStop(uint8_t id) {

  RETURN_ON_ERROR( writeSdoData( socketHandle, id, CONTROL_WORD, 0, 2, 0x000b, RIGID_ASSUMPTIONS ) );

  // wait for quick stop to complete 
  uint32_t curTime_usec, startTime_usec;
  uint32_t statusWord;
  uint32_t bFinished;

  getLocalTime(&startTime_usec);
  curTime_usec = startTime_usec;

  RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &statusWord, RIGID_ASSUMPTIONS ) ); 
  //printf("LLL statuswirdey: 0x%x\n", statusWord);
  bFinished = statusWord & 0x20;

  while (bFinished == 0) {
    getLocalTime(&curTime_usec);
    if (curTime_usec - startTime_usec > 1000000) { // timeout in 1 sec
      return ERROR_TIMEOUT; 
    }
    RETURN_ON_ERROR( readSdoData( socketHandle, id, STATUS_WORD, 0, &statusWord, RIGID_ASSUMPTIONS ) ); 
    bFinished = statusWord & 0x20;
  }

  return ERROR_NONE;
}
