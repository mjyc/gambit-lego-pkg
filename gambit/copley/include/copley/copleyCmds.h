#ifndef __COPLEY_CMDS_H__ 
#define __COPLEY_CMDS_H__ 


#include <stdint.h>
#include "copley/copleyErrors.h"


// normally there is no need to use this function
// because the socket handle is maintained internally
int getSocketHandle(int* pSkt); 

int socketCanInit(const char* interfaceName, uint32_t timeoutUsec);

int copleyAmpInit( uint8_t id );

int copleyGetCurrentLimits( uint8_t id, int16_t* pPeakLimit, int16_t* pPeakLimitTime, int16_t* pContinuousLimit );

// current units: 0.01 amps, valid range 0 -> 32767
// time units: mSec, valid range 0 -> 10000
int copleySetCurrentLimits( uint8_t id, int16_t peakLimit, int16_t peakLimitTime, uint16_t continuousLimit );

// to sync all the nodes, call this once 
int copleySendSyncSignal( void );
// then call this once for each node
int copleyDoSync( uint8_t id );

int copleyCheckIfHomed(uint8_t id, bool *pIsHomed);

int copleyHomeIsCurPos(uint8_t id);

int copleyHomeByHomeSwitch(uint8_t id, int bSearchFwd, uint32_t searchSizeCounts);

int copleyStartInterpPosMove(uint8_t id, uint8_t lagDepth, uint8_t nominalSampleTime_mSec); 

// Note: Wait about 1 nominalSampleTime_mSec after calling copleyStartInterpPosMove
// before calling copleySetInterpTargPos, then 1 nominalSampleTime_mSec between each
// subsequent call to copleySetInterpTargPos.
// If the point buffer underflows, then the InterpPosMove will stop, and this function 
// will return an error (ERROR_MOVE_NOT_ACTIVE).  If this happens, then 
// copleyStartInterpPosMove must be called again.
int copleySetInterpTargPos(uint8_t id, int32_t targPos); 

int copleyStopInterpPosMove(uint8_t id); 

int copleyGetCurPos(uint8_t id, int32_t* pCurPos);

int copleyQuickStop(uint8_t id);

#endif
