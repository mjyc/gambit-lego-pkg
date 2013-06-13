#ifndef __COPLEY_SOCK_CAN_SDO_H__ 
#define __COPLEY_SOCK_CAN_SDO_H__ 


#include <stdint.h>
#include "copley/copleyErrors.h"

#ifndef __cplusplus
#ifndef true
  #define true 1
#endif
#ifndef false 
  #define false 0
#endif
#ifndef bool 
  typedef int bool;
#endif
#endif // __cplusplus



#define DEBUG

int resetNode( int skt, uint8_t id ); 

int disableAllPdos( int skt, uint8_t id );

int startNode( int skt, uint8_t id ); 

int stopNode( int skt, uint8_t id ); 

int syncNode( int skt ); 

int flushCanReadBuffer( int skt );

int readSdoData( int skt, uint8_t id, uint16_t index, uint8_t subindex, uint32_t* pReadData, bool bRigidAssumptions ); 

int writeSdoData( int skt, uint8_t id, uint16_t index, uint8_t subindex, uint8_t len, uint32_t writeData, bool bRigidAssumptions ); 

int writePdoData(int skt, uint16_t cobid, uint8_t len, uint32_t writeData1, uint32_t writeData2);

#endif
