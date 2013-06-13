#include "copley/copleyCmds.h"
#include "copley/copleySockCanSdo.h"
#include "copley/copleyDictionary.h"
#include <stdio.h>
#include <math.h> 
#include <unistd.h>
 
int main(void) { 

  uint8_t id = 0xf;
  int skt;

  RETURN_ON_ERROR( socketCanInit("can0", 999999) );
  RETURN_ON_ERROR( getSocketHandle(&skt) );

  RETURN_ON_ERROR( copleyAmpInit( id ) );

  //RETURN_ON_ERROR( copleyHomeIsCurPos(id) );
  RETURN_ON_ERROR( copleyHomeByHomeSwitch(id, false, 500000) );
  
  int interval = 20;


  RETURN_ON_ERROR( copleyStartInterpPosMove(id, 2, interval) );
  usleep(interval*1000);


  int32_t targPos;
  int c = 100000; 
  float angle = 0;
  float dangle = 0.05;
  float maxAngle = 4.0*3.14159;
  
  uint32_t statusWord;
  uint32_t data;
  uint16_t numBuf;

  while (1) {
    targPos = (int)((cosf(angle)-1.0) *c);
    RETURN_ON_ERROR( copleySetInterpTargPos(id, targPos) );
    RETURN_ON_ERROR( readSdoData( skt, id, STATUS_WORD, 0, &statusWord, true ) ); 
    printf("status: 0x%x\n", statusWord & 0xffff);
    RETURN_ON_ERROR( readSdoData( skt, id, TRAJECTORY_BUFFER_FREE_COUNT, 0, &data, true ) ); 
    numBuf = 0x20 - (0x0000ffff & data);
    printf("numBuf: 0x%x\n", numBuf);
    
    angle = angle+dangle;
    usleep(interval*1000); 
 
    // test restart
    if (angle > maxAngle) {
      sleep(1); // variant 1, buffer underflow
      //RETURN_ON_ERROR(copleyStopInterpPosMove(id)); // variant 2 commanded stop
      RETURN_ON_ERROR(copleyQuickStop(id)); // variant 3 commanded quick stop
      return 0;
      angle = angle - maxAngle;
      printf("Stopped....\n");
      RETURN_ON_ERROR( copleyStartInterpPosMove(id, 2, interval) );
      usleep(interval*1000); 
    }
       
  }
}


