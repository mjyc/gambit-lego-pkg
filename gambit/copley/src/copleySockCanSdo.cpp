#include <ros/console.h>

#include "copley/copleySockCanSdo.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

 
/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#define RX_COBID_BASE 0x0600 
#define RX_COBID_MASK 0xff80 //includes 0x0600 - 0x067f

#define TX_COBID_BASE 0x0580 
#define TX_COBID_MASK 0xff80 //includes 0x0580 - 0x05ff

#define NMT_COBID      0x0000 
#define NMT_COBID_MASK 0xFFFF // only 0x0000

#define HEARTBEAT_COBID_BASE 0x0700 

#define PDO_COBID_BASE 0x0080 
#define PDO_COBID_MASK 0xF8FF // includes 0x0[1-7]



// Private functions 

void dumpCanFrame( struct can_frame* pFrame ) {
  int i;
  printf("**** Can frame dump *****\n");
  printf("id: 0x%x\n", pFrame->can_id);
  printf("dlc: 0x%x\n", pFrame->can_dlc);
  printf("data: ");
  for (i=0; i<7; i++) {
    printf("0x%x ", pFrame->data[i]);
  }
  printf("\n");
}

void emcyReceived(struct can_frame* pFrame) {
    uint16_t status = pFrame->data[0] | (pFrame->data[1] << 8);
    // Error descriptions from Copley CANopen Programmer's Manual p41
    switch(status) {
        case 0x0000: ROS_ERROR("CAN EMCY 0x%04X: No error/error status cleared", pFrame->can_id); break;
        case 0x2280: ROS_ERROR("CAN EMCY 0x%04X: Encoder feedback error", pFrame->can_id); break;
        case 0x2310: ROS_ERROR("CAN EMCY 0x%04X: Current limited", pFrame->can_id); break;
        case 0x2320: ROS_ERROR("CAN EMCY 0x%04X: Short circuit", pFrame->can_id); break;
        case 0x3110: ROS_ERROR("CAN EMCY 0x%04X: Mains over voltage", pFrame->can_id); break;
        case 0x3120: ROS_ERROR("CAN EMCY 0x%04X: Mains under voltage", pFrame->can_id); break;
        case 0x3310: ROS_ERROR("CAN EMCY 0x%04X: Output voltage limited", pFrame->can_id); break;
        case 0x4210: ROS_ERROR("CAN EMCY 0x%04X: Amplifier over temperature", pFrame->can_id); break;
        case 0x4300: ROS_ERROR("CAN EMCY 0x%04X: Motor temperature sensor", pFrame->can_id); break;
        case 0x5080: ROS_ERROR("CAN EMCY 0x%04X: Amplifier error", pFrame->can_id); break;
        case 0x7122: ROS_ERROR("CAN EMCY 0x%04X: Phasing error", pFrame->can_id); break;
        case 0x7380: ROS_ERROR("CAN EMCY 0x%04X: Positive limit switch", pFrame->can_id); break;
        case 0x7381: ROS_ERROR("CAN EMCY 0x%04X: Negative limit switch", pFrame->can_id); break;
        case 0x7390: ROS_ERROR("CAN EMCY 0x%04X: Tracking error", pFrame->can_id); break;
        case 0x73A0: ROS_ERROR("CAN EMCY 0x%04X: Position wrapped", pFrame->can_id); break;
        case 0x8130: ROS_ERROR("CAN EMCY 0x%04X: Node guarding error or heartbeat error", pFrame->can_id); break;
        default: ROS_ERROR("CAN EMCY 0x%04X: Unknown error", pFrame->can_id); break;
    }
}

int readCanFrame( int skt, uint16_t expectedCobId, struct can_frame* pFrame, bool bRigidAssumptions ) {
  // keep reading until we either get the expected cob-id, or we time out
  int err;
  int  bytes_read;
  uint16_t cobId = 0xffff;
  while(cobId != expectedCobId) {
    bytes_read = read( skt, pFrame, sizeof(struct can_frame) );
    if (bytes_read != sizeof(struct can_frame)) {
      // this is likely due to a timeout
      #ifdef DEBUG
      char errorString[2048];
      sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
      perror(errorString);
      #endif
      return ERROR_TIMEOUT;
    }

    cobId = pFrame->can_id;
    if( (cobId & 0xFFF0) == 0x0080) {
        emcyReceived(pFrame);
    }

    if (cobId != expectedCobId) {
      #ifdef DEBUG
      printf("Unexpected cobId: 0x%x\n", cobId); 
      dumpCanFrame(pFrame);
      #endif
      // TODO perhaps we should queue the unexpected cobid frames instead of just throwing them away
      if (bRigidAssumptions) {
        return ERROR_UNEXPECTED_COBID;
      }
    }
  }
  return ERROR_NONE;
}


// public functions (as declared in header)


int readSdoData( int skt, uint8_t id, uint16_t index, uint8_t subindex, uint32_t* pReadData, bool bRigidAssumptions ) { 
   // this is not general, but it works with the Copley Amp 
   int  err;

   struct can_frame requestFrame;
   requestFrame.can_id = RX_COBID_BASE+id;
   requestFrame.can_dlc = 8;
   requestFrame.data[0] = 0x40; // css = 2
   requestFrame.data[1] = (uint8_t)(index & 0x00FF); // index, lower byte
   requestFrame.data[2] = (uint8_t)(index>>8); // index, upper byte
   requestFrame.data[3] = subindex;
   requestFrame.data[4] = 0x00;
   requestFrame.data[5] = 0x00;
   requestFrame.data[6] = 0x00;
   requestFrame.data[7] = 0x00;
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }

   // now read the result
   struct can_frame resultFrame;
   RETURN_ON_ERROR( readCanFrame( skt, TX_COBID_BASE+id, &resultFrame, bRigidAssumptions ) );
   /*
   err =  readCanFrame( skt, TX_COBID_BASE+id, &resultFrame, bRigidAssumptions );
   if (err) {
     #ifdef DEBUG
     printf("error: %d\n", __LINE__);
     #endif
     return err;
   }
   */
   uint8_t scsByte = resultFrame.data[0];
   // expect bits 7-5 to be 2 (initiate upload response)
   if ((scsByte >> 5) != 0x02) {
     #ifdef DEBUG
     printf("ERROR %s:%u scs != 2\n", __FILE__, __LINE__);
     #endif
     return ERROR_BAD_SCS;
   }
   uint8_t bExpeditedTransfer = scsByte & 0x02;
   // expect expedited transfer
   if ( !bExpeditedTransfer ) {
     #ifdef DEBUG
     printf("ERROR %s:%u only expedited transfers allowed\n", __FILE__, __LINE__);
     #endif
     return ERROR_NOT_EXPEDITED;
   }
   uint8_t sizeIndicated = scsByte & 0x01;
   uint8_t nonDataSize = (scsByte&0x06) >> 2;
   uint8_t len;
   if (sizeIndicated) {
     len = 4 - nonDataSize;
   } else {
     len = 4;
   }
   // expect the result index/subindex to match the request
   uint16_t resultIndex = *((uint16_t*)(&resultFrame.data[1]));
   if ( resultIndex != index ) {
     #ifdef DEBUG
     printf("ERROR %s:%u resultIndex(0x%x) != index(0x%x)\n", __FILE__, __LINE__, resultIndex, index);
     #endif
     return ERROR_BAD_INDEX;
   }
   uint16_t resultSubindex = resultFrame.data[3];
   if ( resultSubindex != subindex ) {
     #ifdef DEBUG
     printf("ERROR %s:%u resultSubindex(0x%x) != subindex(0x%x)\n", __FILE__, __LINE__, resultSubindex, subindex);
     #endif
     return ERROR_BAD_SUBINDEX;
   }
   memcpy(pReadData, &(resultFrame.data[4]), 4);
   
   return ERROR_NONE;
}

int writeSdoData( int skt, uint8_t id, uint16_t index, uint8_t subindex, uint8_t len, uint32_t writeData, bool bRigidAssumptions ) { 

   // this is not general, but it works with the Copley Amp 
   int  err;

   struct can_frame requestFrame;
   requestFrame.can_id = RX_COBID_BASE+id;
   requestFrame.can_dlc = 8;

   uint8_t ccsByte;
   // bits 7-5 are 1 (initiate download request)
   ccsByte = 0x20;
   // e: expedited transfer
   ccsByte |= 0x02;
   // s: data size is indicated
   ccsByte |= 0x01;
   // n: ( 1 <= n <= 4 )
   if (len < 1) {
     #ifdef DEBUG
     printf("ERROR %s:%u size < 1\n", __FILE__, __LINE__);
     #endif
     return ERROR_BAD_CCS;
   }
   if (len > 4) {
     #ifdef DEBUG
     printf("ERROR %s:%u size > 4\n", __FILE__, __LINE__);
     #endif
     return ERROR_BAD_LEN; 
   }
   ccsByte |= ((4-len)<<2);

   requestFrame.data[0] = ccsByte;
   requestFrame.data[1] = (uint8_t)(index & 0x00FF); // index, lower byte
   requestFrame.data[2] = (uint8_t)(index>>8); // index, upper byte
   requestFrame.data[3] = subindex;
   memcpy(&(requestFrame.data[4]), (uint8_t*)&writeData, 4);
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }

   // now read the confirm 
   struct can_frame confirmFrame;
   RETURN_ON_ERROR( readCanFrame( skt, TX_COBID_BASE+id, &confirmFrame, bRigidAssumptions ) );
   uint8_t scsByte = confirmFrame.data[0];
   // expect bits 7-5 to be 3 (initiate download response)
   if (scsByte >> 5 != 0x03) {
     #ifdef DEBUG
     printf("ERROR %s:%u scs = 0x%x != 3\n",  __FILE__, __LINE__, scsByte >> 5);
     
     #endif
     return ERROR_BAD_SCS;
   }
   // expect the confirm index/subindex to match the request
   uint16_t confirmIndex = *((uint16_t*)(&confirmFrame.data[1]));
   if ( confirmIndex != index ) {
     #ifdef DEBUG
     printf("ERROR %s:%u confirmIndex(0x%x) != index(0x%x)\n", __FILE__, __LINE__, confirmIndex, index);
     #endif
     return ERROR_BAD_INDEX;
   }
   uint16_t confirmSubindex = confirmFrame.data[3];
   if ( confirmSubindex != subindex ) {
     #ifdef DEBUG
     printf("ERROR %s:%u confirmSubindex(0x%x) != subindex(0x%x)\n", __FILE__, __LINE__, confirmSubindex, subindex);
     #endif
     return ERROR_BAD_SUBINDEX;
   }
   
   return ERROR_NONE;

}


int flushCanReadBuffer( int skt ) {
  //read till there is nothing left to read
  struct can_frame flushFrame;
  int  bytes_read = sizeof(struct can_frame);
  while(bytes_read == sizeof(struct can_frame)) {
    bytes_read = read( skt, &flushFrame, sizeof(struct can_frame) );
  }
  return ERROR_NONE;
}


int startNode( int skt, uint8_t id ) { 

   // this is not general, but it works with the Copley Amp 
   int  bytes_read;


   struct can_frame requestFrame;
   requestFrame.can_id = NMT_COBID;
   requestFrame.can_dlc = 2;
   requestFrame.data[0] = 0x01; // start
   requestFrame.data[1] = id; 
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }
   return ERROR_NONE;
}

int stopNode( int skt, uint8_t id ) { 

   // this is not general, but it works with the Copley Amp 
   int  bytes_read;


   struct can_frame requestFrame;
   requestFrame.can_id = NMT_COBID;
   requestFrame.can_dlc = 2;
   requestFrame.data[0] = 0x02; // start
   requestFrame.data[1] = id; 
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }
   return ERROR_NONE;
}

int resetNode( int skt, uint8_t id ) { 

   // this is not general, but it works with the Copley Amp 
   int  err;

   struct can_frame requestFrame;
   requestFrame.can_id = NMT_COBID;
   requestFrame.can_dlc = 2;
   requestFrame.data[0] = 0x81; // reset (pg 67)
   requestFrame.data[1] = id; 
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }

   sleep(1);

   // now read the heatbeat 
   struct can_frame heartFrame;
   err =  readCanFrame( skt, HEARTBEAT_COBID_BASE+id, &heartFrame, true );
   if (err) {
     return err;
   }
   return ERROR_NONE;
}

int syncNode( int skt) { 

   // this is not general, but it works with the Copley Amp 
   struct can_frame requestFrame;
   requestFrame.can_id = PDO_COBID_BASE; 
   requestFrame.can_dlc = 0;
   requestFrame.data[1] = 0; 
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }
   return ERROR_NONE;
}

int writePdoData(int skt, uint16_t cobid, uint8_t len, uint32_t writeData1, uint32_t writeData2) {

   // this is not general, but it works with the Copley Amp 
   struct can_frame requestFrame;
   requestFrame.can_id = cobid; 
   requestFrame.can_dlc = len;
   memcpy(&(requestFrame.data[0]), (uint8_t*)&writeData1, 4);
   memcpy(&(requestFrame.data[4]), (uint8_t*)&writeData2, 4);
   int bytes_sent = write( skt, &requestFrame, sizeof(struct can_frame) ); // this blocks
   // expect bytes_sent == sizeof(frame)
   if (bytes_sent != sizeof(struct can_frame)) {
     // this is likely due to a timeout
     #ifdef DEBUG
     char errorString[80];
     sprintf(errorString, "ERROR %s: %s(): %u", __FILE__,  __func__,  __LINE__);
     perror(errorString);
     #endif
     return ERROR_TIMEOUT;
   }
   return ERROR_NONE;
}
