#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
 
/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif
 
int main(void) { 
   int i;

   uint8_t msg[8];

   /* Create the socket */
   int skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
   printf("socket created\n");
 
   /* Locate the interface you wish to use */
   struct ifreq ifr;
   strcpy(ifr.ifr_name, "can0");
   ioctl(skt, SIOCGIFINDEX, &ifr); /* ifr.ifr_ifindex gets filled 
                                  * with that device's index */
   printf("interface located\n");
 
   /* Select that CAN interface, and bind the socket to it. */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   bind( skt, (struct sockaddr*)&addr, sizeof(addr) );
   printf("socket bind\n");
 
   /* listen to the CAN bus */
   struct can_frame frame;
 
   while (1) {
   /* Read a message back from the CAN bus */
   printf("loop\n");
   int bytes_read = read( skt, &frame, sizeof(frame) );
   printf("id:0x%x dlc:%d\n", frame.can_id, frame.can_dlc);
   printf("data:\n");
   for (i=0; i<frame.can_dlc; i++) {
     printf("0x%x\n",frame.data[i]);

   }
   printf("\n");
   printf("bytes read: %d\n", bytes_read);
   }
   
}

