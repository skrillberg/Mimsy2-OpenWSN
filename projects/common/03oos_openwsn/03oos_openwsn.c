/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"
#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "servo.c"
IMUData data;


int mote_main(void) {
   
   // initialize
   board_init();
//   scheduler_init();
//   openstack_init();
   
   // indicate
   //init imu TODO: add to board init function
   mimsyIMUInit();
   mimsyIMURead6Dof(&data);
   mimsyIMURead6Dof(&data);
  // while(1){
	   //mimsyIMURead6Dof(&data);

 //  }
   
   servo_init(3,20,2,1);
   int cnt=0;
   while(1){
	   cnt++;
	   if(cnt==5000000){
		   servo_rotate_time(1);
	   }
	   if(cnt==10000000){
		   cnt=0;
		   servo_rotate_time(2);
	   }
   }

   // start
   scheduler_start();
   return 0; // this line should never be reached
}

void sniffer_setListeningChannel(uint8_t channel){return;}
