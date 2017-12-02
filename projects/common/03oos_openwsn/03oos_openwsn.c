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
IMUData data;


int mote_main(void) {
   
   // initialize
   board_init();
   scheduler_init();
   openstack_init();
   
   // indicate
   //init imu TODO: add to board init function
   mimsyIMUInit();
   mimsyIMURead6Dof(&data);
   mimsyIMURead6Dof(&data);
  // while(1){
	   //mimsyIMURead6Dof(&data);

 //  }
   

   // start
   scheduler_start();
   return 0; // this line should never be reached
}

void sniffer_setListeningChannel(uint8_t channel){return;}
