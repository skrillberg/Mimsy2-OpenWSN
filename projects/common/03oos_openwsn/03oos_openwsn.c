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
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
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
   
   servo_init(3,20,1.45);

   mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO|INV_XYZ_COMPASS); //turn on sensor
   mpu_set_accel_fsr(16); //set fsr for accel

   mimsyDmpBegin();

   short gyro[3];
   short accel[3];
   long quat[4];
   long rot[3];
   long timestamp2;
   unsigned char more;
   short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;


   int cnt=0;
   while(1){
	      dmp_read_fifo(gyro, accel, quat,&timestamp2, &sensors, &more);
	     // mpu_get_accel_reg(xl,&debugx);
	   cnt++;
	   if(cnt==5000000){
		   servo_rotate_time(0.9);
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
