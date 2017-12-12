/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/
#include <math.h>
#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"
#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "servo.c"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "uart_mimsy.h"
IMUData data;

long vec[3];

//extern void inv_q_rotate(const long *q, const long *in, long *out);
int mote_main(void) {
   
	int gyro_fsr = 2000;
   // initialize
   board_init();
//   scheduler_init();
//   openstack_init();
   
   uartMimsyInit();
   // indicate
   //init imu TODO: add to board init function
   mimsyIMUInit();
   mimsyIMURead6Dof(&data);
   mimsyIMURead6Dof(&data);
  // while(1){
	   //mimsyIMURead6Dof(&data);

 //  }
   
   servo_init(3,20,1.45);

   mpu_set_sensors(INV_XYZ_ACCEL|INV_XYZ_GYRO); //turn on sensor
   mpu_set_accel_fsr(16); //set fsr for accel
   mpu_set_gyro_fsr(gyro_fsr); //set fsr for accel

   mimsyDmpBegin();

   short gyro[3];
   short accel[3];
   long quat[4];
   //long rot[3];
   long timestamp2;
   unsigned char more;
   short sensors=INV_XYZ_GYRO | INV_WXYZ_QUAT|INV_XYZ_ACCEL;
   long in[3] = {0,0,0x40000000};
   float fvec[3];
   long rot[9];
   int cnt=0;
   float mag;
   float servo_time_0;
   float servo_time_1;
   float xdif;
   float ydif;
   float zdif;
   long xvec[3] = {0x40000000,0,0};
   float xref[3] = {1,0,0};
   long xrot[3];
   float xfloat[3];
   float pitch;
   float fquats[4];
   float roll;
   float rollbias=0.5;
   float pitchbias=1;
   while(1){
	      dmp_read_fifo(gyro, accel, quat,&timestamp2, &sensors, &more);
	      alt_inv_q_rotate(quat,in,vec);
	      fvec[0]=(float)vec[0]/(float)0x40000000;
	      fvec[1]=(float)vec[1]/(float)0x40000000;
	      fvec[2]=(float)vec[2]/(float)0x40000000;

	      //alt_inv_quaternion_to_rotation(quat,rot);
	      //alt_mlMatrixVectorMult(rot, in, vec);
	     // mpu_get_accel_reg(xl,&debugx);
	   cnt++;
	   if(cnt%10==0){
		      //mimsyPrintf("\n Quaternions:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",quat[0],quat[1],quat[2],quat[3],accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2]);

		   	   //pitch = 2*(quat[0]*quat[2]-quat[3]*quat[1]);
		       mimsyPrintf("\n Orientation Vector: \%011d, \%011d, \%011d, ",xrot[0],xrot[1],xrot[2]);
	   }
	   if(cnt%5==0){
		   	  alt_inv_q_rotate(quat,xvec,xrot);
		      xfloat[0]=(float)xrot[0]/(float)0x40000000;
		      xfloat[1]=(float)xrot[1]/(float)0x40000000;
		      xfloat[2]=(float)xrot[2]/(float)0x40000000;
		      //xrot is the direction in global coordinates of where the x body axis is pointing

		   //mag = fvec[0]*fvec[0]+fvec[1]*fvec[1];
		   ydif= xfloat[1]-xref[1];
		   zdif = xfloat[2]-xref[2];

		   //pitch control
		   fquats[0]=(float)quat[0]/(float)0x40000000;
		   fquats[1]=(float)quat[1]/(float)0x40000000;
		   fquats[2]=(float)quat[2]/(float)0x40000000;
		   fquats[3]=(float)quat[3]/(float)0x40000000;

		   pitch = 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1]); //computes sin of pitch
		   servo_time_0 = 1.45+pitch/2 * pitchbias;
		   servo_time_1= 1.45-pitch/2 * pitchbias;

		   //servo_time_0 = 1.45;
		   //servo_time_1= 1.45;
		   //roll control
		   roll = 2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) / (1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2]));
		   servo_time_0 = servo_time_0 + roll/2 * rollbias;
		   servo_time_1 = servo_time_1 +roll/2 * rollbias;


		   servo_rotate_time(servo_time_0,0);
		   servo_rotate_time(servo_time_1,1);


	   }
	   if(cnt==10000){
		   cnt=0;
		   //servo_rotate_time(2);
	   }
   }

   // start
   scheduler_start();
   return 0; // this line should never be reached
}

void sniffer_setListeningChannel(uint8_t channel){return;}
