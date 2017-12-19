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
#include "gpio.h"
#include "headers/hw_memmap.h"
#include "ioc.h"
#include "led_mimsy.h"

IMUData data;

long vec[3];

#define FLASH_PAGE_STORAGE_START              14
#define FLASH_PAGES_TOUSE                       10
#define PAGE_SIZE                2048
//extern void inv_q_rotate(const long *q, const long *in, long *out);

void alt_inv_q_norm4(float *q);
float quat_dot_product(float *quat,float *vec);
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
   float yref[3] = {0,1,0};
   float zref[3] = {0,0,1};
   float xfloat[3];
   float pitch;
   float fquats[4];
   float yaw;
   float roll;
   float q2_err;
   float q1_err;
   float rollbias=0;
   float pitchbias=5;
   int armed=0;
   int bufferCount=0;
   bool triggered=false;
   IMUData datapoint;
   IMUData data[128];
   IMUData flashData[128];
   uint32_t currentflashpage=14;
   uint32_t pagesWritten=0;
   bool stopLogging = false;
   IMUDataCard cards[100];
   IMUDataCard cards_stable[100];

   for(int i=0;i<100;i++){
     (cards_stable[i].page)=FLASH_PAGE_STORAGE_START+i;
   }

  //config arming gpio/////////////////////////////////////////

  // GPIOPinTypeGPIOInput(GPIO_A_BASE,GPIO_PIN_5);
   GPIODirModeSet(GPIO_A_BASE,GPIO_PIN_5,GPIO_DIR_MODE_IN);
   IOCPadConfigSet(GPIO_A_BASE,GPIO_PIN_5,IOC_OVERRIDE_PUE);  //set pull up




   while(1){

	      if(!triggered && armed){
	        if((datapoint.signedfields.accelY<-3*32768/16 ||datapoint.signedfields.accelY>3*32767/16)||(datapoint.signedfields.accelX<-3*32768/16 ||datapoint.signedfields.accelX>3*32767/16)  ||(datapoint.signedfields.accelZ<-3*32768/16 ||datapoint.signedfields.accelZ>3*32767/16) ){
	          mimsyLedSet(RED_LED);
	          triggered=true;
	      //    UARTprintf("\n Accel X: %d, Accel Y: %d, Accel Z: %d ",debug4.signedfields.accelX,debug4.signedfields.accelY,debug4.signedfields.accelZ);
	        }
	      }

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
		      //mimsyPrintf("\n Quaternions:%d,%d,%d,%d,%d,%d,%d",quat[0],quat[1],quat[2],quat[3],gyro[0],gyro[1],gyro[2]);

		   	   //pitch = 2*(quat[0]*quat[2]-quat[3]*quat[1]);
		       //mimsyPrintf("\n Orientation Vector: \%011d, \%011d, \%011d, ",xrot[0],xrot[1],xrot[2]);
		   	  mimsyPrintf("\n pitch: \%05d, yaw: \%05d,\%012d,\%012d,\%012d",(int)(servo_time_0*100),(int)(servo_time_1*100),gyro[0],gyro[1],gyro[2]);
	   }
	  // if(cnt%1==0){
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

		   //mag = sqrtf( fquats[1] * fquats[1] + fquats[2] * fquats[2] + fquats[3] * fquats[3]);
		   alt_inv_q_norm4(fquats);
		   pitch = asinf( 2*(fquats[0]*fquats[2]-fquats[3]*fquats[1])); //computes sin of pitch
		   servo_time_0 = 1.45+pitch/3.14/2 * pitchbias;
		   servo_time_1= 1.45-pitch/3.14/2 * pitchbias;

		   //q2_err = 3.14-acosf(quat_dot_product(fquats,yref)/mag);
		   //q1_err = quat_dot_product(fquats,zref)/mag;
		   //q1_err = atan2f(quat_dot_product(fquats,zref),quat_dot_product(fquats,xref));
		   //q2_err = atan2f(quat_dot_product(fquats,xref),quat_dot_product(fquats,yref));
		   //servo_time_0 = 1.45 + q1_err/2/3.14159;
		   //servo_time_1 = 1.45 -q1_err/2/3.14159;

		   //servo_time_0 = 1.45;
		   //servo_time_1= 1.45;

		   //gyro yaw

		   yaw = atan2f(2*(fquats[0] * fquats[3] + fquats[1] * fquats[2]),1 - 2*(fquats[2]*fquats[2] + fquats[3]*fquats[3]));

		   //roll control
		   //roll =  atan2f(2 * (fquats[0]*fquats[1] + fquats[2] * fquats[3]) ,(1 -2*(fquats[1] * fquats[1] +fquats[2]*fquats[2])));

		   // gyro 1 is body axis roll for rocket
	//	   q2_err = (float) (((float)(gyro[2]*2000))/32780);
		   servo_time_0 = servo_time_0 + yaw/3.14/2 * rollbias;
		   servo_time_1 = servo_time_1 +yaw/3.14/2 * rollbias;
		//   servo_time_0 = servo_time_0 + q2_err/500/2 * rollbias;
		 //  servo_time_1 = servo_time_1 +q2_err/500/2 * rollbias;
		   if(servo_time_0<1.0){
			   servo_time_0=1.0;
		   }
		   if(servo_time_0>1.87){
			   servo_time_0=1.87;
		   }
		   if(servo_time_1<1.0){
			   servo_time_1=1.0;
		   }
		   if(servo_time_1>1.87){
			   servo_time_1=1.87;
		   }

		   servo_rotate_time(servo_time_0,0);
		   servo_rotate_time(servo_time_1,1);


	  // }
	   if(cnt==10000){
		   cnt=0;
		   //servo_rotate_time(2);
	   }

	   ///////////////////////////////////datalogging
	   if(armed){
	      if(bufferCount==128){
	        if(!triggered){
	          //UARTprintf("%c[2K",27);
	        //  UARTprintf("\n Accel X: %d, Accel Y: %d, Accel Z: %d ",debug4.signedfields.accelX,debug4.signedfields.accelY,debug4.signedfields.accelZ);
	        //  UARTprintf(" Gyro X: %d, Gyro Y: %d, Gyro Z: %d ",debug4.signedfields.gyroX,debug4.signedfields.gyroY,debug4.signedfields.gyroZ);
	         // UARTprintf(", Timestamp: %x",debug4.fields.timestamp);
	          mimsyPrintf("\n Quaternions:%d,%d,%d,%d,%d,%d,%d",quat[0],quat[1],quat[2],quat[3],datapoint.signedfields.accelX,datapoint.signedfields.accelY,datapoint.signedfields.accelZ);
	        }
	        bufferCount=0;

	        if(triggered && !stopLogging){
	          IMUDataCard *card=&(cards[pagesWritten]);
	          flashWriteIMU(data,128,currentflashpage,card);

	          pagesWritten++;
	          currentflashpage++;
	        }
	      }

	      if(pagesWritten==FLASH_PAGES_TOUSE&&!stopLogging){
	        stopLogging=true;
	        mimsyLedClear(GREEN_LED);
	      }
	   }
	      // if logging mode is off, mimsy will loop a serial output of the data; it should also do this if it isn't armed
	      while(stopLogging || (!armed)){

	    	 UARTprintf("\n data starts here:+ \n"); //+ is start condition
	        for(int cardindex=0;cardindex<FLASH_PAGES_TOUSE;cardindex++){
	          IMUData sendData[128];
	          flashReadIMU(cards_stable[cardindex],sendData,128);

	          //loop through each data point
	          for(int dataindex=0;dataindex<128;dataindex++){



	              //print csv data to serial
	              //format: xl_x,xl_y,xl_z,gyrox,gyroy,gyroz,timestamp
	            UARTprintf("%d,%d,%d,%d,%d,%d,%x,%d,%d \n",
	                          sendData[dataindex].signedfields.accelX,
	                          sendData[dataindex].signedfields.accelY,
	                          sendData[dataindex].signedfields.accelZ,
	                          sendData[dataindex].signedfields.gyroX,
	                          sendData[dataindex].signedfields.gyroY,
	                          sendData[dataindex].signedfields.gyroZ,
	                          sendData[dataindex].fields.timestamp,
	                          cardindex,
	                          dataindex);


	          }

	        }
	        UARTprintf("= \n data ends here\n"); //= is end
	          for(int ui32Loop=1;ui32Loop<500000;ui32Loop++) {
	          }

	   	   if ((armed==0) && (GPIOPinRead(GPIO_A_BASE,GPIO_PIN_5) == 0) ){
	   		   armed = 1;
	   		   mimsyLedSet(GREEN_LED);
	   	   }

	        }

   }

   // start
   scheduler_start();
   return 0; // this line should never be reached
}

void sniffer_setListeningChannel(uint8_t channel){return;}

void alt_inv_q_norm4(float *q)
{
    float mag;
    mag = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (mag) {
        q[0] /= mag;
        q[1] /= mag;
        q[2] /= mag;
        q[3] /= mag;
    } else {
        q[0] = 1.f;
        q[1] = 0.f;
        q[2] = 0.f;
        q[3] = 0.f;
    }
}

float quat_dot_product(float *quat,float *vec){

	float dot =  quat[1] * vec[0] + quat[2] * vec[1] + quat[3] * vec[2];

	return dot;

}

static void armed_handler(){


}
