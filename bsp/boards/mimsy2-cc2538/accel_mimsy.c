#include "i2c_mimsy.h"
#include "i2c.h"s
#include "MPU9250_RegisterMap.h"
#include "flash_mimsy.h" //TODO: mive imu_data type to a new mimsy.h file
#include "gptimer.h"
#include "hw_gptimer.h"
#include "hw_memmap.h"

//invensense related includes
/*
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
*/
//defines

#define DEFAULT_MPU_HZ  (20)
#define IMU_ADDRESS 0x69
union IMURaw {
  
  uint16_t words[7];
  uint8_t bytes[14];
  
  
};

//invensense related structures################################################

struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};



//functions ####################################################################
//TODO: add an imu init function
//mimsy init function; inits board timers, resets imu, wakes imu, sets clock source
//enables sensors

//phony callback function for dmp
static void tap_cb(unsigned char direction, unsigned char count)
{

    return;
}

//phony callback function for dmp
static void android_orient_cb(unsigned char orientation)
{
	switch (orientation) {

	default:
		return;
	}
}




void mimsyIMUInit(){
    board_timer_init();
    uint8_t readbyte;
    
    
         i2c_init();
    uint8_t address;
    address=0x69;
    
     i2c_write_byte(address,MPU9250_PWR_MGMT_1); //reset
     i2c_write_byte(address,0x80);
    
      i2c_write_byte(address,MPU9250_PWR_MGMT_1); //wake
     i2c_write_byte(address,0x00);
    
    uint8_t bytes[2]={MPU9250_PWR_MGMT_1,0x01}  ; 
     i2c_write_bytes(address,bytes,2); //set gyro clock source
   

     bytes[0]=0x6C;
     bytes[1]=0x03;
        uint8_t *byteptr=&readbyte;
      
        i2c_write_byte(address,MPU9250_PWR_MGMT_2);
     i2c_read_byte(address,byteptr);
     
     i2c_write_byte(address,MPU9250_PWR_MGMT_2); //sens enable
     i2c_write_byte(address,0x00);
     
     i2c_write_byte(address,MPU9250_PWR_MGMT_2);
     i2c_read_byte(address,byteptr);
  
  
}
/*
void mimsySetAccelFsr(int fsr){
   mpu_set_accel_fsr(fsr);
}

void mimsySetGyroFsr(int fsr){
   mpu_set_gyro_fsr(fsr);
}
*/
//gyro reads based on invensense drivers
void mimsyIMURead6DofInv(IMUData *data){
  
}
//TODO: start adding invensense driver-based functions

//reads IMU data from Mimsy's MPU9250 
void mimsyIMURead6Dof( IMUData *data){
  uint8_t address=IMU_ADDRESS;
  uint8_t readbyte;  
  uint8_t *byteptr=&readbyte;

  //Accel X
      i2c_read_register(address,MPU9250_ACCEL_XOUT_H,byteptr);
     (*data).fields.accelX=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_XOUT_L,byteptr);
      (*data).fields.accelX=((uint16_t) readbyte)| (*data).fields.accelX;
      
        //Accel Y
      i2c_read_register(address,MPU9250_ACCEL_YOUT_H,byteptr);
     (*data).fields.accelY=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_YOUT_L,byteptr);
      (*data).fields.accelY=((uint16_t) readbyte)| (*data).fields.accelY;
      
        //Accel Z
      i2c_read_register(address,MPU9250_ACCEL_ZOUT_H,byteptr);
     (*data).fields.accelZ=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_ACCEL_ZOUT_L,byteptr);
      (*data).fields.accelZ=((uint16_t) readbyte)| (*data).fields.accelZ;
      
        //Gyro X
      i2c_read_register(address,MPU9250_GYRO_XOUT_H,byteptr);
     (*data).fields.gyroX=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_XOUT_L,byteptr);
      (*data).fields.gyroX=((uint16_t) readbyte)| (*data).fields.gyroX;
      
        //Gyro Y
      i2c_read_register(address,MPU9250_GYRO_YOUT_H,byteptr);
    (*data).fields.gyroY=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_YOUT_L,byteptr);
      (*data).fields.gyroY=((uint16_t) readbyte)| (*data).fields.gyroY;
      
        //Gyro Z
      i2c_read_register(address,MPU9250_GYRO_ZOUT_H,byteptr);
     (*data).fields.gyroZ=((uint16_t) readbyte)<<8;
    
    i2c_read_register(address,MPU9250_GYRO_ZOUT_L,byteptr);
      (*data).fields.gyroZ=((uint16_t) readbyte)| (*data).fields.gyroZ;
       
      
      (*data).fields.timestamp= TimerValueGet(GPTIMER2_BASE, GPTIMER_A);
}   
   /*
void mimsyDmpBegin(){
      dmp_load_motion_driver_firmware();
    dmp_set_orientation(
       inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
          dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    
        hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    dmp_enable_6x_lp_quat(1);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
}*/
