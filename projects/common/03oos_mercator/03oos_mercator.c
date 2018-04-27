/**
\brief Mercator firmware, see https://github.com/openwsn-berkeley/mercator/.

\author Constanza Perez Garcia <constanza.perezgarcia@gmail.com>, November 2014.
\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, November 2014.
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
#include "uart_mimsy.h"
#include "gpio.h"
#include "headers/hw_memmap.h"
#include "ioc.h"
#include "led_mimsy.h"
#include "inchworm.h"
//=========================== initialization ==================================

int mote_main(void) {
   InchwormMotor iw={GPIO_D_BASE,GPIO_D_BASE,GPIO_PIN_1,GPIO_PIN_2,1};
   InchwormSetup setup={iw,1,500,80,1,2,3};
   
   board_init();
   inchwormInit(setup);
   while(1){
   inchwormFreerun(iw);
   for(int i=0;i<10000;i++){}
   inchwormRelease(iw);
   for(int i=0;i<10000;i++){}
   }

}


