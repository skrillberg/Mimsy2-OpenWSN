#ifndef __XL_MIMSY_H__
#define __XL_MIMSY_H__

#include "flash_mimsy.h"
extern void mimsyIMURead6Dof(IMUData *data);
void mimsyIMUInit(void);
void mimsyIMURead6DofInv(IMUData *data);
void mimsyDmpBegin(void);
void mimsySetAccelFsr(int fsr);
void mimsySetGyroFsr(int fsr);

#endif
