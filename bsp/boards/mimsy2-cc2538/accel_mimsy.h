#include "flash_mimsy.h"
extern void mimsyIMURead6Dof(IMUData *data);
void mimsyIMUInit();
void mimsyIMURead6DofInv(IMUData *data);
void mimsyDmpBegin();
void mimsySetAccelFsr(int fsr);
void mimsySetGyroFsr(int fsr);