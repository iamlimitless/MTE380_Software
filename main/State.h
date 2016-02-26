#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include "Motor.h"

#ifdef __cplusplus
extern "C" {
#endif

volatile bool WAIT_FOR_INTERRUPT;

void SetupState();

void DriveUpRamp(MotorDrive* motors);


#ifdef __cplusplus
}
#endif

#endif //STATE_H