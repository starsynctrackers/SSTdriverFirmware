#ifndef __STEPPER_DRIVERS_H
#define __STEPPER_DRIVERS_H

#include <AccelStepper.h>

//
// Need AccelStepper fork with AFMotor support with library 
//   https://github.com/adafruit/AccelStepper

#define MICROSTEPS 16

extern AccelStepper Astepper1;
void stepper_init(void);
void stepper_reset_lp(void);
void stepper_reset_done(void);

#endif
