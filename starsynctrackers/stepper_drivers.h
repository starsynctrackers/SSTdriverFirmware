#ifndef __STEPPER_DRIVERS_H
#define __STEPPER_DRIVERS_H

#define MICROSTEPS 16

void stepperInit(void);
void setSpeed(float speed);
float getSpeed(void);
long getPosition(void);
void setPosition(long position);
void runStepper(void);

#endif
