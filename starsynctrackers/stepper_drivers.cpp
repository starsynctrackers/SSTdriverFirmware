#include <Arduino.h>
#include "stepper_drivers.h"
#include "starsynctrackers.h"

#define MODE_FULL 1
#define MODE_SIXTEEN 16
#define MODE_FORWARD 1
#define MODE_BACKWARD -1 

extern SSTVARS sstvars;

static const int STEPPER_DIR_PIN = 3;
static const int STEPPER_STEP_PIN = 2;
static const int STEPPER_RESET = 4;
static const int STEPPER_MS1 = 5;
static const int STEPPER_MS2 = 6;
static const int STEPPER_MS3 = 7;

static int mode_steps = 9999;
static int mode_direction = 9999;
static long total_steps = 0;

static long SCounts = 0;
static float SSpeed = 0;
static float count = 0;
static unsigned long SClock = 0;

static void set_step_direction(boolean forward);
static void set_step_resolution(boolean full);
static void step(void);
static void stepper_backwardstep_fast(void);
static void stepper_backwardstep(void);
static void stepper_forwardstep_fast(void);
static void stepper_forwardstep(void);


/* First local functions. */

void set_step_direction(boolean forward) {
  if(forward) {
    if (sstvars.dir > 0) {
      digitalWrite(STEPPER_DIR_PIN, HIGH);
    } else {
      digitalWrite(STEPPER_DIR_PIN, LOW);
    }    
    mode_direction = MODE_FORWARD;
  } else {
    if (sstvars.dir > 0) {
      digitalWrite(STEPPER_DIR_PIN, LOW);
    } else {
      digitalWrite(STEPPER_DIR_PIN, HIGH);
    }
    mode_direction = MODE_BACKWARD;
  }
}


void set_step_resolution(boolean full) {
  if(full) {
    digitalWrite(STEPPER_MS1, LOW);
    digitalWrite(STEPPER_MS2, LOW);
    digitalWrite(STEPPER_MS3, LOW);
    mode_steps = MODE_FULL;
  } else {
    digitalWrite(STEPPER_MS1, HIGH);
    digitalWrite(STEPPER_MS2, HIGH);
    digitalWrite(STEPPER_MS3, HIGH);
    mode_steps = MODE_SIXTEEN;
  }
}


void step() {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(STEPPER_STEP_PIN, LOW);  
}


void stepper_backwardstep_fast() {
  if(mode_steps != MODE_FULL) {
    set_step_resolution(true);
  }
  if(mode_direction != MODE_BACKWARD) {
    set_step_direction(false);
  }
  step();
  total_steps -= MICROSTEPS;
}

void stepper_backwardstep() {
  if(mode_steps != MODE_SIXTEEN) {
    set_step_resolution(false);
  }
  if(mode_direction != MODE_BACKWARD) {
    set_step_direction(false);
  }
  step();
  total_steps--;
}


void stepper_forwardstep_fast() {
  if(mode_steps != MODE_FULL) {
    set_step_resolution(true);
  }
  if(mode_direction != MODE_FORWARD) {
    set_step_direction(true);
  }
  step();
  total_steps += MICROSTEPS;
}


void stepper_forwardstep() {
  if(mode_steps != MODE_SIXTEEN) {
    set_step_resolution(false);
  }
  if(mode_direction != MODE_FORWARD) {
    set_step_direction(true);
  }
  step();
  total_steps++;
}


/* Non-local functions below. */


void stepperInit() {
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_RESET, OUTPUT);
  pinMode(STEPPER_MS1, OUTPUT);  
  pinMode(STEPPER_MS2, OUTPUT);  
  pinMode(STEPPER_MS3, OUTPUT);  
  digitalWrite(STEPPER_RESET, HIGH);
  set_step_resolution(false);
  set_step_direction(true);
}


void setSpeed(float speed) {
  SSpeed = speed;
  SCounts = 0;
  SClock = millis();
}

float getSpeed() {
  return SSpeed;
}


long getPosition() {
  return total_steps;
}


void setPosition(long position) {
  total_steps = position;
}


void runStepper() {
  // count is how many steps we need.
  count = (SSpeed*((float)(millis() - SClock))/1000.0) - SCounts;
  //Serial.println(SSpeed);
  //Serial.println(SClock);
  //Serial.println(SCounts);
  //Serial.println(count);
  if(abs(SSpeed) > 250.0) {
    if (count < -MICROSTEPS) {
        stepper_backwardstep_fast();
        SCounts -= MICROSTEPS;
    } else if (count > MICROSTEPS) {
        stepper_forwardstep_fast();    
        SCounts += MICROSTEPS;
    }    
  } else {
    if (count < -1.0) {
        stepper_backwardstep();
        SCounts--;
    } else if (count > 1.0) {
        stepper_forwardstep();    
        SCounts++;
    }
  }
}
