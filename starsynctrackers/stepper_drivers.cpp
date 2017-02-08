
#include "stepper_drivers.h"
#include "starsynctrackers.h"

extern SSTVARS sstvars;

boolean reset_started = false;

static const int STEPPER_DIR_PIN = 3;
static const int STEPPER_STEP_PIN = 2;
static const int STEPPER_RESET = 4;
static const int STEPPER_MS1 = 5;
static const int STEPPER_MS2 = 6;
static const int STEPPER_MS3 = 7;

AccelStepper Astepper1(1, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

void stepper_init() {
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_RESET, OUTPUT);
  pinMode(STEPPER_MS1, OUTPUT);  
  pinMode(STEPPER_MS2, OUTPUT);  
  pinMode(STEPPER_MS3, OUTPUT);  
  digitalWrite(STEPPER_RESET, HIGH);
  digitalWrite(STEPPER_MS1, HIGH);
  digitalWrite(STEPPER_MS2, HIGH);
  digitalWrite(STEPPER_MS3, HIGH);
}

int reset_lp_loop = 0;
void stepper_reset_lp() {
  if (!reset_started) {
    reset_started = true;
    if (sstvars.dir > 0) {
      digitalWrite(STEPPER_DIR_PIN, LOW);
    } else {
      digitalWrite(STEPPER_DIR_PIN, HIGH);
    }
  }
  reset_lp_loop = 0;
  while(reset_lp_loop < 1) {
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(75);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(75);
    reset_lp_loop++;
  }
}

void stepper_reset_done() {
    if (sstvars.dir > 0) {
      digitalWrite(STEPPER_DIR_PIN, HIGH);
    } else {
      digitalWrite(STEPPER_DIR_PIN, LOW);
    }
}

void fast(long counts) {
  
}

