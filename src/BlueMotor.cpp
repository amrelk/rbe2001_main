#include "BlueMotor.h"
#include <RBE1001Lib.h>

#define PIDINT 100 //TODO: reduce pid interval
#define PWM 13 // motor driver pins
#define AIN2 14
#define AIN1 15
#define ENCA 18
#define ENCB 19

long enCount = 0;
long pidTarget = 0;
long err = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderIsr() {
  if (digitalRead(ENCB)) {
    portENTER_CRITICAL_ISR(&mux);
    enCount++;
    portEXIT_CRITICAL_ISR(&mux);
  } else {
    portENTER_CRITICAL_ISR(&mux);
    enCount--;
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void pidfTask(void* params) {
  BlueMotor* motor = (BlueMotor*) params; // we passed a pointer to `this` into the task so that we can get count and apply effort
  TickType_t xLastWakeTime = xTaskGetTickCount(); // initialize wake time variable
  long lastErr, count = motor->getCount(); // more initialization
  float effort = 0;
  float P, I, D, F;
  for(;;) { // loop forever - delay happens later
    lastErr = err;
    count = motor->getCount();
    err = pidTarget - count;
    P = motor->Kp * err;
    I = 0; // I term is 0 because it seems unnecessary so far
    D = (motor->Kd * (err - lastErr))/PIDINT;
    F = motor->Ff(count); //feedforward!! we probably won't use this
    effort = constrain(P + I - D + F, -255, 255); // constrain the pidf result to [-255, 255]
    motor->setEffort((int)effort); // send constrained pidf result to the motor
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PIDINT)); // delay for PIDINT
  }
}

BlueMotor::BlueMotor() {
  pinMode(ENCA, INPUT); // initialize motor driver pins
  pinMode(ENCB, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  pinMode(PWM, OUTPUT);
  attachInterrupt(ENCA, encoderIsr, RISING);
}

BlueMotor::~BlueMotor() {
}

void BlueMotor::startPid() {
  pidTarget = getCount(); // set pid target to current count so no violent movement
  xTaskCreate(pidfTask, "BlueMotor PID Thread", 1000, (void*) this, 1, NULL); // start the pidf task
}

void BlueMotor::setEffort(int effort) {
  if (effort >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    effort = -effort; // make effort positive because we just set the pins to backwards
  }
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
}

void BlueMotor::setTarget(long count) {
  pidTarget = count;
}

float BlueMotor::getPosition() {
  long value;
  portENTER_CRITICAL(&mux);
  value = enCount;
  portEXIT_CRITICAL(&mux);
  return value / encRatio;
}

long BlueMotor::getCount() {
  long value;
  portENTER_CRITICAL(&mux);
  value = enCount;
  portEXIT_CRITICAL(&mux);
  return value;
}

long BlueMotor::getErr() {
  return err;
}

void BlueMotor::reset() {
  portENTER_CRITICAL(&mux);
  enCount = 0;
  portEXIT_CRITICAL(&mux);
}
