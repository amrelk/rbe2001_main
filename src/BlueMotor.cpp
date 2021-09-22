#include "BlueMotor.h"
#include <RBE1001Lib.h>

long enCount = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderIsr() {
  portENTER_CRITICAL_ISR(&mux);
  enCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

BlueMotor::BlueMotor() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  pinMode(PWM, OUTPUT);
  attachInterrupt(ENCA, encoderIsr, CHANGE);
}

BlueMotor::~BlueMotor() {
}

void BlueMotor::setEffort(int effort) {
  if (effort > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
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

void BlueMotor::reset() {
  portENTER_CRITICAL(&mux);
  enCount = 0;
  portEXIT_CRITICAL(&mux);
}