#include "LineSensor.h"

LineSensor::LineSensor(int pins[]) {
  for( int i = 0; i < NUMSENS; i++ ) {
    sensors[i].attach(pins[i]);
  }
}

LineSensor::~LineSensor() {}

float LineSensor::getOffset() {
  return sensors[0].readVoltage() - sensors[1].readVoltage();
}

float LineSensor::getTotal() {
  return sensors[0].readVoltage() + sensors[1].readVoltage();
}

bool LineSensor::isAtCross() {
    return getTotal() > 3.0;
}