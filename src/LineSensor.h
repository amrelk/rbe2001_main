#pragma once

#include "RBE1001Lib.h"

#define NUMSENS 2

class LineSensor
{
private:
  ESP32AnalogRead sensors [NUMSENS];
public:
  LineSensor(int pins[]);
  ~LineSensor();
  float getOffset();
  float getTotal();
  bool isAtCross();
};