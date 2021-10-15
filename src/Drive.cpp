#include "Drive.h"

Drive::Drive(LeftMotor* left, RightMotor* right, Rangefinder* ranger, LineSensor* lineSensor):
left(left), right(right), ranger(ranger), lineSensor(lineSensor) {
  xTaskCreate(this->driveTask, "Drive Control Task", 1000, this, 1, NULL);
}

Drive::~Drive()
{
}

void Drive::lineFollow() {
  state = LINE;
}

void Drive::approach(float distance) {
  state = APPROACH;
  rangeSP = distance;
}

void Drive::turn(float degrees) {
  if (state != TURN) {
    state = TURN;
    left->startMoveFor(-degrees*2.1, 100);
    right->startMoveFor(degrees*2.1, 100);
  } else if (left->isMotorDoneWithMove() && right->isMotorDoneWithMove()) {
    state = OFF;
  }
}

void Drive::manualForwards(float cm) {
  if (state != MANUAL) {
    state = MANUAL;
    left->startMoveFor(cm*5.14, 100);
    right->startMoveFor(cm*5.14, 100);
  } else if (left->isMotorDoneWithMove() && right->isMotorDoneWithMove()) {
    state = OFF;
  }
}

bool Drive::isIdle() {
  return state == OFF;
}

void Drive::lineFollowerLoop(float speed) {
  left->setSpeed(speed + (speed>0?-followerKp*lineSensor->getOffset():-followerKp*lineSensor->getOffset()));
  right->setSpeed(speed + (speed>0?followerKp*lineSensor->getOffset():followerKp*lineSensor->getOffset()));
}

void Drive::ultrasonicSensorPid() {
  lineFollowerLoop(constrain(25*(ranger->getDistanceCM() - rangeSP), -100, 100));
}

void Drive::driveTask(void* params) {
  Drive* drive = (Drive*) params;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    switch(drive->state) {
      case OFF:
        drive->left->setEffort(0);
        drive->right->setEffort(0);
        break;
      case MANUAL:
      case TURN:
        break;
      case APPROACH:
        drive->ultrasonicSensorPid();
        break;
      case LINE:
        drive->lineFollowerLoop(drive->followerSpeed);
        break;

        
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
}