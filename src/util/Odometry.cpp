#include "Odometry.h"

Odometry::Odometry(LeftMotor* left, RightMotor* right) {

}

Odometry::~Odometry() {
    
}

void thread(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        Serial.println(xTaskGetTickCount());
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void Odometry::start() {
    lastDegL = leftMotor->getCurrentDegrees();
    lastDegR = rightMotor->getCurrentDegrees();
    xTaskCreate(thread, "Odometry Thread", 1000, NULL, 1, NULL);
}