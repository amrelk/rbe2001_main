#include "RBE1001Lib.h"
#include "math/Poses.h"

class Odometry {
private:
    LeftMotor* leftMotor;
    RightMotor* rightMotor;
    float lastDegL;
    float lastDegR;
    Pose2D pose;
public:
    Odometry(LeftMotor* left, RightMotor* right);
    ~Odometry();
    void start();
};
