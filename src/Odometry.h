#include "RBE1001Lib.h"

class Odometry {
private:
    float lastL;
    float lastR;
public:
    
Odometry(Motor* left, Motor* right);
    ~Odometry();
};
