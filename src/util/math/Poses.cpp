#include "Poses.h"


Pose2D::Pose2D(Translation2D pos, Rotation2D theta) : pos(pos), theta(theta) {}


Rotation2D::Rotation2D() : Rotation2D(0) {}
Rotation2D::Rotation2D(float theta) : theta(theta) {}

Translation2D::Translation2D() : Translation2D(0,0) {}
Translation2D::Translation2D(float x, float y) : x(x), y(y) {}

Twist2D::Twist2D() : Twist2D(0,0,0) {}
Twist2D::Twist2D(float dx, float dy, float dtheta) : dx(dx), dy(dy), dtheta(dtheta) {}