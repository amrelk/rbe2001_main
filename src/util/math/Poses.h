class Translation2D
{
private:
    float x;
    float y;
public:
    Translation2D();
    Translation2D(float x, float y);
    ~Translation2D();
};

class Rotation2D
{
private:
    float theta;
public:
    Rotation2D();
    Rotation2D(float theta);
    ~Rotation2D();
};

class Twist2D
{
private:
    float dx;
    float dy;
    float dtheta;
public:
    Twist2D();
    Twist2D(float dx, float dy, float dtheta);
    ~Twist2D();
};

class Pose2D
{
private:
    Translation2D pos;
    Rotation2D theta;
public:
    Pose2D();
    Pose2D(Translation2D pos, Rotation2D theta);
    ~Pose2D();
};