#include <functional>

class BlueMotor
{
private:
  const float encRatio = 0.75;

public:
  BlueMotor();
  ~BlueMotor();

  void setEffort(int effort);
  void setTarget(long count);
  void startPid();

  float getPosition();
  long getCount();
  long getErr();
  void reset();

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  std::function<float (long)> Ff = [] (long count) { return 0.0; };
};
