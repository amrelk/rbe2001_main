class BlueMotor
{
private:
  const int PWM = 5;
  const int AIN2 = 23;
  const int AIN1 = 27;
  const int ENCA = 19;
  const int ENCB = 18;
  const float encRatio = 0.75;

public:
  BlueMotor();
  ~BlueMotor();

  void setEffort(int effort);

  float getPosition();
  long getCount();
  void reset();
};
