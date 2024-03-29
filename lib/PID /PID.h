#define THRESHOLD 0.001 // Twiddle threshold

class PID {
  float P;
  float I;
  float D;  
  float integral;
  float previousError;
  int16_t minimum;
  int16_t maximum; 

  public:
  PID();
  PID(float, float, float, int16_t, int16_t);
  
  void reset();
  void setP(float);
  void setI(float);
  void setD(float);
  void setPID(float, float, float);
  void setLimits(int16_t, int16_t);
  void twiddle();

  float getP();
  float getI();
  float getD();
  float compute(float);
};