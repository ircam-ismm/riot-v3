
#ifndef _MOTION_H
#define _MOTION_H

#include "main.h"
#include "routines.h"
#include "sensors.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Absolute angle (madgwick)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the IMU response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for our motion needs
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// Beta is called the rate of convergence of the filter. Higher value lead to a noisy output but fast response
// If beta = 0.0 => uses gyro only. A very high beta like 2.5 uses almost no gyro and only accel + magneto.

#define BETA_DEFAULT              0.4f  // Much faster - noisier
#define BETA_MAX                  10.f
#define BETA_START                10.f  // Start beta with quick convergence
#define BETA_CONVERGENCE_TIME     1000  // ms


// Try Gyro HPF instead
#define GYRO_NOISEGATE            50    // defines rotation stillness - about 3°/s

#define MIN_SAMPLERATE    3
#define MAX_SAMPLERATE    1000

#define G_TO_MS2          9.80665f

enum s_Axis {
  X_AXIS = 0,
  Y_AXIS,
  Z_AXIS
};

enum s_BoardOrientation {
  TOP_NWU_WIDTH = 0,
  TOP_NWU_LENGTH,
  BOTTOM_NWU_WIDTH,
  BOTTOM_NWU_LENGTH,
  MAX_BOARD_ORIENTATION
};


// Make those dynamic with parameters in the config file
// Default scales
#define GYRO_SCALE  2000.0f     // +- 2000 deg/s
#define ACC_SCALE   8.0f        // +- 8g
#define MAG_SCALE   4.0f        // +- 4 Gauss

class motionCore {
public:
  motionCore();
  ~motionCore();

  void init();
  void begin();
  void grab();    // Retrieves sensors data from sensor classes
  void grabImu(); // Same, just the IMU (acc, gyro, temp)
  void grabMag();
  void compute();
  void applyOrientation();  // Flip axis and signs
  uint32_t getSampleRate() { return sampleRate; }
  void setSampleRate(uint32_t rate);

  void setGyroBias(int bias, uint8_t axis) { gyro_bias[axis] = bias; gbias[axis] = (float)gyro_bias[axis] * gRes; }
  void setAccelBias(int bias, uint8_t axis) { accel_bias[axis] = bias; abias[axis] = (float)accel_bias[axis] * aRes; }
  void setMagBias(int bias, uint8_t axis) { mag_bias[axis] = bias; mbias[axis] = (float)mag_bias[axis] * mRes; }
  void setBeta(float gain) { beta = gain; }
  void setDeclination(float angle) { declination = angle; }
  void setOrientation(uint8_t orient) { orientation = orient; }

  int getGyroBiasRaw(uint8_t axis) { return gyro_bias[axis]; }
  int getAccelBiasRaw(uint8_t axis) { return accel_bias[axis]; }
  int getMagBiasRaw(uint8_t axis) { return mag_bias[axis]; }
  float getBeta() { return beta; }
  float getDeclination() { return declination; }
  uint8_t getOrientation() { return orientation; }
  

  void madgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void updateIMU(float ax, float ay, float az, float gx, float gy, float gz);
  void computeHeading(void);
  void computeGravity(void);
  float accurateinvSqrt(float x);
  float invSqrt(float x);

  bool calibrateAccGyro();
  bool magOffsetCalibration(bool end = false);
  void gyroOffsetCalibration(void);
  void resetGyroOffsetCalibration(void);
  void resetAccOffsetCalibration(void);
  void resetMagOffsetCalibration(void);

  void nextStep(bool state) { nextCalibrationStep = state; }
  bool isNextStep() { return nextCalibrationStep; }

  int16_t accX, accY, accZ;
  int16_t gyrX, gyrY, gyrZ;
  int16_t magX, magY, magZ;
  int16_t boardTemperatureRaw;

  CRGBW8 blinkColor;

  float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z; // variables to hold latest sensor data values
  float temperature, boardTemperature;
  float altitude, pressure;
  float pitch, yaw, roll, heading;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
  float grav_x, grav_y, grav_z; // Gravity vector
  double bno055Data[4]; // stores Euler, acc/gyr/mag data or quaternion

private:
  float beta = BETA_DEFAULT;
  float madgwick_beta_max = BETA_MAX;
  float madgwick_beta_gain = 1.0f;

  int gyroOffsetAutocalTime = 500;                   // ms = 100 samples @5ms
  int gyroOffsetAutocalThreshold = GYRO_NOISEGATE;    // LSB
  int gyroOffsetAutocalCounter;                       // Nb of valid stable samples
  bool gyroOffsetAutocalOn = false;
  bool gyroOffsetCalDone = false;
  int gyroOffsetCalElapsed = 0;
  long gyroOffsetAutocalMin[3];
  long gyroOffsetAutocalMax[3];
  long gyroOffsetAutocalSum[3];
  long magOffsetAutocalMin[3];
  long magOffsetAutocalMax[3];
  long accOffsetAutocalSum[3];
  bool nextCalibrationStep = false;


  float declination = DECLINATION;
  uint8_t orientation = TOP_NWU_LENGTH;
  uint32_t sampleRate = DEFAULT_SAMPLE_RATE;
  float deltat = 0.005f;        // integration interval for both filter schemes - 5ms by default

  int gyro_bias[3] = { 0, 0, 0};
  int accel_bias[3] = { 0, 0, 0};
  int mag_bias[3] = { 0, 0, 0};
  int bias_samples = 32;

  float abias[3] = { 0., 0., 0.};
  float gbias[3] = { 0., 0., 0.};
  float mbias[3] = { 0., 0., 0.};

  float gRes, aRes, mRes;    // Resolution = Sensor range / 2^15

  float gyro_norm; // used to tweak Beta
  float mag_nobias[3];

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float _4q0, _4q1, _4q2, _8q1, _8q2;
  float _8bx, _8bz;

  // Heading calculation
  float iSin, iCos;
  float iBpx, iBpy, iBpz;
  float iBfx, iBfy, iBfz;  // de rotated values of the mag sensors set to NED frame

  
  bool _initialized = false;  
};


extern motionCore motion;

#endif
