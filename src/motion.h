
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
#define BETA_START                2.5f  // Start beta with quick convergence (noisy)
#define BETA_CONVERGENCE_TIME     1000  // ms

#define GYRO_NOISEGATE            50    // defines rotation stillness (about 3°/s) for calibration
#define DEFAULT_GYRO_STILLNESS    0.0f  // For gyro noisegate during live motion computations

/* Use Case Recommended Alpha Behavior
High Stability (best accuracy, slow adaptation)       0.01 - 0.05     Very stable, adapts slowly to changes. Ideal for long-term calibration.
General Use (balanced accuracy & speed)               0.05 - 0.1      Recommended for most applications (drone IMUs, robotics).
Fast Adaptation (responds quickly to disturbances)    0.1 - 0.3       Good for dynamic environments but less stable.
Very Fast Adaptation (unstable but responsive)        0.3 - 0.5       Reacts very quickly to changes but is noisy. Use with caution.
*/
#define MEAN_SMOOTHER_ALPHA           0.001f
#define EPSILON_SYMMETRY_MATRIX       0.2f  
#define MAG_OFFSETS_STABLE_MAX_TIME   5000    // ms
#define MAG_AUTOCAL_MAX_TIME          60000   // ms
#define SCATTER_PARAM_COUNT           10      // Scatter parameter size

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
  void resetBeta();   // resets Beta Interpolator
  void grab();    // Retrieves sensors data from sensor classes
  void grabImu(); // Same, just the IMU (acc, gyro, temp)
  void grabMag();
  void compute();
  float gyroNorm();
  void runAutoCalMag();
  void runAutoCalMotion();
  void applyOrientation();  // Flip axis and signs
  uint32_t getSampleRate() { return sampleRate; }
  void setSampleRate(uint32_t rate);
  void setGyroGate(float gate) { gyroGate = gate; }

  void setGyroBias(int bias, uint8_t axis) { gyro_bias[axis] = bias; gbias[axis] = (float)gyro_bias[axis] * gRes; }
  void setAccelBias(int bias, uint8_t axis) { accel_bias[axis] = bias; abias[axis] = (float)accel_bias[axis] * aRes; }
  void setMagBias(int bias, uint8_t axis) { mag_bias[axis] = bias; mbias[axis] = (float)mag_bias[axis] * mRes; }
  void setBeta(float gain) { beta = gain; }
  void setDeclination(float angle) { declination = angle; }
  void setOrientation(uint8_t orient) { orientation = orient; }
  void setSoftIronMatrix(float v[3], uint8_t axis);
  
  int getGyroBiasRaw(uint8_t axis) { return gyro_bias[axis]; }
  int getAccelBiasRaw(uint8_t axis) { return accel_bias[axis]; }
  int getMagBiasRaw(uint8_t axis) { return mag_bias[axis]; }
  float getBeta() { return beta; }
  float getDeclination() { return declination; }
  uint8_t getOrientation() { return orientation; }
  float getGyroGate() { return gyroGate; }
  float (*getSoftIronMatrix())[3] {return softIronMatrix;}
  float *getSoftIronMatrixRow(uint8_t axis) { return &(softIronMatrix[axis][0]); }

  void madgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void updateIMU(float ax, float ay, float az, float gx, float gy, float gz);
  void computeHeading(void);
  void computeGravity(void);
  void computeMagnetic(void);
  float computeConvergenceError(void);

  bool calibrateAccGyro();
  bool calibrateMag(bool end = false);
  bool accGyroOffsetCompute(void);
  bool magOffsetCompute();
  void magOffsetComputeLive(void);
  void accGyroBiasCompute(void);
  void magBiasCompute(void);
  void resetGyroOffsetCalibration(void);
  void resetAccOffsetCalibration(void);
  void resetMagOffsetCalibration(void);
  void resetSoftIron(void);
  void updateScatterMatrix(void);
  bool computeSoftIronMatrix(void);
  void applySoftIronMatrix(void);
  bool isStillCalibration(void);

  void nextStep(bool state) { nextCalibrationStep = state; }
  void cancel(bool state) { cancelCalibration = state; }
  bool isNextStep() { return nextCalibrationStep; }
  bool isCancel() { return cancelCalibration; }

  int16_t accX, accY, accZ;
  int16_t gyrX, gyrY, gyrZ;
  int16_t magX, magY, magZ;
  int16_t boardTemperatureRaw;

  CRGBW8 blinkColor;

  float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z; // variables to hold latest sensor data values
  float convError;
  float temperature, boardTemperature;
  float altitude, pressure;
  float pitch, yaw, roll, heading;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
  float grav_x, grav_y, grav_z; // Gravity vector
  float mag_x, mag_y, mag_z;    // Magnetic vector
  double bno055Data[4]; // stores Euler
  double bno055Quat[4]; // stores quaternion

private:
  float beta = BETA_DEFAULT;
  float madgwick_beta_max = BETA_MAX;
  float madgwick_beta_gain = 1.0f;

  // Auto-calibration / On the go calibration
  uint32_t autoCalMagElapsed = 0;
  uint32_t stableMeanMagCounter = 0;
  uint32_t autocalMagMaxTime = MAG_AUTOCAL_MAX_TIME;           // ms
  uint32_t stableMagMaxTime = MAG_OFFSETS_STABLE_MAX_TIME;     // ms
  bool stableMeanMag = false;
  bool hardIronOK = false;
  bool softIronOK = false;
  uint32_t magSampleCount = 0;
  int gyroAutocalThreshold = GYRO_NOISEGATE;    // for Acc / Gyro
  uint32_t gyroOffsetAutocalCounter;              // time equivalent to nb valid stable samples
  uint32_t gyroOffsetAutocalTime = 1000;          // ms = 200 samples @5ms
  bool autoCalMagOn = false;
  bool autoCalMotionOn = false;
  bool accGyroCalDone = false;
  bool hardIronLive = true;

  // Calibration offsets storage
  int gyroOffsetAutocalMin[3];
  int gyroOffsetAutocalMax[3];
  int gyroOffsetAutocalSum[3];
  int magOffsetAutocalMin[3];
  int magOffsetAutocalMax[3];
  int accOffsetAutocalSum[3];
  bool nextCalibrationStep = false;
  bool cancelCalibration = false;

  // Mag Soft iron stuff
  float meanMag[3];     // To compare with mag min+max/2 above
  double scatterMatrix[SCATTER_PARAM_COUNT][SCATTER_PARAM_COUNT];   // Scatter Matrix for elipsoid fitting
  float softIronMatrix[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},{0.0f, 0.0f, 1.0f}}; 
  float alphaSmoother = MEAN_SMOOTHER_ALPHA;
  

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

  float gyroGate;
  float mag_nobias[3];

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float _4q0, _4q1, _4q2, _8q1, _8q2;
  float _8bx, _8bz;
  float halfMinusQySquared;

  // Heading calculation
  float iSinRoll, iCosRoll, iSinPitch, iCosPitch;
  float iBpx, iBpy, iBpz;
  float iBfx, iBfy, iBfz;  // de rotated values of the mag sensors set to NED frame

  linearInterpolator lerpBeta;
  
  bool _initialized = false;  
};


extern motionCore motion;

#endif
