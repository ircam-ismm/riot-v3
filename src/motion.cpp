
#include <float.h>
#include "motion.h"

motionCore::motionCore() {
  
}

motionCore::~motionCore() {

}

void motionCore::setSampleRate(uint32_t rate) {
  sampleRate = constrain(rate, MIN_SAMPLERATE, MAX_SAMPLERATE);
  deltat = (float)sampleRate / 1000.0f;
}

void motionCore::init() {
  gRes = GYRO_SCALE / 32768.f;
  aRes = ACC_SCALE / 32768.f;  
  mRes = MAG_SCALE / 32768.f;

  gyroGate = DEFAULT_GYRO_STILLNESS;
  
  for(int i = 0 ; i < 3 ; i++) {
    gyro_bias[i] = accel_bias[i] = mag_bias[i] = 0;
    gbias[i] = abias[i] =  mbias[i] = 0.0f;
  }
  
  beta = BETA_DEFAULT;
  setSampleRate(DEFAULT_SAMPLE_RATE);
  q0 = 1.0f;
  q1 = q2 = q3 = 0.0f;

  // Soft Iron = Identity = no correction
  for(int i = 0; i < 3; i++) {
    softIronMatrix[i][i] = 1.0f;
    for(int j = 0; j < 3; j++) {
      if(j != i)
        softIronMatrix[i][j] = 0.0f;
    }
  }
}


void motionCore::begin() {

  // IMU class only provides raw data
  // Scaling to obtain gs and deg/s
  gRes = (float)lsm6d.getGyroRange() / 32768.f;
  aRes = (float)lsm6d.getAccRange() / 32768.f;  
  mRes = (float)lis3mdl.getRange() / 32768.f;
  

  // Finalize the bias unit conversion
  // Replace this with a proper box filter (float template C++)
  // Test HPF on gyro to remove bias too (?)
  for(int i = 0 ; i < 3 ; i++) {
    gbias[i] = gRes * (float)gyro_bias[i];
    abias[i] = aRes * (float)accel_bias[i];
    meanMag[i] = (float)mag_bias[i];  // EMA live estimator of hard iron bias
    mbias[i] = mRes * (float)mag_bias[i];
  }

  resetBeta();
}

void motionCore::resetBeta() {
  lerpBeta.begin(BETA_START, beta);
  lerpBeta.setDuration(BETA_CONVERGENCE_TIME);
  lerpBeta.start();
}

void motionCore::setSoftIronMatrix(float v[3], uint8_t axis) {
  for(int i = 0; i < 3; i++) {
    softIronMatrix[axis][i] = v[i];
  }
}

// Define Tait-Bryan angles.
// In this coordinate system, the positive z-axis is down toward Earth.
// Yaw is the angle between Sensor x-axis and Earth magnetic North
// (or true North if corrected for local declination, looking down on the sensor
// positive yaw is counterclockwise, which is not conventional for NED navigation.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
void motionCore::grab() {
  lsm6d.read();
  lis3mdl.read();
  bmp390.readPressure();
  bmp390.readAltitude();
  if(riot.hasBNO055()) {
    bno055.Get_Values(bno055Data, Get_EULER);
    // Flip signs / modulo here - BNO performs ZXY rotation
    bno055Data[0] = FROM_360_DEGREE(bno055Data[0]); // Yaw
    bno055Data[1] = -bno055Data[1]; // Roll
    // Pitch Unchanged
    bno055.Get_Values(bno055Quat, Get_QUATERNION);
    // BNO055 frame isn't identical to the onboard fusion. Trick is to swap quaternion X & Y and sign remap
    // Works for our sensor with bno_orien=1
    float tempF = bno055Quat[2];   // swaps X and Y 
    bno055Quat[2] = -bno055Quat[1];
    bno055Quat[1] = tempF; 
  }

  accX = lsm6d.getAccX();
  accY = lsm6d.getAccY();
  accZ = lsm6d.getAccZ();
  gyrX = lsm6d.getGyrX();
  gyrY = lsm6d.getGyrY();
  gyrZ = lsm6d.getGyrZ();
  magX = lis3mdl.getMagX();
  magY = lis3mdl.getMagY();
  magZ = lis3mdl.getMagZ();
  boardTemperatureRaw = lsm6d.getTemp();
  boardTemperature = ((float)boardTemperatureRaw / LSM6DSL_TEMP_SCALE) + LSM_BIAS_TEMPERATURE;
  temperature = bmp390.getTemp();
  pressure = bmp390.getPressure() / 100.f;   // hPa
  altitude = bmp390.getAltitude();
  
  // Review some update loops to subsample what doesn't need to be
  // updated at each iteration. Temperature calibration could be saved for the pressure reading, 
  // or simply call the pressure reading at the update rate of the sensor.
  // Eventually do oversampling of the orientation filter and define
  // an ODR for all the data at once (ie Calculate madgwick quicker to have better convergence and stability)
}


void motionCore::grabImu() {
  lsm6d.read();
  accX = lsm6d.getAccX();
  accY = lsm6d.getAccY();
  accZ = lsm6d.getAccZ();
  gyrX = lsm6d.getGyrX();
  gyrY = lsm6d.getGyrY();
  gyrZ = lsm6d.getGyrZ();
  boardTemperatureRaw = lsm6d.getTemp(); 
}

void motionCore::grabMag() {
  lis3mdl.read();
  magX = lis3mdl.getMagX();
  magY = lis3mdl.getMagY();
  magZ = lis3mdl.getMagZ();
}

// Axis and sign swapping is done on the raw / integer values of the sensors *before* bias computation
// or application
void motionCore::applyOrientation() {
  int16_t swap;
  switch(orientation) {
    // Sensor UP with natural orientation of the sensor X-NORTH-Y-WEST-Z-UP (NWU convention)
    // Sensor / USB is on top - X+ is on the width of the board, Y towards antenna, Z up
    // (natural orientation of the sensor, nothing to flip)
    case TOP_NWU_WIDTH:
      break;

    // Sensor UP with swapped X-Y axis of the sensor so that Y-NORTH-X-WEST-Z-UP (NWU convention)
    // Sensor / USB is on top - X+ is on the legnth of the board towards antenna, Y on the width, Z up
    case TOP_NWU_LENGTH:
      swap = -accX;
      accX = accY;
      accY = swap;
      swap = -gyrX;
      gyrX = gyrY;
      gyrY = swap;
      swap = -magX;
      magX = magY;
      magY = swap;
      break;

    // Sensor DOWN with natural orientation of the sensor X-NORTH-Y-EAST-Z-DOWN (NED convention)
    // Sensor / USB is  bottom side - X+ is on the width of the board, Y is on the legnth of the board towards antenna, Z up
    case BOTTOM_NWU_WIDTH:
      accX = -accX;
      accZ = -accZ;
      gyrX = -gyrX;
      gyrZ = -gyrZ;
      magX = -magX;
      magZ = -magZ;
      break;

    // Sensor DOWN with swapped X-Y axis of the sensor Y-NORTH-X-EAST-Z-DOWN (NED convention)
    // Sensor / USB is bottom side - X+ is on the legnth of the board towards antenna, Y on the width, Z up
    case BOTTOM_NWU_LENGTH:
      swap = accX;
      accX = accY;
      accY = swap;
      accZ = -accZ;
      swap = gyrX;
      gyrX = gyrY;
      gyrY = swap;
      gyrZ = -gyrZ;
      swap = magX;
      magX = magY;
      magY = swap;
      magZ = -magZ;
      break;

    default :
      orientation = TOP_NWU_LENGTH;
      break;
  }
}

//---------------------------------------------------------------------


void motionCore::compute() {

  // Always start by swapping axis and signs
  applyOrientation();

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Magnetometers autocalibration (hard + soft iron)
  if(autoCalMagOn) {
    bool stable = magOffsetCompute();
    
    if(((autocalMagMaxTime && ((millis() - autoCalMagElapsed) > autocalMagMaxTime))) || isCancel() || isNextStep()) {
      Serial.printf("Mag Autocalibration ended\n");
      autoCalMagOn = false;
      // check if acc+gyro cal completed
      if(!isCancel()) {
        magBiasCompute();
      }
    } 
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Acc-Gyro autocalibration (offsets)
  if(autoCalMotionOn) {
    if(!accGyroCalDone) {
      accGyroOffsetCompute();  // Biases auto computed & saved at the end once done
    }
    else {
      autoCalMotionOn = false;
    }
    if(isCancel()) {
      autoCalMotionOn = false;
    }
  }

  if(isNextStep() || isCancel()) {
    cancelCalibration = false;
    nextCalibrationStep = false; 
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////
  // Apply calibration offsets - Hard Iron & Soft Iron
  g_x = (gRes * (float)gyrX) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
  g_y = (gRes * (float)gyrY) - gbias[1];
  g_z = (gRes * (float)gyrZ) - gbias[2];
  
  a_x = (aRes * (float)accX) - abias[0];   // Convert to g's, remove accelerometer biases
  a_y = (aRes * (float)accY) - abias[1];
  a_z = (aRes * (float)accZ) - abias[2];

  // Center magnetometers (remove hard-iron)
  m_x = (mRes * (float)magX) - mbias[0];   // Convert to mGauss and correct for calibration
  m_y = (mRes * (float)magY) - mbias[1];
  m_z = (mRes * (float)magZ) - mbias[2];   

  // Apply Soft-iron correction matrix : corrected = M * centered
  // Apply soft iron correction: 
  float centered[3] = {m_x, m_y, m_z};
  float corrected[3];
  for (int i = 0; i < 3; i++) {
    corrected[i] = 0.0f;
      for (int j = 0; j < 3; j++) {
        corrected[i] += softIronMatrix[i][j] * centered[j];
      }
  }
  m_x = corrected[0];
  m_y = corrected[1];
  m_z = corrected[2];
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Beta interpolation and convergence
  if(lerpBeta.isActive()) {
    beta = lerpBeta.getValue();
    //Serial.printf("Beta = %f\n", beta);
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Note regarding the sensor orientation & angles :
  // Madgwick expects data with a vehicle frame exposed in right-handed convention and NED convention (North East Down)
  // Our sensors are rather in NWU (North West Up) situation, which is still right-handed and works well if axis and signs are
  // swapped properly to match the handedness. However, since the gyro is > 0 counter clockwise (math orientation) and opposed to
  // nav (aircraft / land vehicle heading to the "right" towards EAST = positive), the final Euler yaw angle is signed reversed
  // Pitch is here defined between {-90 ; +90} degrees (gimbal lock) and is positive-up, while roll gets {-180 ; 180} degrees
  // and positive-right (for the standard madgwick version). For the 4 proposed orientation, axis are swapped and signed reflected 
  //so that the end right handedness is preserved. This is now done by applyOrientation(). Other ranges for pitch and roll can be
  // calculated depending on the desired application, examples in comments below.

  // Based on selected orientation, this uses X+ to point north
  //madgwickAHRSupdate(a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z);

  // Based on selected orientation, this uses Y+ to point north as in the W3C standard
  madgwickAHRSupdate(a_y, -a_x, a_z, g_y, -g_x, g_z, m_y, -m_x, m_z);

  // compute the norm of the gyro data => rough estimation of the movement
  // If below threshold, don't update euler and whatnot
  if(gyroGate) {
    if(gyroNorm() < gyroGate)
      return;
  }

  // Euler Angle - Adjust sign here eventually if not happy of the chosen frame convention - Beware of heading computations
  // Standard computation order for quats to euler follow the Yaw-Pitch-Roll convention & order ZYX (standard)
  // This computation brings the pitch in the range of {-90°;+90°} and roll within {-180°;+180°}
  
  // Optimized, using the sum of squared quaternions = 1 and common terms
  halfMinusQySquared = 0.5f - q2*q2; // calculate common terms to avoid repeated operations
    // if BNO is detected, use automatically ZXZ order euler conversion 
  if(!riot.hasBNO055()) {
    yaw   = -atan2f(q1 * q2 + q0 * q3, halfMinusQySquared - q3*q3);   
    pitch = -asinf(2.0f * (q0 * q2) - q1 * q3);
    roll  = atan2f(q0 * q1 + q2 * q3, halfMinusQySquared - q1*q1);
  }
  
  // Yaw-Roll-Pitch order aka YXZ like the BNO055 does, with roll on {-90°;+90} and pitch on {-180°;+180°}
  // TODO : handle gimbal lock with straight transitions on asinf custom (see xio) ?
  else {
    // Original quaternion conversion without optimization (commented)
    //yaw   = -atan2f(2.0f * (q1 * q2 + q0 * q3), q0*q0 + q1*q1 - q2*q2 - q3*q3); // Unchanged since yaw is applied first
    //yaw   = -atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2*q2 + q3*q3) // Unchanged since yaw is applied first
    yaw   = -atan2f(q1 * q2 + q0 * q3, halfMinusQySquared - q3*q3); // Unchanged since yaw is applied first
    roll = asinf(2.0f * (q0 * q1 + q2 * q3));
    //pitch = -atan2f(2.0f * (q0*q2 - q1*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
    //pitch = -atan2f(2.0f * (q0*q2 - q1*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    pitch = -atan2f(q0*q2 - q1*q3, halfMinusQySquared - q1*q1);
  }
  
  // Compute heading *BEFORE* the final export of yaw pitch roll to save float computation of deg2rad / rad2deg
  computeHeading();
  computeGravity();     // Gravity vector
  computeMagnetic();    // Magnetic vector

  // Compute error between magnetic and gravity
  // Cross product between magnetic values in madgwick frame and gravity (indicates how they are colinear or not)
  // This will give an idea of the current convergence error. Needs vector arithmetics => later with new AHRS version
  // Thresholding on the error would allow for having the data stable enough on the output : fusion continues to operate 
  // but angles below are updated only when error rises above a threshold
  //convError = computeConvergenceError();
  //Serial.printf("%d\n", convError);
  
  /////////////////////////////////////////////////////////////////////////////////
  // Degree per second conversion and declination correction 
  pitch *= RAD_TO_DEG;
  yaw   *= RAD_TO_DEG; 
  yaw   -= declination; 
  roll  *= RAD_TO_DEG;
  heading *= RAD_TO_DEG;
  heading = TO_360_DEGREE(heading);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. 
void motionCore::madgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  
  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    //Serial.println("Mag data invalid - no update");
    updateIMU(ax, ay, az, gx, gy, gz);
    return;
  }

  // Convert gyroscope degrees/sec to radians/sec
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = accurateinvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Normalise magnetometer measurement
    recipNorm = accurateinvSqrt(mx * mx + my * my + mz * mz); 
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;


    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
     // Correction / addon
    _8bx = 2.0f * _4bx;
    _8bz = 2.0f * _4bz;

    // Gradient decent algorithm corrective step
    // Commented = old algo with errors
    //s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _4bz * q2 * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q3 + _4bz * q1) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q2 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
    //s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _4bz * q3 * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q2 + _4bz * q0) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q3 - _8bz * q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
    //s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_8bx * q2 - _4bz * q0) * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * q1 + _4bz * q3) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * q0 - _8bz * q2) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz);
    //s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_8bx * q3 + _4bz * q1) * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * q0 + _4bz * q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * q1 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5f - q1q1 - q2q2) - mz); 
    
    recipNorm = accurateinvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltat;
  q1 += qDot2 * deltat;
  q2 += qDot3 * deltat;
  q3 += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = accurateinvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// AHRS with no Mag when not available or NaN detected
void motionCore::updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
  
  // Convert gyroscope degrees/sec to radians/sec
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = accurateinvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    recipNorm = accurateinvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltat;
  q1 += qDot2 * deltat;
  q2 += qDot3 * deltat;
  q3 += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = accurateinvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// https://oduerr.github.io/gesture/ypr_calculations.html
// For NWU frame
void motionCore::computeGravity() {
   grav_x = 2.0f * (q1 * q3 - q0 * q2);
   grav_y = 2.0f * (q0 * q1 + q2 * q3);
   //grav_z = q0q0 - q1q1 - q2q2 + q3q3;
   grav_z = 2.0f * (q0q0 + q3q3) - 1.0f;
}

// For NWU frame
void motionCore::computeMagnetic() {
   mag_x = 2.0f * (q1 * q2 + q0 * q3);
   mag_y = 2.0f * (q0*q0 + q2*q2) - 1.0f;
   mag_z = 2.0f * (q2 * q3 - q0 * q1);
}

// Requires gravity and heading to be computed before. Replace with vector arithmetics when time comes
float motionCore::computeConvergenceError() {
  // Computes the cross product of the gravity vector and magnetic vector then the squared norm of the resulting vector
  // Close to zero, magnetic and gravity are aligned (ie filter converged)
  float vx, vy, vz, squaredNorm, invMagneticNorm;

  // half cross product between gravity and hard-iron compensated mag (in madgwick frame)
  vx = (grav_y * iBpz - grav_z * iBpy);
  vy = (grav_z * iBpx - grav_x * iBpz);
  vz = (grav_x * iBpy - grav_y * iBpx);

  invMagneticNorm = accurateinvSqrt(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);

  //Serial.printf("vx = %f / vy = %f / vz = %f / invNorm = %f \n", vx, vy, vz, invMagneticNorm);
  
  // Normalize using magnetic vector as reference
  vx = vx * invMagneticNorm;
  vy = vy * invMagneticNorm;
  vz = vz * invMagneticNorm;

  squaredNorm = vx*vx + vy*vy + vz*vz;
  return squaredNorm;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Computation below got adapted from Freescale application note for a tilt compensated compass, 
// formerly using accelerometers as inclinometers.
// We however directly grab the stable Pitch / Roll angles from Madgwick to de-rotate the mag data.
// This way we are un-sensitive to shaking (classic algorithm uses static accel data to get absolute angles)
// Beware, the computation below expects angles provided in radian so the routine must be called 
// when pitch and roll are still expressed in that unit. Cheezy dirty cheap optimization but embedded rulz
// Sourced from Freescale / NXP App Note AN4248 - https://www.nxp.com/docs/en/application-note/AN4248.pdf
// See also: https://circuitcellar.com/cc-blog/implement-a-tilt-and-interference-compensated-electronic-compass/

void motionCore::computeHeading(void) {
      
  // We work with the calibrated values of the MAG sensors (hard iron offset removed)
  // We need to apply the same permutation as in madgwick's call to have the natural Y (W3C frame) becoming X and pointing North
  // which corresponds to swapping X and Y plus sign. We remain in NWU frame (we just rotate the frame +90°)
  // This should work regardless to the sensor orientation permutations as long as we keep the same NWU frame as the madwick call
  // so we have a common north pointing direction and axis.
  iBpx = m_y;
  iBpy = -m_x;
  iBpz = m_z;

  // Sensor frame is made NWU by axis swapping. This converts NWU to NED
  iBpy = -iBpy;
  iBpz = -iBpz;
  
  /* calculate sin and cosine of roll angle Phi */
  iSinRoll = sinf(roll);
  iCosRoll = cosf(roll);
  /* calculate sin and cosine of pitch angle Theta */
  iSinPitch = sinf(pitch);
  iCosPitch = cosf(pitch); 

  /* de-rotate by pitch angle Theta */
  iBfx = (iBpx * iCosPitch) + (iBpz * iSinPitch); /* Eq 19: x component */
  /* de-rotate by roll angle Phi and Theta */  
  iBfy = iBpx * iSinRoll * iSinPitch + iBpy * iCosRoll - iBpz * iSinRoll * iCosPitch;
  
  /* calculate current yaw/heading */
  heading = atan2f(-iBfy, iBfx); /* Eq 22 */
}


///////////////////////////////////////////////////////////////////////////
// Calibration functions
void motionCore::runAutoCalMag(void) {
  Serial.printf("Mag Autocalibration started\n");
  resetMagOffsetCalibration();
  autoCalMagElapsed = millis();
  autoCalMagOn = true;
}

void motionCore::runAutoCalMotion(void) {
  Serial.printf("Motion Autocalibration started\n");
  resetAccOffsetCalibration();
  resetGyroOffsetCalibration();
  accGyroCalDone = false;
  autoCalMotionOn = true;
}


bool motionCore::calibrateAccGyro(void) {
  static bool ledState = 0;
  static int winkCounter = 0;
  
  delay(sampleRate);
  grabImu();
  applyOrientation();
  
  winkCounter++;
  // Guaranties a proper blinking independent of the samplerate for standard rates {2;50}
  if(winkCounter > (50 / sampleRate)) {
    winkCounter = 0;
    ledState = 1 - ledState;
    blinkColor = ledState ? Yellow : Black;
    setLedColor(blinkColor);
  }
  return(accGyroOffsetCompute());   // finished or not ?
}

bool motionCore::calibrateMag(bool end) {
  static bool ledState = 0;
  static int winkCounter = 0;

  delay(sampleRate);
  grabMag();
  applyOrientation();

  winkCounter++;
  // Guaranties a proper blinking independent of the samplerate for standard rates {2;50}
  if(winkCounter > (50 / sampleRate)) {
    winkCounter = 0;
    ledState = 1 - ledState;
    blinkColor = ledState ? White : Black;
    setLedColor(blinkColor);
  }

  magOffsetCompute();

  if(end) {
    magBiasCompute();
    return(true);
  }
  return(false);
}

void motionCore::accGyroBiasCompute(void){
  for(int i = 0 ; i < 3 ; i++) {
    gyro_bias[i] = gyroOffsetAutocalSum[i];
    gbias[i] = gRes * (float)gyro_bias[i];
    accel_bias[i] = accOffsetAutocalSum[i];
    abias[i] = aRes * (float)accel_bias[i];
  }
  char str[MAX_STRING_LEN];
  sprintf(str, "*** FOUND Bias acc= %d %d %d", accel_bias[0], accel_bias[1], accel_bias[2]);
  Serial.printf("%s\n", str);
  printToOSC(str);
  sprintf(str,"*** FOUND Bias gyro= %d %d %d", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  Serial.printf("%s\n", str);
  printToOSC(str);
}


void motionCore::magBiasCompute(void) {
  // TODO : compare accumulated mean with max+min / 2  
  // Compute eigen values and softIron matrix
  computeSoftIronMatrix();
  for(int i = 0 ; i < 3 ; i++) {
    mag_bias[i] = (magOffsetAutocalMax[i] + magOffsetAutocalMin[i]) / 2;
    //mag_bias[i] = (int)meanMag[i];
    mbias[i] = mRes * (float)mag_bias[i];
    //Serial.printf("Mag Max[%d]=%d ; Min[%d]=%d\n",i, magOffsetAutocalMax[i], i, magOffsetAutocalMin[i]);
  }
  char str[MAX_STRING_LEN];
  sprintf(str, "*** FOUND Bias mag (MinMax) = %d %d %d", mag_bias[0], mag_bias[1], mag_bias[2]);
  Serial.printf("%s\n", str);
  Serial.printf("*** FOUND Hard Iron (EMA) = %f %f %f\n", meanMag[0], meanMag[1], meanMag[2]);
  printToOSC(str);
  Serial.printf("Soft Iron Correction Matrix:\n");
  printMatrix3x3(softIronMatrix);
}

// Should be called only when relevant rotations are detected, otherwise bias is shifting during stillness
void motionCore::magOffsetComputeLive(void) {
  float magSample[3] = {magX, magY, magZ};
  static int cnt = 0;
  if(cnt++ > 10) {
      cnt = 0;
  }
      
  // Update hard iron offset with EMA
  for(int i = 0 ; i < 3 ; i++) {
    meanMag[i] = (1.0f - alphaSmoother) * meanMag[i] + alphaSmoother * magSample[i];
    mag_bias[i] = (int)meanMag[i];
    mbias[i] = mRes * (float)mag_bias[i];
  }
  if(!cnt) {
    Serial.printf("%f %f %f\n", meanMag[0],meanMag[1],meanMag[2]);
  }
}

// EMA for mag bias doesn't work as it expects continuous motion to properly converge
// We don't want data collection either, so min-max method is used when rotating the 
// module over the 3 axis. stable/dirty flag is used to automatically end calibration after stabilized time
bool motionCore::magOffsetCompute(void) {
  int vect[3] = {magX, magY, magZ};
  int tempVal;
  bool stable = true;
  for(int i = 0 ; i < 3 ; i++) {
    tempVal = max(magOffsetAutocalMax[i], vect[i]);
    if(tempVal != magOffsetAutocalMax[i]) {
      stable = false;
      magOffsetAutocalMax[i] = tempVal;
    }
    tempVal = min(magOffsetAutocalMin[i], vect[i]);
    if(tempVal != magOffsetAutocalMin[i]) {
      stable = false;
      magOffsetAutocalMin[i] = tempVal;
    }
    mag_bias[i] = (float)(magOffsetAutocalMax[i] + magOffsetAutocalMin[i]) / 2.0f;
  }

  if(stableMeanMag != stable) {
    stableMeanMag = stable;
    stableMeanMagCounter = millis();
  }
  // Offsets (hard iron) didn't change for long enough
  if(!hardIronOK && stableMeanMag && (millis() - stableMeanMagCounter) > stableMagMaxTime) {
    char str[MAX_STRING_LEN];
    sprintf(str, "Mag means stable for long enough - Hard Iron completed");
    Serial.printf("%s\n", str);
    printToOSC(str);
    hardIronOK = true;
    magSampleCount = 0;
  }
  if(hardIronOK) {
    updateScatterMatrix();
  }
  return(stable);
}

void motionCore::updateScatterMatrix(void) {
  float magSample[3] = {magX, magY, magZ};
  
  // Update hard iron offset with EMA on the side to further compare
  for(int i = 0 ; i < 3 ; i++) {
    meanMag[i] = (1.0f - alphaSmoother) * meanMag[i] + alphaSmoother * magSample[i];
  }
  
  // Remove hard iron before soft iron estimation
  for(int i = 0 ; i < 3 ; i++) {
    magSample[i] = magSample[i] - mag_bias[i];
  }
  
  double v[SCATTER_PARAM_COUNT] = {
    magSample[0]*magSample[0],
    magSample[1]*magSample[1],
    magSample[2]*magSample[2],
    magSample[0]*magSample[1],
    magSample[0]*magSample[2],
    magSample[1]*magSample[2],
    magSample[0],
    magSample[1],
    magSample[2],
    1.0f
  };

  for (int i = 0; i < SCATTER_PARAM_COUNT; i++) {
    for (int j = 0; j < SCATTER_PARAM_COUNT; j++) {
      scatterMatrix[i][j] += v[i] * v[j];
    }
  }
  magSampleCount++;
}


// --- Compute soft iron correction matrix (whitening transformation) ---
bool motionCore::computeSoftIronMatrix(void) {
  
  //Serial.printf("Raw Scatter Matrix\n");
  //printMatrix10x10(scatterMatrix);
  // Normalize (scale) by number of samples 
  double norm = 1.0f / (double)magSampleCount;
  double elipsoidCenter[3];
  double elipsoidCoeffs[SCATTER_PARAM_COUNT];  // Eigen vector

  
  Serial.printf("norm = %f / sampleCount = %d\n", norm, magSampleCount);
  for (int i = 0; i < SCATTER_PARAM_COUNT; i++) {
    for (int j = 0; j < SCATTER_PARAM_COUNT; j++) {
      scatterMatrix[i][j] *= norm;
    }
  }
  //symmetrize10x10(scatterMatrix); // Most likely not needed
  //Serial.printf("Scatter Matrix after normalization by sampleCount\n");
  //printMatrix10x10(scatterMatrix);
  
  // elipsoidCoeffs is the resulding eigen vector
  if(powerIteration10x10(scatterMatrix, elipsoidCoeffs, 100, 1e-6f)) {
    // beta = [A B C D E F G H I J]
    Serial.printf("Ellipsoid Coefficients:\n");
    for (int i = 0; i < SCATTER_PARAM_COUNT; i++) {
        Serial.printf("%.6f ", elipsoidCoeffs[i]);
    }
    Serial.printf("\n");
  }

  // Step 1: Construct M matrix from elipsoid parameters (beta)
  double A = elipsoidCoeffs[0], B = elipsoidCoeffs[1], C = elipsoidCoeffs[2];
  double D = elipsoidCoeffs[3], E = elipsoidCoeffs[4], F = elipsoidCoeffs[5];
  double G = elipsoidCoeffs[6], H = elipsoidCoeffs[7], I = elipsoidCoeffs[8];

  double Q[3][3] = {
    { A, D / 2.0, E / 2.0 },
    { D / 2.0, B, F / 2.0 },
    { E / 2.0, F / 2.0, C }
  };

  double v[3] = { G / 2.0, H / 2.0, I / 2.0 };
  double c = elipsoidCoeffs[9];

  // Step 2: invert matrix
  double Q_inv[3][3];
  if (!inverse3x3(Q, Q_inv)) {
    Serial.printf("Failed to invert ellipsoid matrix.\n");
    return false;
  }

  // Step 3 : extract center c = -inv(Q) * v
  for (int i = 0; i < 3; i++) {
    elipsoidCenter[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      elipsoidCenter[i] -= Q_inv[i][j] * v[j];
    }
  }

  // Step 4: Compute vtAinvb = v^T * Q_inv * v
    double vtAinvb = 0.0;
    for (int i = 0; i < 3; i++) {
        double row = 0.0;
        for (int j = 0 ; j < 3; j++)
            row += Q_inv[i][j] * v[j];
        vtAinvb += v[i] * row;
    }

    // Step 5: Compute scale
    double scale = 1.0f;
    if(vtAinvb != c)
      scale = 1.0f / (vtAinvb - c);

    //Serial.printf("Scale = %f\n", scale);

    // Step 6: Compute Q_inv = scale * Q_inv
    // (no need to apply scale to Q then invert again => scalar on Q_inv
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Q_inv[i][j] = scale * Q_inv[i][j];  

    // Step 7: Normalize to get the whitening matrix 
    // Should be "close" to identity depending on soft Iron effect
    normalize3x3(Q_inv);
    
    // Step 8: stores soft iron matrix as float for reduce CPU
    // when applying correction
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        softIronMatrix[i][j] = (float)Q_inv[i][j];
      }
    }  

  return true;
}


// calibrate the offset of the gyroscope and accelerometer
bool motionCore::accGyroOffsetCompute(void) {
    //gyro offset autocalibration
    // update min, max, sum and counter
    gyroOffsetAutocalMax[0] = max(gyroOffsetAutocalMax[0], (int)gyrX);
    gyroOffsetAutocalMax[1] = max(gyroOffsetAutocalMax[1], (int)gyrY);
    gyroOffsetAutocalMax[2] = max(gyroOffsetAutocalMax[2], (int)gyrZ);
    
    gyroOffsetAutocalMin[0] = min(gyroOffsetAutocalMin[0], (int)gyrX);
    gyroOffsetAutocalMin[1] = min(gyroOffsetAutocalMin[1], (int)gyrY);
    gyroOffsetAutocalMin[2] = min(gyroOffsetAutocalMin[2], (int)gyrZ);
    
    gyroOffsetAutocalSum[0] = (gyroOffsetAutocalSum[0] + (int)gyrX) / 2;
    gyroOffsetAutocalSum[1] = (gyroOffsetAutocalSum[1] + (int)gyrY) / 2;
    gyroOffsetAutocalSum[2] = (gyroOffsetAutocalSum[2] + (int)gyrZ) / 2;

    accOffsetAutocalSum[0] = (accOffsetAutocalSum[0] + (int)accX) / 2;
    accOffsetAutocalSum[1] = (accOffsetAutocalSum[1] + (int)accY) / 2;
    accOffsetAutocalSum[2] = (accOffsetAutocalSum[2] + ((int)accZ - (int)(1./aRes))) / 2; // Removes gravity
    gyroOffsetAutocalCounter++;
    
    // if the max-min differences are above the threshold, reset the counter and values
    if(!isStillCalibration())  {
        resetGyroOffsetCalibration();
        resetAccOffsetCalibration();
    }
    
    // check if there are enough stable samples. If yes, update the offsets and the "calibrate" flag
    if(gyroOffsetAutocalCounter >= (gyroOffsetAutocalTime / sampleRate)) { // update bias
      accGyroBiasCompute();
      accGyroCalDone = true;
      return(true);  
    }
    return(false);
}

bool motionCore::isStillCalibration() {
  float vect[3];
  float norm = 0.0f;
  // we use the min max we found as we can't use absolute angular speed, since it's biased with an offset
  for(int i = 0 ; i < 3 ; i++) {
    vect[i] = (float)(gyroOffsetAutocalMax[i]-gyroOffsetAutocalMin[i]);
    norm += vect[i] * vect[i];
  }
  norm = sqrtf(norm);
  //Serial.printf("stillness = %f\n", norm);
  return((int)norm < gyroAutocalThreshold); 
}

float motionCore::gyroNorm() {
  return sqrtf(g_x * g_x + g_y * g_y + g_z * g_z);
}

// Reset gyro offset auto calibration / min / max
void motionCore::resetGyroOffsetCalibration(void) {
  gyroOffsetAutocalCounter = 0;
      
  for(int i = 0 ; i<3 ; i++) {                            
    gyroOffsetAutocalMin[i] = 40000;
    gyroOffsetAutocalMax[i] = -40000;
    gyroOffsetAutocalSum[i] = 0;    
    gyro_bias[i] = 0;
    gbias[i] = 0.0f;
   }
}

// Reset gyro offset auto calibration / min / max
void motionCore::resetMagOffsetCalibration(void) {
  for(int i = 0 ; i<3 ; i++) {  
    magOffsetAutocalMin[i] = 40000;
    magOffsetAutocalMax[i] = -40000;
    mag_bias[i] = 0;
    mbias[i] = 0.0f;
    meanMag[i] = 0.0f;
  }
  stableMeanMag = false;
  hardIronOK = false;
  softIronOK = false;
  stableMeanMagCounter = millis();
  magSampleCount = 1;
  resetSoftIron();
}

void motionCore::resetSoftIron(void) {
  alphaSmoother = MEAN_SMOOTHER_ALPHA;
  meanMag[0] = (float)magX;
  meanMag[1] = (float)magY;
  meanMag[2] = (float)magZ;
    
  for(int i = 0 ; i < SCATTER_PARAM_COUNT ; i++) {
    for(int j = 0 ; j < SCATTER_PARAM_COUNT ; j++) {
      scatterMatrix[i][j] = 0.0f;
    }
  }
}


// Reset gyro offset auto calibration / min / max
void motionCore::resetAccOffsetCalibration(void) {
  for(int i = 0 ; i<3 ; i++) {                            
    accel_bias[i] = 0;
    abias[i] = 0.0f;
    accOffsetAutocalSum[i] = 0;
  }
}





motionCore motion;
