// IMU / sensors Driver
// Emmanuel FLETY - March 2024

#include "sensors.h"

////////////////////////////////////////////////////////////////////////////////:
// LSM6D IMU (Acc + Gyro)
bool imu::begin(uint8_t pin) {
  _pin = pin;
  pinMode(_pin,OUTPUT);
  digitalWrite(_pin, HIGH);

  // Performs default init
  Serial.printf("Init LSM6D IMU\n");
  
  // Check WHO-AM-I - Manufactured RIOT v3 units use a LSM6DSL which should reply with 0x6A
  uint8_t xgTest = xgReadByte(LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
  Serial.printf("Sensor type (WHO_AM_I Token) = 0x%02X\n", xgTest);
  
  switch(xgTest) {    
    case WHO_AM_I_LSM6D_RSP1 :      // LSM6DS3US 
      _imuType = IMU_LSM6DS3US;
      break;
      
    case WHO_AM_I_LSM6D_RSP2 :      // LSM6DSL
      _imuType = IMU_LSM6DSL;
      break;
      
    case WHO_AM_I_LSM6D_RSP3 :      // LSM6DSR
      _imuType = IMU_LSM6DSR;
      break;
  
    case WHO_AM_I_LSM6D_RSP4 :      // LSM6DSV(16X)
      _imuType = IMU_LSM6DSV;
      break;
      
    default:
      _imuType = IMU_UNKNOWN;
      break;
  }

  // We initialize the sensors (accel + gyro) in normal mode to get the proper 16 bit resolution
  // We use an ODR of 400 as we used to sample the motion sensor as low as 2-3ms
  // All axis X,Y,Z enabled
  // Mandatory config. By default, the accelerometer is in high perf mode, so no config needed on CTRL6
  if(_imuType == IMU_LSM6DSV) {
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL1_XL, 0b00001000);  // Accel Hi perf. mode / ODR 480 Hz 
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL2_G,  0b01101100);  // Accel Hi perf. mode / ODR 480 Hz
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL3_C, 0b00000100);   // No block update (continuous mode) + auto increment
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL4_C, 0b00000000);  // No interrupts
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL5_C, 0b00000000);
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL6_G, 0b00000100);  // 2000 °/s
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL7_G, 0b00000000);  // No LPF1 on gyro
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL8_XL, 0b00000010);  // +-8g (no LPF2 - CTRL9) - LPF1 = ODR/2      
  }
  else {
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL1_XL, 0b01101100);  // ODR 416 Hz / +-8g / antialiazing filter 416 Hz (ODR based)   
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL2_G,  0b01101100);  // Gyro ODR 416 Hz / 2000°/s  
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL3_C, 0b00000100);  // No Block update / address auto inc (default)
    xgWriteByte(LSM6DS3_ACC_GYRO_CTRL4_C, 0b00000100);   // SPI only, no I2C, no bandwidth control (only ODR LPF) - LPF disabled
  }

  // Todo : test Gyro HPF (CTRL_7G) to see if we can get a better stability with the orientation filter
  
  if(_imuType != IMU_UNKNOWN) {
    Serial.printf("Init LSM6D IMU: Success\n");
    _initialized = true;
  }
 
  return _initialized;
}

void imu::xgWriteByte(uint8_t subAddress, uint8_t data) {
  digitalWrite(_pin, LOW); // Initiate communication
  // If write, bit 0 (MSB) should be 0
  // If single write, bit 1 should be 0
  SPI.transfer(subAddress & 0x3F); // Send Address
  SPI.transfer(data); // Send data
  digitalWrite(_pin, HIGH); // Close communication
}

uint8_t imu::xgReadByte(uint8_t subAddress) {
  uint8_t temp;
  xgReadBytes(subAddress, &temp, 1);
  return temp;
}

uint8_t imu::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {
  // To indicate a read, set bit 0 (msb) of first byte to 1
  uint8_t rAddress = 0x80 | subAddress;
  digitalWrite(_pin, LOW); // Initiate communication
  SPI.transfer(rAddress);
  for (int i=0; i<count; i++) {
    dest[i] = SPI.transfer(0x00); // Read into destination array
  }
  digitalWrite(_pin, HIGH); // Close communication
  return count;
}

void imu::readAcc() {
  // We don't use the xgReadByte() to save on transaction time
  // Still, this is individual transaction, wasting 24µs per SPI.transfer()
  digitalWrite(_pin, LOW);
  SPI.transfer(LSM6DS3_ACC_GYRO_OUTX_L_XL | READ_AND_AUTOINCREMENT);
  accX.Val[0] = SPI.transfer(0xFF);
  accX.Val[1] = SPI.transfer(0xFF);

  accY.Val[0] = SPI.transfer(0xFF);
  accY.Val[1] = SPI.transfer(0xFF);
  
  accZ.Val[0] = SPI.transfer(0xFF);
  accZ.Val[1] = SPI.transfer(0xFF);
  
  digitalWrite(_pin, HIGH);
}

void imu::readGyro() {
  // We don't use the xgReadByte() to save on transaction time
  // Still, this is individual transaction, wasting 24µs per SPI.transfer()
  digitalWrite(_pin, LOW);
  SPI.transfer(LSM6DS3_ACC_GYRO_OUTX_L_G | READ_AND_AUTOINCREMENT);
  // TODO : check GYRO Axis + deal with module orientation for signs
  // Define a sign matrix in motion class
  gyrX.Val[0] = SPI.transfer(0xFF);
  gyrX.Val[1] = SPI.transfer(0xFF);

  gyrY.Val[0] = SPI.transfer(0xFF);
  gyrY.Val[1] = SPI.transfer(0xFF);
  
  gyrZ.Val[0] = SPI.transfer(0xFF);
  gyrZ.Val[1] = SPI.transfer(0xFF);
  digitalWrite(_pin, HIGH);
}

void imu::readTemp() {
  // We don't use the xgReadByte() to save on transaction time
  // Still, this is individual transaction, wasting 24µs per SPI.transfer()
  digitalWrite(_pin, LOW);
  SPI.transfer(LSM6DS3_ACC_GYRO_OUT_TEMP_L | READ_AND_AUTOINCREMENT);
  temperature.Val[0] = SPI.transfer(0xFF);
  temperature.Val[1] = SPI.transfer(0xFF);
  digitalWrite(_pin, HIGH);
}

void imu::read() {
  // https://docs.espressif.com/projects/esp-idf/en/v3.3.6/api-reference/peripherals/spi_master.html
  // Transaction time is huge on the EPS32 (about 24µs). It's more interesting to have grouped data
  // within a single transaction to reduce acquisition time of the sensors
  digitalWrite(_pin, LOW);
  /*SPI.transfer(LSM6DS3_ACC_GYRO_OUT_TEMP_L | READ_AND_AUTOINCREMENT);
  temperature.Val[0] = SPI.transfer(0xFF);
  temperature.Val[1] = SPI.transfer(0xFF);

  gyrX.Val[0] = SPI.transfer(0xFF);
  gyrX.Val[1] = SPI.transfer(0xFF);

  gyrY.Val[0] = SPI.transfer(0xFF);
  gyrY.Val[1] = SPI.transfer(0xFF);
  
  gyrZ.Val[0] = SPI.transfer(0xFF);
  gyrZ.Val[1] = SPI.transfer(0xFF);

  accX.Val[0] = SPI.transfer(0xFF);
  accX.Val[1] = SPI.transfer(0xFF);

  accY.Val[0] = SPI.transfer(0xFF);
  accY.Val[1] = SPI.transfer(0xFF);
  
  accZ.Val[0] = SPI.transfer(0xFF);
  accZ.Val[1] = SPI.transfer(0xFF);*/
  SPI.transferBytes(spiBufferOut, spiBufferIn, 15);
  temperature.Val[0] = spiBufferIn[1];
  temperature.Val[1] = spiBufferIn[2];
  
  gyrX.Val[0] = spiBufferIn[3];
  gyrX.Val[1] = spiBufferIn[4];

  gyrY.Val[0] = spiBufferIn[5];
  gyrY.Val[1] = spiBufferIn[6];
  
  gyrZ.Val[0] = spiBufferIn[7];
  gyrZ.Val[1] = spiBufferIn[8];

  accX.Val[0] = spiBufferIn[9];
  accX.Val[1] = spiBufferIn[10];

  accY.Val[0] = spiBufferIn[11];
  accY.Val[1] = spiBufferIn[12];
  
  accZ.Val[0] = spiBufferIn[13];
  accZ.Val[1] = spiBufferIn[14];
  digitalWrite(_pin, HIGH);
  
}

///////////////////////////////////////////////////////////////////////////////////////
// Mag Sensor
bool mag::begin(uint8_t pin) {
  _pin = pin;
  pinMode(_pin,OUTPUT);
  digitalWrite(_pin, HIGH);

  // Performs default init
  Serial.printf("Init LIS3MDL Mag sensor\n");
  // Check WHO-AM-I - Manufactured RIOT v3 units use a LIS3MDL which should reply with 0x3D
  uint8_t mTest = mReadByte(LIS3MDL_REG_WHO_AM_I);
  if(mTest == WHO_AM_I_LIS3MDL) {
    _initialized = true;
    Serial.printf("Sensor type (WHO_AM_I Token) = 0x%02X\n", mTest);
    Serial.printf("Init LIS3MDL : Success\n");
  }

  // Magneto configuration
  mWriteByte(LIS3MDL_REG_CTRL_REG1, 0b01010110); // Temp. sensor dis. / Hi Perf + Fast ODR = 300 Hz
  mWriteByte(LIS3MDL_REG_CTRL_REG2, 0b00000000); // Full scale, +- 4 Gauss
  mWriteByte(LIS3MDL_REG_CTRL_REG3, 0b00000000); // SPI 4 wires, continuous conversion (fast ODR)
  mWriteByte(LIS3MDL_REG_CTRL_REG4, 0b00001000); // Z axis Hi Perf + BLE 0
  mWriteByte(LIS3MDL_REG_CTRL_REG5, 0b00000000); // Block update = 0 (continous update)
     
  return _initialized;
}

void mag::mWriteByte(uint8_t subAddress, uint8_t data) {
     digitalWrite(_pin, LOW); // Initiate communication
    // If write, bit 0 (MSB) should be 0
    // If single write, bit 1 should be 0
    SPI.transfer(subAddress & 0x3F); // Send Address
    SPI.transfer(data); // Send data
    digitalWrite(_pin, HIGH); // Close communication
  }

uint8_t mag::mReadByte(uint8_t subAddress) {
  uint8_t temp;
  mReadBytes(subAddress, &temp, 1);
  return temp;
}

uint8_t mag::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {
  // To indicate a read, set bit 0 (msb) of first byte to 1
  uint8_t rAddress = 0x80 | subAddress;
  digitalWrite(_pin, LOW); // Initiate communication
  SPI.transfer(rAddress);
  for (int i=0; i<count; i++) {
    dest[i] = SPI.transfer(0x00); // Read into destination array
  }
  digitalWrite(_pin, HIGH); // Close communication
  return count;
}

void mag::read() {
  // We don't use the mReadByte() to save on transaction time
  // https://docs.espressif.com/projects/esp-idf/en/v3.3.6/api-reference/peripherals/spi_master.html
  // Transaction time is huge on the EPS32 (about 24µs). It's more interesting to have grouped data
  // within a single transaction to reduce acquisition time of the sensors
  digitalWrite(_pin, LOW);
  SPI.transferBytes(spiBufferOut, spiBufferIn, 7);
  magX.Val[0] = spiBufferIn[1];
  magX.Val[1] = spiBufferIn[2];

  magY.Val[0] = spiBufferIn[3];
  magY.Val[1] = spiBufferIn[4];
  
  magZ.Val[0] = spiBufferIn[5];
  magZ.Val[1] = spiBufferIn[6];
  /*SPI.transfer(LIS3MDL_REG_OUT_X_L | MAG_READ_AND_AUTOINCREMENT);
  magX.Val[0] = SPI.transfer(0xFF);
  magX.Val[1] = SPI.transfer(0xFF);

  magY.Val[0] = SPI.transfer(0xFF);
  magY.Val[1] = SPI.transfer(0xFF);
  
  magZ.Val[0] = SPI.transfer(0xFF);
  magZ.Val[1] = SPI.transfer(0xFF);*/
  
  digitalWrite(_pin, HIGH);
}

void mag::readTemp() {  
  digitalWrite(_pin, LOW);
  SPI.transfer(LIS3MDL_REG_TEMP_L | READ_AND_AUTOINCREMENT);
  temperature.Val[0] = SPI.transfer(0xFF);
  temperature.Val[1] = SPI.transfer(0xFF);
  digitalWrite(_pin, HIGH);
}


///////////////////////////////////////////////////////////////////////////////////////
// Baro Sensor - BMP390
bool baro::begin(uint8_t pin) {
  _pin = pin;
  pinMode(_pin,OUTPUT);
  digitalWrite(_pin, HIGH);

 // Performs default init
  Serial.printf("Init BMP390 Baro sensor\n");
  // Check WHO-AM-I - Manufactured RIOT v3 units use a BMP390 which should reply with 0x60
  uint8_t bTest = bReadByte(BMP3XX_CHIP_ID);
  if(bTest == BMP390L_ID) {
    _initialized = true;
    Serial.printf("Sensor type (WHO_AM_I Token) = 0x%02X\n", bTest);
    Serial.printf("Init BMP390 : Success\n");
  }

  // Baro configuration
  samplingMode = BARO_NORMAL_PRECISION2;
  bRes = 0.016f;    // Res over 21 bits - actual available bits depend on ODR / OSR
  refPressure = STANDARD_SEA_LEVEL_PRESSURE_PA;
  bWriteByte(BMP3XX_IF_CONF, 0b00000000);     // SPI 4-wire
  bWriteByte(BMP3XX_PWR_CTRL, 0b00110011);    // Pressure On, Temp On, Normal mode
  delay(50);
  applySamplingMode();
  readCalibration();    
  return _initialized;
}

void baro::setOSR(uint8_t osr) {
  bWriteByte(BMP3XX_OSR, osr);
}

void baro::setODR(uint8_t odr) {
  bWriteByte(BMP3XX_ODR, odr);
}

void baro::setIIR(uint8_t iir) {
  bWriteByte(BMP3XX_IIR_CONFIG, iir);
}

void baro::applySamplingMode() {
  Serial.printf("Apply Baro Sampling mode : %u\n", samplingMode);
  switch(samplingMode) {
    case BARO_ULTRA_LOW_PRECISION:
      setOSR(BARO_OSR_MODE1 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_0P01_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_0);
      break;

    case BARO_LOW_PRECISION:
      setOSR(BARO_OSR_MODE2 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_100_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_0);
      break;

    case BARO_NORMAL_PRECISION1:
      setOSR(BARO_OSR_MODE4 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_50_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_3);
      break;

    case BARO_NORMAL_PRECISION2:
      setOSR(BARO_OSR_MODE8 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_50_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_3);
      break;

    case BARO_HIGH_PRECISION:
      setOSR(BARO_OSR_MODE8 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_12P5_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_1);
      break;

    case BARO_ULTRA_PRECISION:
      setOSR(BARO_OSR_MODE16 | TEMP_OSR_MODE1);
      setODR(BMP3XX_ODR_25_HZ);
      setIIR(BMP3XX_IIR_CONFIG_COEF_3);
      break;

    default:
      samplingMode = BARO_NORMAL_PRECISION1;
      break;
  }
}

void baro::bWriteByte(uint8_t subAddress, uint8_t data) {
  digitalWrite(_pin, LOW); // Initiate communication
  // If write, bit 0 (MSB) should be 0
  // If single write, bit 1 should be 0
  SPI.transfer(subAddress & 0x7F); // Send Address
  SPI.transfer(data); // Send data
  digitalWrite(_pin, HIGH); // Close communication
}

uint8_t baro::bReadByte(uint8_t subAddress) {
  uint8_t temp;
  bReadBytes(subAddress, &temp, 1);
  return temp;
}

uint8_t baro::bReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count) {
  // To indicate a read, set bit 0 (msb) of first byte to 1
  uint8_t rAddress = subAddress | READ_AND_AUTOINCREMENT;
  digitalWrite(_pin, LOW); // Initiate communication
  SPI.transfer(rAddress);
  SPI.transfer(0x00);     // One dummy byte - Datasheet page 43
  for (int i=0; i<count; i++) {
    dest[i] = SPI.transfer(0x00); // Read into destination array
  }
  digitalWrite(_pin, HIGH); // Close communication
  return count;
}

// Combo read of temperature + pressure + calibration with fast SPI packed-up transaction
// https://docs.espressif.com/projects/esp-idf/en/v3.3.6/api-reference/peripherals/spi_master.html
  // Transaction time is huge on the EPS32 (about 24µs). It's more interesting to have grouped data
  // within a single transaction to reduce acquisition time of the sensors
float baro::readPressure() {
  /*uint8_t buf[6] = {0}; 
  bReadBytes(BMP3XX_P_DATA_PA, buf, 6);
  pressureRaw = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
  temperatureRaw = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);*/
  digitalWrite(_pin, LOW);
  SPI.transferBytes(spiBufferOut, spiBufferIn, 8);
  pressureRaw = (uint32_t)spiBufferIn[2] | ((uint32_t)spiBufferIn[3] << 8) | ((uint32_t)spiBufferIn[4] << 16);
  temperatureRaw = (uint32_t)spiBufferIn[5] | ((uint32_t)spiBufferIn[6] << 8) | ((uint32_t)spiBufferIn[7] << 16);
  digitalWrite(_pin, HIGH);
  calibrateTemp();
  calibratePressure();
  return(pressure);
}

// Calibration of temperature as per section 8.5 / page 55
void baro::calibrateTemp() {
  partialData1 = (float)(temperatureRaw - calibData.parT1);
  partialData2 = (float)(partialData1 * calibData.parT2);

  /* Update the compensation temperature in the correction structure, which needs to be used to calculate the pressure */
  temperature = partialData2 + ((partialData1 * partialData1) * calibData.parT3);
}


// Calibration of pressure as per section 8.6 / page 56
// Call of calibrateTemp() must be executed before
void baro::calibratePressure() {
  partialData1 = calibData.parP6 * temperature;     // Compensated temperature
  partialData2 = calibData.parP7 * (temperature * temperature);
  partialData3 = calibData.parP8 * (temperature * temperature * temperature);
  partialOut1 = calibData.parP5 + partialData1 + partialData2 + partialData3;

  partialData1 = calibData.parP2 * temperature;
  partialData2 = calibData.parP3 * (temperature * temperature);
  partialData3 = calibData.parP4 * (temperature * temperature * temperature);
  partialOut2 = (float)pressureRaw * (calibData.parP1 + partialData1 + partialData2 + partialData3);

  partialData1 = (float)pressureRaw * (float)pressureRaw;
  partialData2 = calibData.parP9 + (calibData.parP10 * temperature);
  partialData3 = partialData1 * partialData2;
  partialData4 = partialData3 + ((float)pressureRaw * (float)pressureRaw * (float)pressureRaw) * calibData.parP11;
  pressure = partialOut1 + partialOut2 + partialData4;
  pressure = pressure - refPressure + STANDARD_SEA_LEVEL_PRESSURE_PA;
}


// Read pressure must be called before
// https://rechneronline.de/barometer/
// https://en.wikipedia.org/wiki/Barometric_formula
// Using subscript 0 for altitudes {0;11km} where Sea Level pressure is 1013.25 hPa
// and temperature gradient is 0.0065 K/m
float baro::readAltitude() {
  // BEWARE pow is CPU greedy. exp/log method takes 60µs@80MHz while pow() takes 69µs
  // Use optimized version since this calculation is called live and often
  altitude = (1.0 - expf(logf(pressure / STANDARD_SEA_LEVEL_PRESSURE_PA) * 0.190284f)) * 44307.7f;
  //altitude = (1.0 - powf(pressure / STANDARD_SEA_LEVEL_PRESSURE_PA, 0.190284f)) * 44307.7f;
  return(altitude);
}

// Calculates the current sea pressure refence from a reference altitude and its matching temperature compensated
// pressure reading - This adds an offset to the ref sea level and sets the ref altitude as the current one (QFE)
float baro::setRefAltitude(float alt) {
  refAltitude = alt;
  // Resets reference pressure to get a standardized pressure reading
  refPressure = STANDARD_SEA_LEVEL_PRESSURE_PA;
  readPressure();
  refPressure = (pressure / pow(1.0 - (refAltitude / 44307.7), 5.255302));
  Serial.printf("@Current pressure %f Pa : Reference altitude %f m <=> Sea level pressure = %f Pa \n", pressure, alt, refPressure);
  return(refPressure);
}


// Grabs calibration data and compute float coefficients
// Using double precision calculation then cast to float
void baro::readCalibration() {
  uint8_t regData[BMP3XX_CALIB_DATA_LEN];
  bReadBytes(BMP3XX_CALIB_DATA, regData, BMP3XX_CALIB_DATA_LEN);

  // 1 / 2^8 = 0.00390625f;
  calibDataRaw.parT1 = BMP3XX_CONCAT_BYTES(regData[1], regData[0]);
  calibData.parT1 = ((float)calibDataRaw.parT1 / pow(2, -8));
  // 1073741824.0f;
  calibDataRaw.parT2 = BMP3XX_CONCAT_BYTES(regData[3], regData[2]);
  calibData.parT2 = ((float)calibDataRaw.parT2 / pow(2, 30));
  // 281474976710656.0f;
  calibDataRaw.parT3 = (int8_t)regData[4];
  calibData.parT3 = ((float)calibDataRaw.parT3 / pow(2, 48));
  // 1048576.0f;
  calibDataRaw.parP1 = (int16_t)BMP3XX_CONCAT_BYTES(regData[6], regData[5]);
  calibData.parP1 = ((float)(calibDataRaw.parP1 - (16384)) / pow(2, 20));
  // 536870912.0f;
  calibDataRaw.parP2 = (int16_t)BMP3XX_CONCAT_BYTES(regData[8], regData[7]);
  calibData.parP2 = ((float)(calibDataRaw.parP2 - (16384)) / pow(2, 29));
  // 4294967296.0f;
  calibDataRaw.parP3 = (int8_t)regData[9];
  calibData.parP3 = ((float)calibDataRaw.parP3 / pow(2, 32));
  // 137438953472.0f;
  calibDataRaw.parP4 = (int8_t)regData[10];
  calibData.parP4 = ((float)calibDataRaw.parP4 / pow(2, 37));

  // 1 / 2^3 = 0.125f;
  calibDataRaw.parP5 = BMP3XX_CONCAT_BYTES(regData[12], regData[11]);
  calibData.parP5 = ((float)calibDataRaw.parP5 / pow(2, -3));
  // 64.0f;
  calibDataRaw.parP6 = BMP3XX_CONCAT_BYTES(regData[14], regData[13]);
  calibData.parP6 = ((float)calibDataRaw.parP6 / pow(2, 6));
  // 256.0f;
  calibDataRaw.parP7 = (int8_t)regData[15];
  calibData.parP7 = ((float)calibDataRaw.parP7 / pow(2, 8));
  // 32768.0f;
  calibDataRaw.parP8 = (int8_t)regData[16];
  calibData.parP8 = ((float)calibDataRaw.parP8 / pow(2, 15));
  // 281474976710656.0f;
  calibDataRaw.parP9 = (int16_t)BMP3XX_CONCAT_BYTES(regData[18], regData[17]);
  calibData.parP9 = ((float)calibDataRaw.parP9 / pow(2, 48));
  // 281474976710656.0f;
  calibDataRaw.parP10 = (int8_t)regData[19];
  calibData.parP10 = ((float)calibDataRaw.parP10 / pow(2, 48));
  // 36893488147419103232.0f;
  calibDataRaw.parP11 = (int8_t)regData[20];
  calibData.parP11 = ((float)calibDataRaw.parP11 / pow(2, 65));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensors instanciation
imu lsm6d;
mag lis3mdl;
baro bmp390;
Simple_BNO055 bno055;
