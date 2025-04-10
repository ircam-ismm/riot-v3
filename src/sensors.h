#ifndef	_IMU_H
#define	_IMU_H

// IMU / sensors Driver
// Emmanuel FLETY - March 2024

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "./src/Simple_BNO055.h"
#include "main.h"

// Define of the chip select pins
#define PIN_CS_ACC_GYR    34
#define PIN_CS_MAG        33
#define PIN_CS_ATM        21


enum s_sensorConnectionMode {
  SPI_MODE = 0,
  I2C_MODE,
  AUTO_MODE,
};

#define READ_AND_AUTOINCREMENT          0b10000000
#define MAG_READ_AND_AUTOINCREMENT      0b11000000
#define JUST_READ                       0b10000000


#define LSM_BIAS_TEMPERATURE            25
#define LSM6DSL_TEMP_SCALE              256 // 256 LSB / °C

// Register map definition
// LSM6D
/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_TEST_PAGE        0X00
#define LSM6DS3_ACC_GYRO_RAM_ACCESS       0X01
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME     0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN     0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1       0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2       0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3       0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4       0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5       0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G       0X0B
#define LSM6DS3_ACC_GYRO_REFERENCE_G        0X0C
#define LSM6DS3_ACC_GYRO_INT1_CTRL        0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL        0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG       0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL       0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G        0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C        0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C        0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C        0X14
#define LSM6DS3_ACC_GYRO_CTRL6_G        0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G        0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL       0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL       0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C       0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG      0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC        0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC        0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC        0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG       0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L       0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H       0X21
#define LSM6DS3_ACC_GYRO_OUTX_L_G       0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G       0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G       0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G       0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G       0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G       0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL        0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL        0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL        0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL        0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL        0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL        0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG     0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG     0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG     0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG     0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG     0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG     0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG     0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG     0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG     0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG      0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG      0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG      0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1       0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2       0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3       0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4       0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L      0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H      0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG     0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG     0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG     0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L     0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H     0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC       0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1       0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D       0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2       0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS        0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR        0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL        0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG        0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG        0X5F

/************** Access Device RAM  *******************/
#define LSM6DS3_ACC_GYRO_ADDR0_TO_RW_RAM         0x62
#define LSM6DS3_ACC_GYRO_ADDR1_TO_RW_RAM         0x63
#define LSM6DS3_ACC_GYRO_DATA_TO_WR_RAM          0x64
#define LSM6DS3_ACC_GYRO_DATA_RD_FROM_RAM        0x65

#define LSM6DS3_ACC_GYRO_RAM_SIZE                4096

/************** Embedded functions register mapping  *******************/
#define LSM6DS3_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DS3_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DS3_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DS3_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DS3_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DS3_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DS3_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DS3_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DS3_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DS3_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DS3_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DS3_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DS3_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_IIR              0x10
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_ACOS             0x11
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_WTIME            0x12
#define LSM6DS3_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DS3_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DS3_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DS3_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DS3_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DS3_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DS3_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DS3_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DS3_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DS3_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DS3_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DS3_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DS3_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DS3_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_H                   0x32

#define ACC_2G        2
#define ACC_4G        4
#define ACC_8G        8
#define ACC_16G       16
#define GYRO_250DPS   250
#define GYRO_500DPS   500
#define GYRO_1000DPS  1000
#define GYRO_2000DPS  2000

enum s_AccRange {
  ACC_RANGE_2G = 0,
  ACC_RANGE_16G,
  ACC_RANGE_4G,
  ACC_RANGE_8G
};

enum s_GyroRange {
  GYRO_RANGE_250DPS = 0,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
};

enum s_AccRangeDsv {
  ACC_RANGE_DSV_2G = 0,
  ACC_RANGE_DSV_4G,
  ACC_RANGE_DSV_8G,
  ACC_RANGE_DSV_16G
};

enum s_GyroRangeDsv {
  GYRO_RANGE_DSV_125DPS = 0,
  GYRO_RANGE_DSV_250DPS,
  GYRO_RANGE_DSV_500DPS,
  GYRO_RANGE_DSV_1000DPS,
  GYRO_RANGE_DSV_2000DPS,
  GYRO_RANGE_DSV_4000DPS
};



////////////////////////////////
// LSM6D WHO_AM_I Responses   //
////////////////////////////////
#define WHO_AM_I_LSM6D_RSP1    0x69   // LSM6DS3US 
#define WHO_AM_I_LSM6D_RSP2    0x6A   // LSM6DSL
#define WHO_AM_I_LSM6D_RSP3    0x6B   // LSM6DSR
#define WHO_AM_I_LSM6D_RSP4    0x70   // LSM6DSV(16X)


///////////////////////////////////////////////////////////////////////////////////////
// LIS3MDL 3 axis Mag sensor 
#define LIS3MDL_REG_WHO_AM_I    0x0F    ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1   0x20    ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2   0x21    ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3   0x22    ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4   0x23    ///< Register address for control 4
#define LIS3MDL_REG_CTRL_REG5   0x24    ///< Register address for control 5
#define LIS3MDL_REG_STATUS      0x27    ///< Register address for status
#define LIS3MDL_REG_OUT_X_L     0x28    ///< Register address for X axis lower byte
#define LIS3MDL_REG_TEMP_L      0x2E    ///< Register address for TEMP lower byte
#define LIS3MDL_REG_INT_CFG     0x30    ///< Interrupt configuration register
#define LIS3MDL_REG_INT_THS_L   0x32    ///< Low byte of the irq threshold

#define WHO_AM_I_LIS3MDL        0x3D


#define MAG_4GAUSS    4
#define MAG_8GAUSS    8
#define MAG_12GAUSS   12
#define MAG_16GAUSS   16

enum s_MagRange {
  MAG_RANGE_4GAUSS = 0,
  MAG_RANGE_8GAUSS,
  MAG_RANGE_12GAUSS,
  MAG_RANGE_16GAUSS
};

///////////////////////////////////////////////////////////////////////////////////////
// BMP390
#define BMP390L_ID            0x60   ///< BMP390L chip version

/* BMP3XX register address */
#define BMP3XX_CHIP_ID        0x00   ///< The “CHIP_ID” register contains the chip identification code.
#define BMP3XX_REV_ID         0x01   ///< The “Rev_ID” register contains the mask revision of the ASIC.
#define BMP3XX_ERR_REG        0x02   ///< Sensor Error conditions are reported in the “ERR_REG” register.
#define BMP3XX_STATUS         0x03   ///< The Sensor Status Flags are stored in the “STATUS” register.

#define BMP3XX_P_DATA_PA      0x04   ///< The 24Bit pressure data is split and stored in three consecutive registers.
#define BMP3XX_T_DATA_C       0x07   ///< The 24Bit temperature data is split and stored in three consecutive registersd.
#define BMP3XX_TIME           0x0C    ///< The 24Bit sensor time is split and stored in three consecutive registers.

#define BMP3XX_EVENT          0x10   ///< The “EVENT” register contains the sensor status flags.
#define BMP3XX_INT_STATUS     0x11   ///< The “INT_STATUS” register shows interrupt status and is cleared after reading.
#define BMP3XX_FIFO_LENGTH    0x12   ///< The FIFO byte counter indicates the current fill level of the FIFO buffer.
#define BMP3XX_FIFO_DATA      0x14   ///< The “FIFO_DATA” is the data output register.
#define BMP3XX_FIFO_WTM       0x15   ///< The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
#define BMP3XX_FIFO_WTM2      0x16   ///< The FIFO Watermark size is 9 Bit and therefore written to the FIFO_WTM_0 and FIFO_WTM_1 registers.
#define BMP3XX_FIFO_COFG_1    0x17   ///< The “FIFO_CONFIG_1” register contains the FIFO frame content configuration.
#define BMP3XX_FIFO_COFG_2    0x18   ///< The “FIFO_CONFIG_2” register extends the FIFO_CONFIG_1 register.

#define BMP3XX_INT_CTRL       0x19   ///< Interrupt configuration can be set in the “INT_CTRL” register.
#define BMP3XX_IF_CONF        0x1A   ///< The “IF_CONF” register controls the serial interface settings.
#define BMP3XX_PWR_CTRL       0x1B   ///< The “PWR_CTRL” register enables or disables pressure and temperature measurement.
#define BMP3XX_OSR            0x1C   ///< The “OSR” register controls the oversampling settings for pressure and temperature measurements.
#define BMP3XX_ODR            0x1D   ///< The “ODR” register set the configuration of the output data rates by means of setting the subdivision/subsampling.
#define BMP3XX_IIR_CONFIG     0x1F   ///< The “CONFIG” register controls the IIR filter coefficients
#define BMP3XX_CALIB_DATA     0x31   ///< 0x31-0x45 is calibration data
#define BMP3XX_CMD            0x7E   ///< Command register, can soft reset and clear all FIFO data

#define BMP3XX_ODR_200_HZ         0x00   ///< Prescaler:1; ODR 200Hz; Sampling period:5 ms
#define BMP3XX_ODR_100_HZ         0x01   ///< Prescaler:2; Sampling period:10 ms
#define BMP3XX_ODR_50_HZ          0x02   ///< Prescaler:4; Sampling period:20 ms
#define BMP3XX_ODR_25_HZ          0x03   ///< Prescaler:8; Sampling period:40 ms
#define BMP3XX_ODR_12P5_HZ        0x04   ///< Prescaler:16; Sampling period:80 ms
#define BMP3XX_ODR_6P25_HZ        0x05   ///< Prescaler:32; Sampling period:160 ms
#define BMP3XX_ODR_3P1_HZ         0x06   ///< Prescaler:64; Sampling period:320 ms
#define BMP3XX_ODR_1P5_HZ         0x07   ///< Prescaler:127; Sampling period:640 ms
#define BMP3XX_ODR_0P78_HZ        0x08   ///< Prescaler:256; Sampling period:1.280 s
#define BMP3XX_ODR_0P39_HZ        0x09   ///< Prescaler:512; Sampling period:2.560 s
#define BMP3XX_ODR_0P2_HZ         0x0A   ///< Prescaler:1024 Sampling period:5.120 s
#define BMP3XX_ODR_0P1_HZ         0x0B   ///< Prescaler:2048; Sampling period:10.24 s
#define BMP3XX_ODR_0P05_HZ        0x0C   ///< Prescaler:4096; Sampling period:20.48 s
#define BMP3XX_ODR_0P02_HZ        0x0D   ///< Prescaler:8192; Sampling period:40.96 s
#define BMP3XX_ODR_0P01_HZ        0x0E   ///< Prescaler:16384; Sampling period:81.92 s
#define BMP3XX_ODR_0P006_HZ       0x0F   ///< Prescaler:32768; Sampling period:163.84 s
#define BMP3XX_ODR_0P003_HZ       0x10   ///< Prescaler:65536; Sampling period:327.68 s
#define BMP3XX_ODR_0P0015_HZ      0x11   ///< Prescaler:131072; ODR 25/16384Hz; Sampling period:655.36 s

/* IIR filter coefficient setting constant */
#define BMP3XX_IIR_CONFIG_COEF_0           0x00   ///< Filter coefficient is 0 -> bypass mode
#define BMP3XX_IIR_CONFIG_COEF_1           0x02   ///< Filter coefficient is 1
#define BMP3XX_IIR_CONFIG_COEF_3           0x04   ///< Filter coefficient is 3
#define BMP3XX_IIR_CONFIG_COEF_7           0x06   ///< Filter coefficient is 7
#define BMP3XX_IIR_CONFIG_COEF_15          0x08   ///< Filter coefficient is 15
#define BMP3XX_IIR_CONFIG_COEF_31          0x0A   ///< Filter coefficient is 31
#define BMP3XX_IIR_CONFIG_COEF_63          0x0C   ///< Filter coefficient is 63
#define BMP3XX_IIR_CONFIG_COEF_127         0x0E   ///< Filter coefficient is 127

#define BMP3XX_CALIB_DATA_LEN   (21)   ///< Number of calibration data bytes in the BMP3XX register
#define BMP3XX_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data
#define STANDARD_SEA_LEVEL_PRESSURE_PA  101325.f   ///< Standard sea level pressure, unit: pa

 /*
 * BARO_ULTRA_LOW_PRECISION, ultra-low precision, suitable for weather monitoring (minimum power consumption), power mode is enforcing mode
 * BARO_LOW_PRECISION, low precision, suitable for random detection, power mode is normal mode
 * BARO_NORMAL_PRECISION1, normal accuracy 1, suitable for dynamic detection on handheld devices (such as mobile phones), power mode is normal mode
 * BARO_NORMAL_PRECISION2, normal accuracy 2, suitable for drones, power mode is normal mode
 * BARO_HIGH_PRECISION, high precision, suitable for low-power handheld devices (such as mobile phones), power mode is normal mode
 * BARO_ULTRA_PRECISION, ultra high precision, suitable for indoor guide, the collection rate is very low and the collection period is 1000ms, power mode is normal mode
 */
enum s_BaroSamplingMode {
  BARO_ULTRA_LOW_PRECISION = 0,   // OSRx1, T-OSRx1, ODR 25/2048, no IIR
  BARO_LOW_PRECISION,             // OSRx2, T-OSRx1, ODR 100HZ, no IIR 
  BARO_NORMAL_PRECISION1,         // OSRx4, T-OSRx1, ODR 50HZ, IIR COEF 3 
  BARO_NORMAL_PRECISION2,         // OSRx8, T-OSRx1, ODR 50HZ, IIR COEF 1 (more oversampling, less filtering) 
  BARO_HIGH_PRECISION,            // OSRx8, T-OSRx1, ODR 12.5HZ, IIR COEF 1
  BARO_ULTRA_PRECISION,           // OSRx16, T-OSRx2, ODR 25HZ, IIR COEF 3
  BARO_MAX_SAMPLING_MODE
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor classes
class imu {
public: 

    enum s_imuType {
      IMU_LSM6DS3US,
      IMU_LSM6DSL,
      IMU_LSM6DSR,
      IMU_LSM6DSV,
      IMU_UNKNOWN
    };

    bool begin(uint8_t pin);
    void read();   // Read it all (including temperature)
    void readAcc();
    void readGyro();
    void readTemp();

    // TODO
    void setAccRange(int range);
    void setGyroRange(int range);
    void setGyroHpf(bool hpf);
    //void setAccODR();
    //void setGyrODR();

    int getAccRange() { return accRange; }
    int getGyroRange() { return gyroRange; }
    bool getGyroHpf() { return gyroHpf; }
    int16_t getAccX() { return accX.Value; }
    int16_t getAccY() { return accY.Value; }
    int16_t getAccZ() { return accZ.Value; }
    int16_t getGyrX() { return gyrX.Value; }
    int16_t getGyrY() { return gyrY.Value; }
    int16_t getGyrZ() { return gyrZ.Value; }
    int16_t getTemp() { return temperature.Value; }
     
private:
  void xgWriteByte(uint8_t subAddress, uint8_t data);  
  uint8_t xgReadByte(uint8_t subAddress);  
  uint8_t xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

  const uint8_t spiBufferOut[15] = {LSM6DS3_ACC_GYRO_OUT_TEMP_L | READ_AND_AUTOINCREMENT, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t spiBufferIn[20];
  Word accX, accY, accZ;
  Word gyrX, gyrY, gyrZ;
  Word temperature;
  int accRange, gyroRange;
  bool gyroHpf = false;
  
  bool _initialized = false;
  uint8_t _imuType = IMU_UNKNOWN;
  uint8_t _pin;
};

class mag {
public: 
    bool begin(uint8_t pin);
    void read();   // Read mag only
    void readTemp();

    
    void setRange(int range);
    // TODO
    //void setMagODR();

    int getRange() { return magRange; }
    int16_t getMagX() { return magX.Value; }
    int16_t getMagY() { return magY.Value; }
    int16_t getMagZ() { return magZ.Value; }
    int16_t getTemp() { return temperature.Value; }
     
private:
  void mWriteByte(uint8_t subAddress, uint8_t data);  
  uint8_t mReadByte(uint8_t subAddress);  
  uint8_t mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

  const uint8_t spiBufferOut[7] = {LIS3MDL_REG_OUT_X_L | MAG_READ_AND_AUTOINCREMENT, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t spiBufferIn[10];
  int magRange;
  Word magX, magY, magZ;
  Word temperature;
  
  bool _initialized = false;
  uint8_t _pin;
};


class baro {
public: 
    bool begin(uint8_t pin);
    void readCalibration();
    float setRefAltitude(float alt);
    float readPressure();
    float readAltitude();
    void calibratePressure();
    void calibrateTemp();    

    void setSamplingMode(uint8_t mode) {samplingMode = mode; applySamplingMode(); }
    void setRange(int range);
    void applySamplingMode();
    void setOSR(uint8_t osr);
    void setODR(uint8_t odr);
    void setIIR(uint8_t iir);

    int getRange() { return magRange; }
    float getPressure() { return pressure; }
    float getAltitude() { return altitude; }
    float getTemp() { return temperature; }
    float getRes() { return bRes; }
    uint8_t getSamplingMode() { return samplingMode; }
    float getRefPressure() { return refPressure; }
    float getRefAltitude() { return refAltitude; }

enum s_BaroOSR{
    BARO_OSR_MODE1 = 0,       /**< sampling×1, 16 bit / 2.64 Pa(recommended temperature oversampling×1) */
    BARO_OSR_MODE2,           /**< sampling×2, 16 bit / 2.64 Pa(recommended temperature oversampling×1) */
    BARO_OSR_MODE4,           /**< sampling×4, 18 bit / 0.66 Pa(recommended temperature oversampling×1) */
    BARO_OSR_MODE8,           /**< sampling×8, 19 bit / 0.33 Pa(recommended temperature oversampling×2) */
    BARO_OSR_MODE16,          /**< sampling×16, 20 bit / 0.17 Pa(recommended temperature oversampling×2) */
    BARO_OSR_MODE32,          /**< sampling×32, 21 bit / 0.085 Pa(recommended temperature oversampling×2) */
};

enum s_TempOSR{
    TEMP_OSR_MODE1 = 0<<3,    /**< sampling×1, 16 bit / 0.0050 °C */
    TEMP_OSR_MODE2 = 1<<3,    /**< sampling×2, 16 bit / 0.0025 °C */
    TEMP_OSR_MODE4 = 2<<3,    /**< sampling×4, 18 bit / 0.0012 °C */
    TEMP_OSR_MODE8 = 3<<3,    /**< sampling×8, 19 bit / 0.0006 °C */
    TEMP_OSR_MODE16 = 4<<3,   /**< sampling×16, 20 bit / 0.0003 °C */
    TEMP_OSR_MODE32 = 5<<3,   /**< sampling×32, 21 bit / 0.00015 °C */
};

typedef struct{
    uint16_t parT1;
    uint16_t parT2;
    int8_t parT3;
    int16_t parP1;
    int16_t parP2;
    int8_t parP3;
    int8_t parP4;
    uint16_t parP5;
    uint16_t parP6;
    int8_t parP7;
    int8_t parP8;
    int16_t parP9;
    int8_t parP10;
    int8_t parP11;
  } sCalibData_t;

typedef struct {
    float parT1;
    float parT2;
    float parT3;
    float parP1;
    float parP2;
    float parP3;
    float parP4;
    float parP5;
    float parP6;
    float parP7;
    float parP8;
    float parP9;
    float parP10;
    float parP11;
  } sQuantizedCalibData_t;
    
private:
  void bWriteByte(uint8_t subAddress, uint8_t data);  
  uint8_t bReadByte(uint8_t subAddress);  
  uint8_t bReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);

  uint8_t samplingMode = BARO_NORMAL_PRECISION2;
  uint32_t temperatureRaw;
  uint32_t pressureRaw;
  float bRes, pressure, refPressure, refAltitude, altitude, temperature;

  int magRange;

  const uint8_t spiBufferOut[8] = {BMP3XX_P_DATA_PA | READ_AND_AUTOINCREMENT, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t spiBufferIn[10];

  sCalibData_t calibDataRaw;
  sQuantizedCalibData_t calibData;
  float partialData1, partialData2, partialData3, partialData4, partialOut1, partialOut2;
  
  bool _initialized = false;
  uint8_t _pin;
};

extern imu lsm6d;
extern mag lis3mdl;
extern baro bmp390;
extern Simple_BNO055 bno055;



#endif
