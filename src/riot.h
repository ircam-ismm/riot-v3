#ifndef _RIOT_H
#define _RIOT_H


#include "main.h"
#include "textfile.h"
#include "esp_wifi.h"
#include "motion.h"
#include "routines.h"
#include "osc.h"
#include "web.h"

extern IPAddress defaultIP;
extern IPAddress defaultAccessPointIP;
extern IPAddress defaultSubnetMask;
extern IPAddress defaultGatewayIP;
extern IPAddress defaultDestinationIP;

#define CONNECTING_TIMER_UPDATE   300     // ms
#define CONNECTING_MAX_DOTS       30
#define CHARGING_TIMER_UPDATE     15      // ms
#define POLL_CHARGER_UPDATE       1000    // ms
#define SWITCH_POLLING_PERIOD     10      // ms

#define DEFAULT_CPU_SPEED         240     // MHz
#define DEFAULT_CPU_DOZE          80      // MHz


#define NO_USB_VOLTAGE_THRESHOLD  3.0f    // volts

#define DEFAULT_PLI_LOW           3.5f    // volts
#define DEFAULT_PLI_HIGH          3.7f    // volts
#define MAX_PLI_RANGE             5.0f    // volts

// Slowboot parameter stored in EEPROM (emulated by internal flash)
#define SLOW_BOOT_ADDRESS         0x00
#define MAX_SLOW_BOOT             10000 // ms

#define ODR_LOG_MOTION            50   // ms
#define ODR_STREAMING_LED         20   // ms

#define OSC_DATA_SLOTS            26      // 26 data exported - 9D IMU RAW, 2 switches, pressure, alt, board temp, air temp, Vbatt, Quaternions, Euler+compass
#define OSC_SLOTS_BNO055          4       // yaw, pitch, roll, timestamp

#define OSC_SLOTS_ACCELEROMETER   4     // x, y, z, timestamp
#define OSC_SLOTS_GYROSCOPE       4     // x, y, z, timestamp
#define OSC_SLOTS_MAGNETOMETERS   4     // x, y, z, timestamp
#define OSC_SLOTS_BAROMETER       3     // pressure, temperature, timestamp
#define OSC_SLOTS_TEMPERATURE     2     // onboard temp, barometer temp, timestamp
#define OSC_SLOTS_GRAVITY         4     // x, y, z, timestamp
#define OSC_SLOTS_HEADING         4     // magnetic, geographic, accuracy, timestamp
#define OSC_SLOTS_EULER           4
#define OSC_SLOTS_QUATERNIONS     5
#define OSC_SLOTS_SWITCHES        2

#define OSC_STRING_ACCELEROMETER  "accelerometer"
#define OSC_STRING_GYROSCOPE      "gyroscope"
#define OSC_STRING_MAGNETOMETER   "magnetometer"
#define OSC_STRING_TEMPERATURE    "temperature"
#define OSC_STRING_BAROMETER      "barometer"
#define OSC_STRING_GRAVITY        "gravity"
#define OSC_STRING_HEADING        "heading"
#define OSC_STRING_ORIENTATION    "absoluteorientation"
#define OSC_STRING_EULER          "euler"
#define OSC_STRING_QUATERNION     "quaternion"
#define OSC_STRING_CONTROL        "control"
#define OSC_STRING_KEY            "key"
#define OSC_STRING_BATTERY        "battery"
#define OSC_STRING_ANALOG         "analog"
#define OSC_STRING_BNO055         "bno055"
#define OSC_STRING_MESSAGE        "message"
#define OSC_STRING_API_VERSION    "v3"
#define OSC_STRING_SOURCE         "riot"

           

enum s_riotWifiStateMachine {
  RIOT_DISCONNECTED = 0,
  RIOT_INITIATING_CONNECTION,
  RIOT_CONNECTING,
  RIOT_WAIT_IP,
  RIOT_GOT_IP,
  RIOT_CONNECTED,
  RIOT_LOST_CONNECTION,
};

enum s_riotChargeMode {
  CHARGE_ALWAYS_STREAM = 0,
  CHARGE_NO_STREAM,
  CHARGE_STREAM_IF_ON,
  MAX_CHARGE_MODE
};

enum s_riotChargingStateMachine {
  RIOT_CHARGER_UNPLUGGED = 0,
  RIOT_NOT_CHARGING,
  RIOT_CHARGING,
  RIOT_CHARGING_FINISHED,
};

enum s_chargerState {
  CHARGING_FINISHED = 0,
  CHARGING_IN_PROGRESS,
  BATTERY_DISCONNECTED,
};

enum s_chargerPulse {
  CHARGER_PULSE_UP = 0,
  CHARGER_PULSE_DOWN
};

enum s_riotOperationStateMachine {
  RIOT_IDLE = 0,
  RIOT_STREAMING,           // Not in calibration mode
  RIOT_START_CALIBRATION_ACC_GYR,
  RIOT_CALIBRATION_ACC_GYR,
  RIOT_START_CALIBRATION_MAG,
  RIOT_CALIBRATION_MAG
};


enum s_riotOperationMode {
  STATION_MODE = 0,
  AP_MODE,
};


class riotCore {

public:

  riotCore();
  ~riotCore();

  void init();
  void begin();
  void end();
  void connect();
  void process();
  void printWifiData();
  int getRSSI();
  void printCurrentNet();
  void printMAC();
  void update();
  void calibrate();
  void charge();
  void poll();
  void start();
  void version(bool logToFile = false);
  char* getVersion() { return versionString; }
  void readSlowBoot();
  void writeSlowBoot();  
   
  char* getSSID() { return ssid; }
  char* getPassword() { return password;}
  char* getBonjour() { return mdnsName; }
  IPAddress& getOwnIP() { return localIP; }
  IPAddress& getDestIP() { return destIP; }
  IPAddress& getSubnetMask() { return subnetMask; }
  IPAddress& getGatewayIP() { return gatewayIP; }
  IPAddress& getAPIP() { return accessPointIP; }
  
  uint16_t getDestPort() { return destPort; }
  uint16_t getReceivePort() { return receivePort; }
  
  bool getOperatingMode() { return operatingMode; }
  uint8_t getID() { return moduleID; }
  uint8_t* getMac() { return mac; }
  wifi_power_t getWifiPower() { return wifiPower; }
  int getCalibrationTimer() { return calibrationCountdown; }
  uint8_t getChargingMode() { return chargingMode; }
  uint32_t getSlowBoot() { return slowBoot; }
  uint8_t getChargingState() { return chargingStateMachine; }
  uint8_t getOperationState() { return operationStateMachine; }
  CRGBW8& getPixelColor() { return ledColor; }
  int getCpuSpeed() { return cpuSpeed; }
  int getCpuDoze() { return cpuDoze; }
  float getPliLow() { return pliLow; }
  float getPliHigh() { return pliHigh; }
  char* getOscAddress() { return oscAddressString; }
  void updateStreaming(CRGBW8 color);
  bool pollChargerPlugged();

  void setOwnIP(IPAddress ip) {localIP = ip;}
  void setDestIP(IPAddress ip) {destIP = ip;}
  void setGatewayIP(IPAddress ip) {gatewayIP = ip;}
  void setSubnetMask(IPAddress mask) {subnetMask = mask;}
  void setAPIP(IPAddress ip) {accessPointIP = ip;}
  void setID(uint8_t id) { moduleID = id; }
  void setDestPort(uint16_t port) { destPort = port; }
  void setReceivePort(uint16_t port) { receivePort = port; }
  void setSSID(char *newSSID) { memset(ssid, '\0', sizeof(ssid)); strcpy(ssid, newSSID); }
  void setPassword(char *newPass) { memset(password, '\0', sizeof(password)); strcpy(password, newPass); }
  void setUseDHCP(bool dhcp) { useDHCP = dhcp; }
  void setConfigMode(bool enable) { configurationMode = enable; }
  void setForcedConfigMode(bool enable) { forceConfigMode = enable; }
  void setOperatingMode(uint8_t mode) { operatingMode = mode; }
  void setOscInput(bool acceptOsc) { acceptOSCin = acceptOsc; }
  void setCalibrationTimer(int val) { calibrationCountdown = val; calibrationTimer = millis(); if(val) calibrationEnabled = true; }
  void setChargingMode(uint8_t mode) { chargingMode = mode; }
  void setWifiPower(wifi_power_t power) { wifiPower = power; }
  void setCpuSpeed(int speed) { cpuSpeed = speed; }
  void setCpuDoze(int speed) { cpuDoze = speed; }
  void setSlowBoot(uint32_t del) { slowBoot = del; }
  bool isStation() { return operatingMode == STATION_MODE; }
  void setState(uint8_t state) { state = constrain(state, RIOT_DISCONNECTED, RIOT_LOST_CONNECTION); stateMachine = state; }
  void setOperationState(uint8_t state) { state = constrain(state, RIOT_IDLE, RIOT_CALIBRATION_MAG); operationStateMachine = state; }
  void setChargingState(uint8_t state) { state = constrain(state, RIOT_NOT_CHARGING, RIOT_CHARGING_FINISHED); chargingStateMachine = state; }
  void setDebugMode(bool flag) { debugMode = flag; }
  void setLogMotion(bool flag) { logMotion = flag;}
  void setLogMag(bool flag) { logMag = flag; }
  void setPixelColor(CRGBW8 color) { ledColor = color; }
  void setBonjour(char *name) { strcpy(mdnsName, name); }
  void setPliLow(float thresh) { pliLow = thresh; }
  void setPliHigh(float thresh) { pliHigh = thresh; }
  bool isAP() { operatingMode == AP_MODE; }
  bool isConfig() { return configurationMode; }
  bool isForcedConfig() { return forceConfigMode; }
  bool isDHCP() { return useDHCP; }
  bool isOSCinput() { return acceptOSCin; }
  bool isDebug() { return debugMode; }
  bool isCalibrate() { return calibrationEnabled; }
  bool isCalibrating() { return operationStateMachine > RIOT_STREAMING; }
  bool isConnected() { return (stateMachine == RIOT_CONNECTED); }
  bool isStreaming() { return operationStateMachine == RIOT_STREAMING; }
  bool isIdle() { return operationStateMachine == RIOT_IDLE; }
  bool isCharging() { return chargingStateMachine == RIOT_CHARGING; }
  bool isChargingFinished() { return chargingStateMachine == RIOT_CHARGING_FINISHED; }
  bool isOn() { return powerSwitchStatus; }
  bool isPlugged() { return chargerPlugged;}
  bool isAlwaysStreaming() { return (chargingMode == CHARGE_ALWAYS_STREAM); }
  bool isLogMotion() { return logMotion;}
  bool isLogMag() { return logMag; }
  void bno055Found(bool val) { _bno055Flag = val; }
  void oledFound(bool val) { _oledFlag = val; }
  bool hasBNO055() { return _bno055Flag; }
  bool hasDisplay() { return _oledFlag; }

  uint8_t connectionStatus = WL_IDLE_STATUS;

  Switch onBoardSwitch;
  Switch auxSwitch;

private:
  uint8_t stateMachine = RIOT_DISCONNECTED;
  uint8_t operationStateMachine = RIOT_STREAMING;
  uint8_t chargingStateMachine = RIOT_CHARGER_UNPLUGGED;
  uint8_t chargerStatus = BATTERY_DISCONNECTED;
  bool powerSwitchStatus;
  uint8_t operatingMode = STATION_MODE;
  char ssid[32];
  char ssidAP[32];
  char password[32] = "";
  char mdnsName[32] = DEFAULT_MDNS;
  IPAddress localIP;
  IPAddress accessPointIP;
  IPAddress subnetMask;
  IPAddress gatewayIP;
  IPAddress destIP;
  uint16_t destPort;
  uint16_t receivePort;
  uint8_t moduleID;
  int channel;
  bool hidden;
  wifi_power_t wifiPower;
  uint8_t mac[MAC_SIZE];
  bool useDHCP = true;
  bool configurationMode = false;
  bool forceConfigMode = false;
  bool acceptOSCin = true;
  bool debugMode = false;
  int cpuSpeed = DEFAULT_CPU_SPEED;
  int cpuDoze = DEFAULT_CPU_DOZE;
  int connectingTimer = 0;
  int pollingTimer = 0;
  int chargingTimer = 0;
  int chargerPollTimer = 0;
  int samplingCounter = 0;
  int ledCounter;
  bool chargingPulseDir = CHARGER_PULSE_UP;
  int chargingPulsePWM = 0;
  bool chargerPlugged = false;
  bool ledState;
  CRGBW8 ledColor;
  CRGBW8 chargingColor;
  uint8_t chargingMode = CHARGE_ALWAYS_STREAM;
  uint32_t  slowBoot = 0;
  bool logMotion = false;
  bool logMag = false;

  // Allow config only shortly after start-up
  // Done in the idle 300 ms sample loop hence 17*300 ms = 5100 ms
  int calibrationCountdown = 5000;
  int calibrationTimer;
  bool calibrationEnabled = true;

  int ODR_osc, ODR_logMotion;

  short unsigned int RemoteOutputState = LOW;

  int batteryVoltageRaw;
  BoxFilter<float, 10> batteryVoltageFiltered;
  float batteryVoltage, batterySoC, analogInput1, analogInput2;
  float pliLow, pliHigh;

  uint32_t now;   // time since start to add as timestamp
  
  char versionString[80];
  char fwString[20];
  char dateString[20];
  char oscAddressString[20];
  bool _bno055Flag = false;
  bool _oledFlag = false;

  bool _initialized = false;
  
};


extern riotCore riot;



#endif
