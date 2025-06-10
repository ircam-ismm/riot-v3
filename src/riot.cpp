
// An container object class that deals with / operates the WIFI, UDP and OSC layer of the app
// including disconnect / reconnect etc

#include "riot.h"

void WiFiEvent(WiFiEvent_t event);

IPAddress defaultIP(192, 168, 1, 40);
IPAddress defaultAccessPointIP(192, 168, 3, 1);
IPAddress defaultSubnetMask(255, 255, 255, 0);
IPAddress defaultGatewayIP(192, 168, 1, 1);
IPAddress defaultDestinationIP(192, 168, 1, 100);


riotCore::riotCore() {
}


riotCore::~riotCore() {

}

void riotCore::init() {
  debugMode = false;
  setSSID(DEFAULT_SSID);
  setOwnIP(defaultIP);
  setDestIP(defaultDestinationIP);
  setGatewayIP(defaultGatewayIP);
  setSubnetMask(defaultSubnetMask);
  setAPIP(defaultAccessPointIP);
  setID(DEFAULT_ID);
  setDestPort(DEFAULT_UDP_PORT);
  setReceivePort(DEFAULT_UDP_SERVICE_PORT);
  operatingMode = STATION_MODE;
  useDHCP = true;
  configurationMode = false;
  wifiPower = WIFI_POWER_2dBm;
  channel = 0;  // auto channel after scan for the AP mode
  hidden = false; // visible wifi SSID by default
  RemoteOutputState = LOW;
  chargingMode = CHARGE_STREAM_IF_ON;
  ledColor = Blue;
  pliLow = DEFAULT_PLI_LOW;
  pliHigh = DEFAULT_PLI_HIGH;

  // Global SPI parameters (should work for all sensors)
  // HSPI = SPI2 - VSPI = SPI3
  SPI.setFrequency(8000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(SPI_MSBFIRST);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI); // Very important to specify pins minus _CS so that hwCS isn't used
  Wire.begin();

  pinMode(PIN_NEOPIXEL, OUTPUT);   // RGB Pixel output (WS2812)
  setLedColor(Red);  // RED
  pinMode(PIN_SWITCH_GND, OUTPUT);
  digitalWrite(PIN_SWITCH_GND, LOW);
  pinMode(PIN_OPTION_SW, INPUT_PULLUP);
  pinMode(PIN_CHARGE_STATUS, INPUT);
  pinMode(REMOTE_OUTPUT, OUTPUT);
  digitalWrite(REMOTE_OUTPUT, RemoteOutputState);
  pinMode(39, OUTPUT);

  // Sensors must have all _CS high before init and asap
  pinMode(PIN_CS_ACC_GYR, OUTPUT);
  digitalWrite(PIN_CS_ACC_GYR, HIGH);
  pinMode(PIN_CS_MAG, OUTPUT);
  digitalWrite(PIN_CS_MAG, HIGH);
  pinMode(PIN_CS_ATM, OUTPUT);
  digitalWrite(PIN_CS_ATM, HIGH);
  
  onBoardSwitch.begin(SWITCH_INPUT, MomentarySwitch);
  auxSwitch.begin(SWITCH2_INPUT, MomentarySwitch);
  onBoardSwitch.poll();
  auxSwitch.poll();

  // Analog Inputs
  analogReadResolution(12);
  pinMode(ANALOG_INPUT, INPUT);
  pinMode(ANALOG2_INPUT, INPUT);
  batteryVoltageFiltered.init(readBatteryVoltage());

  if(!EEPROM.begin(20)) {
    Serial.println("Failed to initialize EEPROM");
    slowBoot = 0;
  }
  else {
    readSlowBoot();
  }
}

void riotCore::readSlowBoot() {
  slowBoot = EEPROM.readUInt(SLOW_BOOT_ADDRESS);
  if(slowBoot > MAX_SLOW_BOOT) {
    slowBoot = 0;
    writeSlowBoot();
  }
}

void riotCore::writeSlowBoot() {
  EEPROM.writeUInt(SLOW_BOOT_ADDRESS, slowBoot);
  EEPROM.commit();
}

// Basic inits of the core behaviors go there, OSC mostly
void riotCore::begin() {

  setLedColor(Green);
  // Power can be from 0dBm (min) to 20.5 dBm (max)
  WiFi.setTxPower(wifiPower);

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  // initialise OSC message structures
  if (!isConfig()) {
    if (isStation()) {
      // Attempt to connect to Wifi network:
      Serial.print("R-IoT connecting to: ");
      // print the network name (SSID);
      Serial.println(ssid);
    }
    else { // AP mode
      setLedColor(Red);
      // Attempt to create to Wifi network (soft AP)
      Serial.print("R-IoT creates network: ");
      // print the network name (SSID);
      Serial.println(ssid);

      // Creates the AP & config
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(gatewayIP, gatewayIP, subnetMask);
      WiFi.softAP(ssid, password, channel, hidden);
    }

    char str[MAX_STRING_LEN];

    // OSC address to talk back to the module (commands, ping etc)
    sprintf(oscAddressString, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_MESSAGE);

    // Prepare the new structure of split OSC message
    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_ACCELEROMETER);
    accelerometerOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_GYROSCOPE);
    gyroscopeOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_MAGNETOMETER);
    magnetometerOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_BAROMETER);
    barometerOSC.begin(str, "ffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_GRAVITY);
    gravityOSC.begin(str, "fffi");
    
    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_HEADING);
    headingOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_TEMPERATURE);
    temperatureOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_ORIENTATION, OSC_STRING_EULER);
    eulerOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_ORIENTATION, OSC_STRING_QUATERNION);
    quaternionsOSC.begin(str, "ffffi");

    sprintf(str, "/%s/%s/%d/%s/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_CONTROL, OSC_STRING_KEY);
    controlOSC.begin(str, "ffi");

    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_BATTERY);
    batteryOSC.begin(str, "fii"); // battery State of Charge (SoC) normalized {0;1} + charging or not.
    
    sprintf(str, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_ANALOG);
    analogInputsOSC.begin(str, "fffi"); // Battery voltage, AN0 & AN1

    sprintf(str, "/%s/%s/%d/%s/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_BNO055, OSC_STRING_EULER);
    bno055EulerOSC.begin(str, "fffi");

    sprintf(str, "/%s/%s/%d/%s/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, moduleID, OSC_STRING_BNO055, OSC_STRING_QUATERNION);
    bno055QuatOSC.begin(str, "ffffi");

    uint32_t bundleSize = accelerometerOSC.getSize() + gyroscopeOSC.getSize() + magnetometerOSC.getSize() + barometerOSC.getSize();
    bundleSize += temperatureOSC.getSize() + gravityOSC.getSize() + headingOSC.getSize() + quaternionsOSC.getSize() + eulerOSC.getSize();
    bundleSize += controlOSC.getSize() + analogInputsOSC.getSize() + bno055EulerOSC.getSize() + bno055QuatOSC.getSize();
    bundleOSC.begin(bundleSize);
  }

  // If in configuration mode we setup a webserver for configuring the unit
  // The module becomes an AP with DHCP
  else {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(accessPointIP, accessPointIP, subnetMask);
    uint64_t chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
    uint8_t *id = (uint8_t*)&chipid;
    sprintf(ssidAP, "RIOT-%02x:%02x\0", id[4], id[5]); // Uses the 6 HEX numbers of the MAC address in the AP/SSID name
    Serial.printf("Setting up Access Point named: %s\n", ssidAP);

    WiFi.softAP(ssidAP, DEFAULT_AP_PASSWORD, channel, false);  // hidden = false - Beware, password must be 8 char long minimum
    WiFi.softAPmacAddress(mac);
    Serial.printf("AP MAC %X:%X:%X:%X:%X:%X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  _initialized = true;

}

void riotCore::start(void) {
  if (!isConfig()) {
    stateMachine = RIOT_INITIATING_CONNECTION;
  }
}

void riotCore::connect(void) {

  if (!useDHCP) // DNS = gateway IP
    WiFi.config(localIP, gatewayIP, gatewayIP, subnetMask);
  // This step shouldn't be needed, a blank password should connect without security
  // To double check with ESP32 APIP
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Stores the MAC ADDRESS for further use (like AP naming)
  WiFi.macAddress(mac);
  Serial.printf("Retrieved STA MAC %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  bool logToFile = !checkFile(VERSION_FILE);
  if(logToFile)
    version(true);  // populates the version string (+MAC) + displays it - logs to file if version.txt not present
}


void riotCore::poll() {
  // Xtimer task should take care of speed limiter here
  // but we still prevent a direct call from overriding
  // polling period
  if (millis() - pollingTimer < SWITCH_POLLING_PERIOD)
    return;

  pollingTimer = millis();
  onBoardSwitch.poll();
  auxSwitch.poll();
}

// Processes the wifi state machine
void riotCore::update() {

  static int cnt = 0;
  static bool blinkIt = false;

  // Ressources saver and times info display
  if (millis() - connectingTimer < CONNECTING_TIMER_UPDATE) {
    return;
  }
  connectingTimer = millis();

  switch (stateMachine) {
    case RIOT_DISCONNECTED:
      break;

    case RIOT_INITIATING_CONNECTION:
      Serial.printf("Connecting to WIFI\n");
      connect();
      connectingTimer = millis();
      cnt = 0;
      blinkIt = false;
      stateMachine = RIOT_CONNECTING;
      break;

    case RIOT_CONNECTING:
      Serial.printf(".");
      cnt++;
      if (cnt > CONNECTING_MAX_DOTS) {
        cnt = 0;
        Serial.printf("\n");
      }
      blinkIt = !blinkIt;
      setLedColor(blinkIt ? Green : Black);
      break;

    case RIOT_WAIT_IP:
      if (!isDHCP()) {
        stateMachine = RIOT_GOT_IP;
        break;
      }

      if (millis() - connectingTimer > CONNECTING_TIMER_UPDATE) {
        connectingTimer = millis();
        Serial.printf("x");
        cnt++;
        if (cnt > CONNECTING_MAX_DOTS) {
          cnt = 0;
          Serial.printf("\n");
        }
        blinkIt = !blinkIt;
        setLedColor(blinkIt ? Green : Black);
      }
      if (WiFi.localIP() != INADDR_NONE) {
        stateMachine = RIOT_GOT_IP;
      }
      break;

    case RIOT_GOT_IP:
      Serial.printf("\nGot IP :-)\n");
      localIP = WiFi.localIP();
      printCurrentNet();
      printWifiData();
      udpPacket.begin(localIP, destPort);
      // Open the service port to talk to the module (config, calibration)
      configPacket.begin(receivePort);    // remote control packets are on the same port as dest port
      oscUdp.setDestination(destIP, destPort); // that's more for outgoing packets but configured nonetheless

      stateMachine = RIOT_CONNECTED;
      //setLedColor(Blue);
      Serial.println("\nConnected to the network");
      break;

    case RIOT_CONNECTED:
      break;

    case RIOT_LOST_CONNECTION:
      Serial.printf("WIFI lost connnection or unable to connect\n");
      start();
      break;

    default:
      //stateMachine = RIOT_DISCONNECTED;
      break;
  }  
}

// Processes the operation state machine (streaming vs. calibration)
void riotCore::calibrate() {
  static int winkCounter = 0;
  char str[MAX_STRING_LEN];
  
  switch(operationStateMachine) {
    case RIOT_IDLE:
      break;

    // We autorize changing to calibration only when Streaming (and being connected)
    case RIOT_STREAMING:
      if(calibrationEnabled) {
        if((millis() - calibrationTimer) < calibrationCountdown) {
          if(onBoardSwitch.pressed() || motion.isNextStep()) {
            setLedColor(Red);
            while(onBoardSwitch.pressed())
              delay(20);
            calibrationEnabled = false;
            motion.nextStep(false);
            setLedColor(Yellow);
            sprintf(str, "ACC+GYRO+MAG Calibration Started");
            printToOSC(str);
            Serial.printf("%s\n", str);
            sprintf(str, "Please place the module on a flat and stable surface");
            printToOSC(str);
            Serial.printf("%s\n", str);
            sprintf(str, "Press Switch when ready or type serial command \"GO\"");
            printToOSC(str);
            Serial.printf("%s\n", str);
            setOperationState(RIOT_START_CALIBRATION_ACC_GYR);
          }
        }
        else {
          calibrationEnabled = false;
          Serial.println("Calibration is now disabled");
        }
      }
      break;
      
    case RIOT_START_CALIBRATION_ACC_GYR:  
      if(onBoardSwitch.pressed() || motion.isNextStep()) { 
        while(onBoardSwitch.pressed())
          delay(20);
        motion.resetAccOffsetCalibration();
        motion.resetGyroOffsetCalibration();
        motion.nextStep(false);
        setOperationState(RIOT_CALIBRATION_ACC_GYR);
      }
      break;
      
    case RIOT_CALIBRATION_ACC_GYR:
      if(onBoardSwitch.pressed() || motion.isNextStep()) {
        Serial.println("Acc-Gyro Calibration interrupted");
        while(onBoardSwitch.pressed())
          delay(20);
        motion.nextStep(false);  
        setOperationState(RIOT_STREAMING);
        break;
      } 
      if(motion.calibrateAccGyro()) { // if acc-gyro ended after found stable
        setOperationState(RIOT_START_CALIBRATION_MAG);
        winkCounter = 0;
        setLedColor(White);
        motion.nextStep(false);
        sprintf(str, "MAG calibration - Press Switch (or GO) & Max out all axis");
        Serial.printf("%s\n", str);
        printToOSC(str);
      }
      break;

    case RIOT_START_CALIBRATION_MAG: 
      if(onBoardSwitch.pressed() || motion.isNextStep()) { // Waits for the GP switch to proceed to mag calibration
        while(onBoardSwitch.pressed())
          delay(20);
        motion.resetMagOffsetCalibration();
        motion.nextStep(false);
        setOperationState(RIOT_CALIBRATION_MAG);
        break;
      }
      else {  // burst winking the led, awaiting next calibration step
        delay(20);
        winkCounter++;
        if(winkCounter > 50) {
          setLedColor(White);
          delay(100);
          setLedColor(Black);
          winkCounter = 0;
        }
      }
      break;
      
    case RIOT_CALIBRATION_MAG:
      motion.calibrateMag();
      if(onBoardSwitch.pressed() || motion.isNextStep()) {
        motion.calibrateMag(true);  // stores offsets
        setLedColor(Green);
        while(onBoardSwitch.pressed())
          delay(20);
        storeConfig();
        motion.begin();   // Recomputes offsets
        // End of Calibration
        setOperationState(RIOT_STREAMING);
      }
      break;

    default:
      setOperationState(RIOT_STREAMING);
      break; 
  }
}

void riotCore::charge() {
  // Ressource saver - time grain = PWM update resolution of pixel pulsing during charge
  if((millis() - chargingTimer) < CHARGING_TIMER_UPDATE) {
    return;
  }
  chargingTimer = millis();

  if(isAlwaysStreaming() || isCalibrating())
    return;

  pollChargerPlugged();

  if(!isPlugged()) {
    chargingStateMachine = RIOT_CHARGER_UNPLUGGED;
    return;
  }
    
  if(chargingMode == CHARGE_STREAM_IF_ON) {
    if(isOn()) {
      updateStreaming(chargingColor);
      return;
    }
  }
    
  // Else we process the charging state machine for displaying it on the pixel LED  
  chargerStatus = readChargeStatus();
  //Serial.printf("Charge State = %d / Charge Status = %d\n", chargingStateMachine, chargerStatus);
  
  switch(chargingStateMachine) {
    case RIOT_CHARGER_UNPLUGGED:
      if(chargerPlugged) {
        if(riot.isDebug()) {
          Serial.printf("Charger inserted\n");
        }
        chargingColor = Orange;
        if(chargerStatus == BATTERY_DISCONNECTED) {  // Battery physically disconnected from input
          if(riot.isDebug()) {
            Serial.printf("Battery disconnected\n");
          }
          chargingColor = Red;  
          setChargingState(RIOT_NOT_CHARGING);
        }
        else if(chargerStatus == CHARGING_IN_PROGRESS) {
          if(riot.isDebug()) {
            Serial.printf("Charging\n");
          } 
          chargingColor = Purple;
          setChargingState(RIOT_CHARGING);
          
        }
        else {
          if(riot.isDebug()) {
            Serial.printf("Charging\n");
          }
          chargingColor = Green;
          setChargingState(RIOT_CHARGING_FINISHED); 
        }
        updateStreaming(chargingColor);
      }
      break;

    case RIOT_NOT_CHARGING:
      if(!chargerPlugged) {
        if(riot.isDebug()) {
          Serial.printf("Charger removed\n");
        }
        chargingColor = Yellow;
        setChargingState(RIOT_CHARGER_UNPLUGGED);
        break;
      }
      if(chargerStatus == CHARGING_IN_PROGRESS) {
        if(riot.isDebug()) {
          Serial.printf("Charging\n");
        }
        chargingColor = Purple;
        setChargingState(RIOT_CHARGING);
        break;
      }
      else if(chargerStatus == CHARGING_FINISHED){
         if(riot.isDebug()) {
          Serial.printf("Charging ended\n");
        }
        chargingColor = Green;
        setChargingState(RIOT_CHARGING_FINISHED);
        break;
      }
      updateStreaming(chargingColor);
      break;

    // In the riot-charging state, differentiate the switch on/off by reading the Vbatt-cut pin
    // which drops a lot < 3.5v when switch is off
    case RIOT_CHARGING:
      if(!chargerPlugged) {
        if(riot.isDebug()) {
          Serial.printf("Charger removed\n");
        }
        chargingColor = Yellow;
        setChargingState(RIOT_CHARGER_UNPLUGGED);
        break;
      }
      if(chargerStatus == BATTERY_DISCONNECTED) {
        if(riot.isDebug()) {
          Serial.printf("Battery disconnected\n");
        }
        chargingColor = Red;
        setChargingState(RIOT_NOT_CHARGING);
        break;
      }
      else if(chargerStatus == CHARGING_FINISHED){
         if(riot.isDebug()) {
          Serial.printf("Charging ended\n");
         }
        chargingColor = Green;
        setChargingState(RIOT_CHARGING_FINISHED);
        break;
      }
      // Led Pulsing during charge
      if(chargingPulseDir == CHARGER_PULSE_UP) {
        chargingPulsePWM++;
        if(chargingPulsePWM > 255) {
          chargingPulsePWM = 255;
          chargingPulseDir = CHARGER_PULSE_DOWN;
        }
      }
      else {
        chargingPulsePWM--;
        if(chargingPulsePWM < 0) {
          chargingPulsePWM = 0;
          chargingPulseDir = CHARGER_PULSE_UP;
        }
      }
      //Serial.printf("pwm=%d\n", chargingPulsePWM);
      updateStreaming(chargingColor * chargingPulsePWM);
      break;
      
    case RIOT_CHARGING_FINISHED:
      if(!chargerPlugged) {
        if(riot.isDebug()) {
          Serial.printf("Charger removed\n");
        }
        chargingColor = Yellow;
        setChargingState(RIOT_CHARGER_UNPLUGGED);
        break;
      }
      if(chargerStatus == BATTERY_DISCONNECTED) {
        if(riot.isDebug()) {
          Serial.printf("Battery disconnected\n");
        }
        chargingColor = Red;
        setChargingState(RIOT_NOT_CHARGING);
        break;
      }
      else if(chargerStatus == CHARGING_IN_PROGRESS){
         if(riot.isDebug()) {
          Serial.printf("Charging\n");
        }
        chargingColor = Purple;
        setChargingState(RIOT_CHARGING); 
        break;
      }
      updateStreaming(chargingColor);
      break;

    default:
      setChargingState(RIOT_CHARGER_UNPLUGGED);
      break;
  }
}


// Defines streaming mode based on charging mode & options
void riotCore::updateStreaming(CRGBW8 color) {
  switch(chargingMode) {
    case CHARGE_ALWAYS_STREAM:
      if(isIdle())
        setOperationState(RIOT_STREAMING);
      break;

    case CHARGE_NO_STREAM:
    // update led only when not streaming   
      setLedColor(color);
      if(isCharging() || isChargingFinished())
        setOperationState(RIOT_IDLE);
      else
        setOperationState(RIOT_STREAMING);
      break;

    case CHARGE_STREAM_IF_ON:
      if(isOn()) { // switch is most likely off
        setOperationState(RIOT_STREAMING);
      }
      else {
        setOperationState(RIOT_IDLE);
        //color.print();
        setLedColor(color);
      }
      break;
    
    default:
      chargingMode = CHARGE_ALWAYS_STREAM;
      break;
  }
}


bool riotCore::pollChargerPlugged() {
  if((millis() - chargerPollTimer) < POLL_CHARGER_UPDATE) {
    return chargerPlugged;
  }
  chargerPollTimer = millis();
  chargerPlugged = (readUsbVoltage() > NO_USB_VOLTAGE_THRESHOLD)? true:false;
  powerSwitchStatus = (readBatteryVoltage() > 2.0f) ? true : false;
  //Serial.printf("usb = %f v / charger : %d\n", readUsbVoltage(), chargerPlugged);
  //Serial.printf("Switch On/Off = %d / Vbatt = %f v\n", powerSwitchStatus, readBatteryVoltage()); 
  return chargerPlugged; 
}

// This is where we grab motion data and trigger computations, send OSC
void riotCore::process() {  
  if (!isConnected())
    return;

  if (millis() - samplingCounter < motion.getSampleRate())
    return;

  samplingCounter = millis();  
  //digitalWrite(REMOTE_OUTPUT, HIGH);
  // We speed up the processor during the CPU intensive compute task then sleep the WIFI modem and doze CPU util next time
  wakeModemSleep();
  setLedColor(ledColor);    // Turns blue or specified led color in config
  
  // Debug : use physical output to measure compute / processing duration
  // Durations @240MHz during process() after wake() - Doze off:
  // - digitalWrite :  about 960ns - WakeUpModem() : 120µs - setModemSleep() : 250µs 
  // - update the led color (ws2812 pixel) : takes about 117µs which is huge.
  // - analogRead() : 112µs - Sensors grab (low level drivers + calculations) : 156µs (including pressure computations)
  // - SPI sensors acquisition (almost no math) with SPI packed transactions : 78µs
  // - readPressure() is computation intensive due to float math and expf/logf (72µs total)
  // - Madgwick etc : 188µs
  // - OSC packet forge: 56µs - Wifi UDP packet : 480µs
  // - Full process (acquisition, math, orientation computation) : 1.76ms
  // Current payload (OSC) is 128 bytes approx. + Eth 14 bytes + IP 20 bytes + UDP 8 Bytes ~ 200 bytes total with OSC address

  // Decide whether you prefer the raw voltage or filtered (moving average)
  //batteryVoltage = batteryVoltageFiltered.filter(readBatteryVoltage());
  batteryVoltage = readBatteryVoltage();
  batterySoC = voltageToSoC(batteryVoltage);
  batterySoC = constrain(batterySoC, 0.f, 1.f);
  analogInput1 = (float)analogRead(ANALOG_INPUT) * ANALOG_INPUT_VOLTAGE_SCALE;
  analogInput2 = (float)analogRead(ANALOG2_INPUT) * ANALOG_INPUT_VOLTAGE_SCALE;
  now = millis();
  analogInputsOSC.rewind();
  analogInputsOSC.addFloat(batteryVoltage);
  analogInputsOSC.addFloat(analogInput1);
  analogInputsOSC.addFloat(analogInput2);
  analogInputsOSC.addInt(now); 

  batteryOSC.rewind();
  batteryOSC.addFloat(batterySoC);
  batteryOSC.addInt(isCharging());
  batteryOSC.addInt(now); 

  controlOSC.rewind();
  controlOSC.addFloat((float)onBoardSwitch.pressed());
  controlOSC.addFloat((float)auxSwitch.pressed());
  controlOSC.addInt(now); 

  motion.grab();
  motion.compute();
  now = millis();

  // Live debug to Arduino serial plotter - Raw values of the motion sensors, un calibrated
  if((millis() - ODR_logMotion) > (ODR_LOG_MOTION / motion.getSampleRate())) {
    ODR_logMotion = millis();
    if(isLogMotion()) {
      Serial.printf("%d %d %d ", motion.accX, motion.accY, motion.accZ);
      Serial.printf("%d %d %d\n", motion.gyrX, motion.gyrX, motion.gyrZ);      
    }
    else if(isLogMag()) {
      Serial.printf("%d %d %d\n", motion.magX, motion.magY, motion.magZ);
    }
  }

  // OSC export - multiple layers and structures in one single OSC Bundle
  // Add timetags to the OSC bundle when NTP is there

  // Sensors data order now complies with the W3C device motion standard (order and units)
  // https://www.w3.org/TR/orientation-event/
  // Magnetometers are exported in µT which are 100 Gauss
  accelerometerOSC.rewind();
  accelerometerOSC.addFloat(motion.a_x * G_TO_MS2);  // Range {-8 ; +8} g x 9.81 => m.s-2
  accelerometerOSC.addFloat(motion.a_y * G_TO_MS2);
  accelerometerOSC.addFloat(motion.a_z * G_TO_MS2);
  accelerometerOSC.addInt(now);
  
  gyroscopeOSC.rewind();
  gyroscopeOSC.addFloat(motion.g_x * DEG_TO_RAD); // rad/s (2000° / PI)/s)
  gyroscopeOSC.addFloat(motion.g_y * DEG_TO_RAD);
  gyroscopeOSC.addFloat(motion.g_z * DEG_TO_RAD);
  gyroscopeOSC.addInt(now);

  magnetometerOSC.rewind();
  magnetometerOSC.addFloat(motion.m_x * 100.f);  // Range {-4 ; +4} Gauss <=> {-400 ; +400} µTesla
  magnetometerOSC.addFloat(motion.m_y * 100.f);
  magnetometerOSC.addFloat(motion.m_z * 100.f);
  magnetometerOSC.addInt(now);

  barometerOSC.rewind();
  barometerOSC.addFloat(motion.pressure);
  barometerOSC.addFloat(motion.altitude);
  barometerOSC.addInt(now);
  
  temperatureOSC.rewind();
  temperatureOSC.addFloat(motion.boardTemperature); // Acc sensor
  temperatureOSC.addFloat(motion.temperature);      // Barometer sensor
  temperatureOSC.addFloat(motion.mcuTemperature);   // ESP32-S3 sensor
  temperatureOSC.addInt(now);

  quaternionsOSC.rewind();
  quaternionsOSC.addFloat(motion.q1); // x
  quaternionsOSC.addFloat(motion.q2); // y
  quaternionsOSC.addFloat(motion.q3); // z
  quaternionsOSC.addFloat(motion.q0); // w
  quaternionsOSC.addInt(now);

  eulerOSC.rewind();
  eulerOSC.addFloat(motion.yaw);    // in Degree
  eulerOSC.addFloat(motion.pitch);
  eulerOSC.addFloat(motion.roll);
  eulerOSC.addInt(now);

  if(hasBNO055()) {
    bno055EulerOSC.rewind();
    bno055EulerOSC.addFloat((float)motion.bno055Data[0]); // Yaw
    bno055EulerOSC.addFloat((float)motion.bno055Data[2]); // Pitch
    bno055EulerOSC.addFloat((float)motion.bno055Data[1]); // Roll
    bno055EulerOSC.addInt(now);

    bno055QuatOSC.rewind();
    bno055QuatOSC.addFloat((float)motion.bno055Quat[1]); // x
    bno055QuatOSC.addFloat((float)motion.bno055Quat[2]); // y
    bno055QuatOSC.addFloat((float)motion.bno055Quat[3]); // z
    bno055QuatOSC.addFloat((float)motion.bno055Quat[0]); // w
    bno055QuatOSC.addInt(now);
  }

  gravityOSC.rewind();
  gravityOSC.addFloat(motion.grav_x * G_TO_MS2);
  gravityOSC.addFloat(motion.grav_y * G_TO_MS2);
  gravityOSC.addFloat(motion.grav_z * G_TO_MS2);
  gravityOSC.addInt(now);

  headingOSC.rewind();
  headingOSC.addFloat(motion.heading);    // Magnetic heading (acc+mag) = compass heading
  headingOSC.addFloat(-1.f);              // we don't have geographic heading on RIOT, no GPS avail.
  headingOSC.addFloat(-1.f);              // Accuracy - in degree of accuracy - When iOS is happy : 15° - bad accuracy is more like 50°. -1 for "unknown"
  headingOSC.addInt(now);

  // Create bundle out of all individual OSC message. Here you can cherry pick what you send or not,
  // to save on wifi traffic and/or latency or CPU load
  bundleOSC.rewind();
  bundleOSC.addMessage(accelerometerOSC.getBuffer(), accelerometerOSC.getSize());
  bundleOSC.addMessage(gyroscopeOSC.getBuffer(), gyroscopeOSC.getSize());
  bundleOSC.addMessage(magnetometerOSC.getBuffer(), magnetometerOSC.getSize());
  bundleOSC.addMessage(barometerOSC.getBuffer(), barometerOSC.getSize());
  bundleOSC.addMessage(temperatureOSC.getBuffer(), temperatureOSC.getSize());
  bundleOSC.addMessage(quaternionsOSC.getBuffer(), quaternionsOSC.getSize());
  bundleOSC.addMessage(eulerOSC.getBuffer(), eulerOSC.getSize());
  bundleOSC.addMessage(gravityOSC.getBuffer(), gravityOSC.getSize());
  bundleOSC.addMessage(headingOSC.getBuffer(), headingOSC.getSize());
  if(hasBNO055()) {
    bundleOSC.addMessage(bno055EulerOSC.getBuffer(), bno055EulerOSC.getSize());
    bundleOSC.addMessage(bno055QuatOSC.getBuffer(), bno055QuatOSC.getSize());
  }
  bundleOSC.addMessage(batteryOSC.getBuffer(), batteryOSC.getSize());
  bundleOSC.addMessage(analogInputsOSC.getBuffer(), analogInputsOSC.getSize());
  bundleOSC.addMessage(controlOSC.getBuffer(), controlOSC.getSize());
  
  udpPacket.beginPacket(destIP, destPort);
  udpPacket.write(bundleOSC.getBuffer(), bundleOSC.getSize());
  udpPacket.endPacket();
     
  setLedColor(Black);
  setModemSleep();
  //digitalWrite(REMOTE_OUTPUT, LOW);
}

int riotCore::getRSSI() {
  int rssi = WiFi.RSSI();
  Serial.printf("signal strength (RSSI) in dB: %d\n", rssi);
  return rssi;
}

void riotCore::version(bool logToFile) {
  sprintf(versionString, "%s-%s.%s.%s-%02d-%02d-%d-%u", VERSION, FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH, COMPILE_DAY_INT, COMPILE_MONTH_INT, COMPILE_YEAR_INT, COMPILE_DOS_TIME);
  sprintf(fwString, "%s.%s.%s", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
  sprintf(dateString, "%02d-%02d-%d-%u", COMPILE_DAY_INT, COMPILE_MONTH_INT, COMPILE_YEAR_INT, COMPILE_DOS_TIME);
  Serial.printf("[VERSION] %s\n", versionString);
  Serial.printf("[FW] %s\n", fwString);
  Serial.printf("[DATE] %s\n", dateString);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  if(logToFile) {  
    FIL VersionFile;
    UINT write;
    char str[MAX_STRING_LEN];
    sprintf(str, "/%s", VERSION_FILE);
    if (f_open(&VersionFile, str, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      return;
      
      if(isDebug())
        printf("[LOG] Saving %s\n", VERSION_FILE, str);

      sprintf(str, "R-IoT fw version\r\n");
      strcat(str, versionString);
      strcat(str, TEXT_FILE_EOL);
      f_write(&VersionFile, str, strlen(str), &write);
      sprintf(str, fwString);
      strcat(str, TEXT_FILE_EOL);
      strcat(str, dateString);
      strcat(str, TEXT_FILE_EOL);
      sprintf(str, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      strcat(str, TEXT_FILE_EOL);
      f_write(&VersionFile, str, strlen(str), &write);
      f_close(&VersionFile);
  }
}


void riotCore::printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.printf("SSID: %s\n", WiFi.SSID());

  // print the received signal strength:
  getRSSI();

  // AP information
  wifi_ap_record_t info;
  esp_wifi_sta_get_ap_info(&info);
  Serial.printf("AP PHY:\n");
  Serial.printf("- Channel: %d\n", info.primary);
  Serial.printf("- phy_11b: %d\n", info.phy_11b);
  Serial.printf("- phy_11g: %d\n", info.phy_11g);
  Serial.printf("- phy_11n: %d\n", info.phy_11n);
  Serial.printf("- phy_lr: %d\n", info.phy_lr);

}

void riotCore::printWifiData() {
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  // print mac from mac array for debug only
  //printMAC();

  Serial.print("Dest. IP Address: ");
  Serial.println(destIP);

  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);

  Serial.printf("UDP/OSC Port=%u\n", destPort);
  Serial.printf("Module ID=%u\n", moduleID);
  Serial.printf("Sample Period (ms)=%u\n", motion.getSampleRate());
}

void riotCore::printMAC() {
  Serial.printf("Stored MAC : ");
  for (int i = 0 ; i < MAC_SIZE ; i++) {
    Serial.printf("%02X:", mac[i]);
  }
  Serial.println();
}


/*
  WiFi Events
  0  ARDUINO_EVENT_WIFI_READY               < ESP32 WiFi ready
  1  ARDUINO_EVENT_WIFI_SCAN_DONE                < ESP32 finish scanning AP
  2  ARDUINO_EVENT_WIFI_STA_START                < ESP32 station start
  3  ARDUINO_EVENT_WIFI_STA_STOP                 < ESP32 station stop
  4  ARDUINO_EVENT_WIFI_STA_CONNECTED            < ESP32 station connected to AP
  5  ARDUINO_EVENT_WIFI_STA_DISCONNECTED         < ESP32 station disconnected from AP
  6  ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
  7  ARDUINO_EVENT_WIFI_STA_GOT_IP               < ESP32 station got IP from connected AP
  8  ARDUINO_EVENT_WIFI_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
  9  ARDUINO_EVENT_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
  10 ARDUINO_EVENT_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
  11 ARDUINO_EVENT_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
  12 ARDUINO_EVENT_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
  13 ARDUINO_EVENT_WIFI_AP_START                 < ESP32 soft-AP start
  14 ARDUINO_EVENT_WIFI_AP_STOP                  < ESP32 soft-AP stop
  15 ARDUINO_EVENT_WIFI_AP_STACONNECTED          < a station connected to ESP32 soft-AP
  16 ARDUINO_EVENT_WIFI_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
  17 ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
  18 ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
  19 ARDUINO_EVENT_WIFI_AP_GOT_IP6               < ESP32 ap interface v6IP addr is preferred
  19 ARDUINO_EVENT_WIFI_STA_GOT_IP6              < ESP32 station interface v6IP addr is preferred
  20 ARDUINO_EVENT_ETH_START                < ESP32 ethernet start
  21 ARDUINO_EVENT_ETH_STOP                 < ESP32 ethernet stop
  22 ARDUINO_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
  23 ARDUINO_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
  24 ARDUINO_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
  19 ARDUINO_EVENT_ETH_GOT_IP6              < ESP32 ethernet interface v6IP addr is preferred
  25 ARDUINO_EVENT_MAX
*/


//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  //Serial.printf("WIFI event : %d\n", event);
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("Found WiFi network\n");
      Serial.printf("Waiting for eventual DHCP\n");
      riot.setState(RIOT_WAIT_IP);
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      riot.setState(RIOT_GOT_IP);
      if(riot.isForcedConfig()) {
        startWebServer();
      }
      startBonjour();
      
      motion.resetBeta();
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("Lost network, reconnecting\n");
      riot.setState(RIOT_LOST_CONNECTION);
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.printf("AP started successfully\n");
      if(riot.isConfig() || riot.isForcedConfig()) {
        startWebServer();
      }
      startBonjour();
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.printf("Client connected to AP\n");
      // We should have the client list updated here to send the motion data to
      break;

    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.printf("Client left AP\n");
      break;

    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.printf("Client got DHCP IP\n");
      break;

    default:
      break;
  }
  riot.connectionStatus = WiFi.status();
}

// Instantiate only one. We could use a singleton but it's not that popular :-)
riotCore riot;
