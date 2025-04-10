
/* R-IoT v3 - FW of the 3rd generation Wireless WiFi OSC IMU Board - ESP32-S3 based MCU board
   
   Emmanuel FLETY - IRCAM - PIP Team
   
   This firmware handles everything from sensor collection using low level drivers taking advantage of the ESP32-S3 DMA,
   OSC encoding with low footprint libraries, sensor fusion using S. Madgwick's algorithm.
   Copyright (c) 2014-present IRCAM – Centre Pompidou (France, Paris)

    All rights reserved.
    
    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:
    
    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.
    
    * Redistributions in binary form must reproduce the above copyright notice, this
      list of conditions and the following disclaimer in the documentation and/or
      other materials provided with the distribution.
    
    * Neither the name of the IRCAM nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
    
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
 *   UPDATE 10/07/2024 : works great with toolchain 3.0.2 - code size 52% + RAM 20%
     UPDATE 15/01/2025 : works great with toolchain 3.1.1 - code size 54% (with OTA) + RAM 24%

     ** Dev History
    v3.0 : rough port of the CC3200 code to the ESP32 - Compiles initially with ESP32 toolchain 2.0.14
     PSRAM : disabled
     Code size :  42% (8MB) / RAM used : 21% (67k)
    v3.1 : migrating all C code to C++ with split classes and global objects (including OSC)
    v3.12 : moving forward with the new board, news sensors, OTA and more structure code
    
    Compilation target : ESP32-S3 dev module - USB Mode : USB-OTG / CDC on boot : enabled / DFU on boot : disabled / Upload Mode : USB-OTG(TinyUSB)
    CPU frequency : 160 MHz (or 80 MHz) / Flash Mode : QIO 80 MHz / Flash Size : 8 MB / Partition : 8MB with FAT (2MB app / 3.7MB FFAT)
  
  Todo : 
  
  - use vector arithmetics ?
  - add a HW timer for low jitter measurements and accurate integration delta T on gyros ?
  - Pixel could switch between blue and black at a constant period (based on sampleRate). Maybe a task on the other MCU core ?

  have a concert mode that is less energy hungry and disables wifi in idle / hibernate, super low power
  param to define dozing of the CPU 80/160/240MHz (changes current usage of 10mA and board temp +5° @240 MHz)
  but computations go down to 1.32ms instead of 2ms !!!
  
  add a help/usage/? command listing all the existing commands : could actually play a help page from the USB drive
  in the style of a man page. Could be a webpage. Could be a text page (on USB MSB) displayed thru serial, line by line

  Build a test jig with pogo. Test automated upload method using GPIO Zero and the regular UART to avoid plugging USB and having
  a single COM port to deal with. Test the DTR method and the 2 transistors like a regular ESP32 to see if it works. Make a test bed program

  code an autotest for the board : test sensors, fat, I/O, ADC, pixel (rainbow)

  add a gyro noisegate + test gyro HPF : decides whether to update madgwick or not if below threshold (gyro norm)
  => Update madgwick but don't update the resulting angles if < gyro gate

  have a dynamic beta when gyro spin, with decay like during boot, using the linear interpolator (convergence over XX duration, like 1s)

  See how it goes with live update of the magnetometers offsets to avoid relying on calibration. Export a confidence / accurary score with the Euler
  to define if the bias are changing drastically or not. The default hard iron are still being calibrated along with the soft iron, stored, and 
  recalled dynamically at boot time and are used as start point, then their drift is analyzed in real time.
  
  param : useMagneto. Automatically switches between madgwick and mahony filter.

  integrate an autofw update by connecting to internet to the github repo then lauch elegantOTA

  - Use a decreasing Beta Gain at the start to have a quick convergence then use the desired Beta convergence rate
  - param to enable autocalibration of the gyro during stillness for long enough
  - calibration of gyros with temperature (biblio)
  - bring back elipsoid fitting for the spherical calibration of the magnetometers

  auto-sleep mechanism to detect idle time after a while and go to sleep mode (reduced power) with auto wake up based on moves

  have ALL config parameters parsed as OSC string routed from /id/msg => parsed by the serial command parser
  have calibration respond to OSC commands.

  Implement a unitary test with madgwick's updated filter & classes, port to arduino to make it usable with ESP32 or else

  check if we accept domain names / URL instead of the IP (in parseConfig())

  explore Websockets
 
  add param for sending data with calibration offsets or without (motion class)
  add proper soft+hard iron calibration and compensation
  perform basic live calibration of hard iron for mag. Maybe BNO055 does that live and detects pertubations of the mag field => new calib

  Add oversampling of the orientation filter (see limits and how long it takes to compute madgwick) for better convergence and stability
  then define an general ODR for data (OSC)

  add OLED support + display menus => Make a wristlet demo

  create an HTML page that displays the graphs of the sensors in HTML5 

  local NTP server connection + OSC time tags ?
  check if AP can be started without passphrase just SSID + open network ? 

  test Touch capacitive inputs

  add all analysis bricks (kick, tap, move, freefall etc + some of the Accel internal primitives)
  add 1€ filter algorithms in the bricks - combine with other low pass (Seb)

  USE log_v for debug message using the verbose target in the compile options

*/


#if ARDUINO_USB_MODE
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else

#include "main.h"
#include "osc.h"
#include "routines.h"
#include "textfile.h"
#include "riot.h"
#include "motion.h"
#include "sensors.h"
#include "web.h"


// Mass storage driver using TinyUSB to serve a FATFS flash drive using ESP32's internal flash
// S3-pico + internal flash of 8Mo => 3.7 MB FATFS with a 2MB app (with OTA)
static const uint32_t DISK_SECTOR_COUNT = 947; // We have 4096 bytes sectors with the internal flash, that's about 3.7 MB
static const uint16_t DISK_SECTOR_SIZE = FF_SS_WL;    // Should be 512 for SD but here we have 4096 bytes per flash page
USBMSC MSC;

// OLED 128x64 pixels support
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Global vars & objects to be shared
// Network, webserver, UDP, OSC
static int elapsedTimeServer;
WebServer httpServer(HTTP_SERVER_PORT);   // This is the configuration webserver + OTA (same port : 80)
WiFiClient client;
WiFiUDP udpPacket;
WiFiUDP configPacket;

// The number 1024 between the < > below  is the maximum number of bytes reserved for incomming messages.
// Outgoing messages are written directly to the output and do not need more reserved bytes.
// Review this to have the configuration (IP, port) reworked
MicroOscUdp<1024> oscUdp(&configPacket, defaultIP, DEFAULT_UDP_PORT);
void receivedOscMessage( MicroOscMessage& message);


/////////////////////////////////////////////////////////////////
// Serial port message / buffers / temporary strings
char serialBuffer[MAX_SERIAL_LEN];
unsigned char serialIndex = 0;

TimerHandle_t xTimerSwitches;
void timerCallback(TimerHandle_t pxTimer);  // To poll switches & debounce


void setup() {
// Tests & Debug Only
  //Serial0.begin(115200);
  //Serial0.setDebugOutput(false);
  Serial.setTxTimeoutMs(0);
  Serial.setDebugOutput(false);   // Ensures that Serial.printf doesn't locks when CDC port is closed
  Serial.begin(115200);
  //delay(2000);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.clearDisplay();

  riot.init();
  xTimerSwitches = xTimerCreate("switch_poll_timer",SWITCH_POLLING_PERIOD, pdTRUE, 0, timerCallback);
  xTimerStart(xTimerSwitches, 0); // start now
  
  // CHECK FAT / File system. FormatOnFail doesn't work so well
  // F_Fat::begin(bool formatOnFail, const char * basePath, uint8_t maxOpenFiles, const char * partitionLabel)
  if (!FFat.begin()) {
    Serial.println("FFat Mount Failed - attempting format");
    FFat.format();
    if (!FFat.begin()) {
      Serial.println("FFat Mount Failed permanently");
      die();
    }
    restoreDefaults(true);
  }

  // Checks for fw updates to perform
  // Review this : maybe call from main loop, disable MSD while performing update etc
  // see if Update.write() by chunks works better than Update.writeStream()
  updateFromFS(FFat);


  // MASS STORAGE USB Driver to expose FFAT drive
  USB.onEvent(usbEventCallback);
  MSC.vendorID("IRCAM");//max 8 chars
  MSC.productID("RIOT3-MSC");//max 16 chars
  MSC.productRevision("1.0");//max 4 chars
  MSC.onStartStop(onStartStop);
  MSC.onRead(onRead);
  MSC.onWrite(onWrite);
  MSC.mediaPresent(true);
  MSC.begin(DISK_SECTOR_COUNT, DISK_SECTOR_SIZE);
  
  USB.begin();
  
  delay(riot.getSlowBoot());

  ///////////////////////////////////////////////////////////////////////////
  //// Init sensors and motion engine
  lsm6d.begin(PIN_CS_ACC_GYR);
  lis3mdl.begin(PIN_CS_MAG);
  bmp390.begin(PIN_CS_ATM);
  // Check presence of BNO055 on I2C
  if(bno055.begin().TestConnection()) {
    Serial.println("Found BNO055 sensor\n");
    riot.bno055Found(true);
    bno055.Initialize();
    bno055.Set_Mode(NDOF);  // 9DoF fusion
  }
  motion.init();
  
  int parsingTime = millis();
  Serial.printf("Parsing configuration file\n\n");
  // Retrieve saved params in FLASH using the FFAT file system
  if (!configFile.parseConfigFile(true)) {
    Serial.printf("%s : No Main Config File\n", TEXT_ERROR_LOG);
  }
  
  Serial.printf("\nFinished Parsing config in %dms\n", millis() - parsingTime);  
  Serial.println("Params Loaded");

  motion.begin(); // Must be executed after parsing file to compute biases from config

  /////////////////////////////////////////////////////////////////////////////////////////
  // Some self-diag @boot time
  riot.version();  // populates the version string + displays it
  Serial.printf("Battery = %f Volts\n", readBatteryVoltage());
  Serial.printf("USB = %f Volts\n", readUsbVoltage());
  Serial.printf("Battery Charge : %s\n", readChargeStatus() ? "Charging" : "Finished");
  uint64_t chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32)); //print High 2 bytes
  Serial.printf("%08X\n", (uint32_t)chipid); //print Low 4bytes.
  Serial.printf("Flash Drive Total space: %u bytes\n", FFat.totalBytes());
  Serial.printf("Flash Drive Free space: %u bytes\n", FFat.freeBytes());

  /////////////////////////////////////////////////////////////////////////////////////////
  // Check if we are going in configuration mode (+webserver + OTA)
  // 2-3 second shorting onboard the switch pin to ground during boot
  // Config mode disables streaming
  int configModeCounter = millis();
  bool blinkIt = false;
  riot.setOperationState(RIOT_STREAMING);
  riot.poll();
  while (riot.onBoardSwitch.pressed()) {
    delay(20);
    riot.poll();
    blinkIt = !blinkIt;
    setLedColor(blinkIt ? Red : Black);

    if ((millis() - configModeCounter) > CONFIG_MODE_TIMEOUT) {
      riot.setConfigMode(true);
      riot.setOperationState(RIOT_IDLE);
      Serial.println("Configuration Mode enabled");
      riot.setOperationState(RIOT_IDLE);
      elapsedTimeServer = millis();
      break;
    }
  }

  // Improve seeding with motion sensor, port what is done with CFX with the new seed initializer
  randomSeed((int)(readBatteryVoltage() * 1000.f));

  riot.begin();
  // Initiates the wifi connection, state machine will be processed in the main loop
  riot.start();

  if(riot.isCalibrate())
    Serial.println("Calibration available for now");
}


void loop() {
  
  riot.update();      // Updates the network/Wifi connection state machine
  riot.calibrate();   // Handles the streaming / calibration state machine
  riot.charge();      // Handles the module's charge vs. streaming based on selected mode

  // The main process of the module : sensors acquisition, computation, OSC streaming
  if(riot.isStreaming()) {

    riot.process();

    // Incoming messages consume 170µs with wifi / UDP processing but no harm to the main loop
    if (riot.isOSCinput()) {
      //Serial.println("osc in check");
      // Parses incoming OSC messages
      oscUdp.receiveMessages( receivedOscMessage );  
    }
  } // end of IF RIOT IS !config (ie. normal use of sensors digitzing & OSC export)

  // Process the Webserver and OTA update server - Could be a SW timer task like switches
  if (riot.isConfig() || riot.isForcedConfig()) {
    if(millis() - elapsedTimeServer > 100) {
      elapsedTimeServer = millis();
      wakeModemSleep();
      httpServer.handleClient();
      ElegantOTA.loop();
      setModemSleep();
    }
  }

  // Serial commands and messages parser
  while (Serial.available() > 0) {
    serialBuffer[serialIndex] = Serial.read();
    if (serialBuffer[serialIndex] == '\n' || serialBuffer[serialIndex] == '\r') {
      if (serialIndex > MAX_SERIAL_LEN) {
        serialIndex = 0;
        clearString(serialBuffer, sizeof(serialBuffer));
        break;
      }
      else {
        // Process command
        char stringBuffer[strlen(serialBuffer) + 2];
        strcpy(stringBuffer, serialBuffer);
        int len = strlen(stringBuffer);
        if (len == 1) // Empty command, just the terminator
          break;
        purgeCRLF(stringBuffer, len);
        // Reset Index
        serialIndex = 0;
        //printf("process serial\n");
        clearString(serialBuffer, sizeof(serialBuffer));
        //printf("Received on Serial : %s\n", StringBuffer);
        processSerial(stringBuffer);
      }
      serialIndex = 0;
    }
    else {
      serialIndex++;
      if (serialIndex > MAX_SERIAL_LEN)
        serialIndex = 0;
    }
  } // End of Serial processing / parsing
}


// Todo : use callbacks
void receivedOscMessage( MicroOscMessage& message) {
  int32_t firstArgument;
  char line[MAX_STRING_LEN];
  
  if(message.fullMatch("/output", "i") ) {
    firstArgument = message.nextAsInt();
    firstArgument = constrain(firstArgument, false, true);

    //Serial.print("DEBUG /output/i ");
    digitalWrite(REMOTE_OUTPUT, firstArgument);
    return;
  }
  else if(message.fullMatch("/pwm", "i") ) {
    firstArgument = message.nextAsInt();
    firstArgument = constrain(firstArgument, 0, 255);
    //Serial.print("DEBUG /output/i ");
    analogWrite(REMOTE_OUTPUT, firstArgument);
    return;
  }
  else if(message.fullMatch(riot.getOscAddress(), "s") ) {
    strcpy(line, message.nextAsString());
    // Parsing message commands
    if(!strncmp(TEXT_PING, line, strlen(TEXT_PING))) {
      Serial.printf("%s\n", TEXT_ECHO);
      printToOSC(TEXT_ECHO);
      return;
    }
    else if(!strncmp(TEXT_GO_COMMAND, line, strlen(TEXT_GO_COMMAND))) {
      printToOSC("Next Step");
      motion.nextStep(true);  // Proceed with calibration - emulates the switch press
      return;
    }
    else if(!strncmp(TEXT_CANCEL_COMMAND, line, strlen(TEXT_CANCEL_COMMAND))) {
      printToOSC("Cancelling operation");
      motion.cancel(true);  // Cancel calibration - emulates the switch press
      return;
    }
    else if(!strncmp(TEXT_AUTOCAL_MAG, line, strlen(TEXT_AUTOCAL_MAG))) {
      printToOSC("Starting Mag Calibration");
      motion.runAutoCalMag();
      return;
    }
    else if(!strncmp(TEXT_AUTOCAL_MOTION, line, strlen(TEXT_AUTOCAL_MOTION))) {  
      printToOSC("Starting Acc-Gyro Calibration");  
      motion.runAutoCalMotion();
      return;
    }
    else if(!strncmp(TEXT_SAVE_CONFIG, line,strlen(TEXT_SAVE_CONFIG))) { // Saves config to FLASH
      storeConfig();
      printToOSC("Config saved");
      return;
    }
    else if(!strncmp(TEXT_REBOOT, line,strlen(TEXT_REBOOT))) { // Saves config to FLASH
      // Reboot is needed to use new settings - force reboot with the watchdog or another technique or wait for the reset command
      printToOSC("Reboot module");
      reset();
      return;
    }
    // implement getIP, wifi RSSI etc.
  }
  else {
    Serial.printf("[RX] Wrong OSC syntax\n");
  }
}


// Xtimer polling & debouncing switches @SWITCH_POLLING_PERIOD
void timerCallback(TimerHandle_t pxTimer) {
    riot.poll();
}


//////////////////////////////////////////////////////////////////////////////////////////////
// USB MSD callbacks
static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  log_v("MSC WRITE: lba: %u,  : %u, bufsize: %u\n", lba, offset, bufsize);
  //DRESULT res = disk_write(FFat.getDrive(), (uint8_t*)buffer, lba, bufsize / DISK_SECTOR_SIZE);
  DRESULT res = disk_write(0, (uint8_t*)buffer, lba, bufsize / DISK_SECTOR_SIZE);
  if (res != RES_OK) {
    log_e("[FAT] MSC write failed - err: %d\n", res);
    return 0;
  }
  return bufsize;
}

static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  log_v("MSC READ: lba: %u, offset: %u, bufsize: %u\n", lba, offset, bufsize);
  DRESULT res = disk_read(0, (uint8_t*)buffer, lba, bufsize / DISK_SECTOR_SIZE);
  if (res != RES_OK) {
    log_e("[FAT] MSC read failed - err: %d\n", res);
    return 0;
  }
  return bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  log_v("MSC START/STOP: power: %u, start: %u, eject: %u\n", power_condition, start, load_eject);
  if (load_eject) {
    log_v("MSC EJECT NOW\n");
    // TODO : disable MSD / end or just set drive "off"
  }
  return true;
}

static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    arduino_usb_event_data_t * data = (arduino_usb_event_data_t*)event_data;
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT:
        log_v("USB PLUGGED");
        break;
      case ARDUINO_USB_STOPPED_EVENT:
        log_v("USB UNPLUGGED");
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        log_v("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
        break;
      case ARDUINO_USB_RESUME_EVENT:
        log_v("USB RESUMED");
        break;

      default:
        break;
    }
  }
}

#endif /* ARDUINO_USB_MODE */
