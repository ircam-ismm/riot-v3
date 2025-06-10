
#include "routines.h"



#define SOC_NUM_POINTS 8
const voltageSOCMap socMap[SOC_NUM_POINTS] = {
    {3.8f, 1.f}, {3.75f, 0.95f}, {3.70f, 0.90f}, {3.55f, 0.2f},
    {3.50f, 0.10f}, {3.45f, 0.05f}, {3.4f, 0.02f}, {3.35f, 0.f}
};

// Use CRGBW class simplified RGB to at least define colors with 8:8:8 RGB24
// Could be a #define substitute
void setLedColor(CRGBW8 color) { 
  rgbLedWrite(PIN_NEOPIXEL, color.r, color.g, color.b);
}

void printToOSC(char *str) {
  if(!riot.isConnected())
    return;
  //log_v("%s printToOSC: %s", TEXT_LOG_OSC, str);
  char addr[MAX_STRING_LEN];
  sprintf(addr, "/%s/%s/%d/%s", OSC_STRING_SOURCE, OSC_STRING_API_VERSION, riot.getID(), OSC_STRING_MESSAGE);
  printOscMessage.createString(addr, str);
  udpPacket.beginPacket(riot.getDestIP(), riot.getDestPort());
  udpPacket.write(printOscMessage.getBuffer(), printOscMessage.getSize());
  udpPacket.endPacket(); 
}

void die() {
  while(1) {
    delay(10);
    yield();
  }
}

void reset() {
  ESP.restart();
}



int readBatteryRaw(void) {
  return(analogRead(PIN_BATT_VOLTAGE));
}

float readBatteryVoltage(void) {
  //float voltage = readBatteryRaw() * 3.3f * 2.96f / 4096.f;
  float voltage = readBatteryRaw() * BATTERY_VOLTAGE_SCALE;
  return(voltage);
}

float readUsbVoltage(void) {
  //float voltage = (float)analogRead(PIN_USB_VOLTAGE) * 3.3f * 1.51f / 4096.f;
  float voltage = (float)analogRead(PIN_USB_VOLTAGE) * USB_VOLTAGE_SCALE;
  return voltage;
}

float voltageToSoC(float voltage) {
  if (voltage >= socMap[0].voltage)
    return socMap[0].soc;
    
  if (voltage <= socMap[SOC_NUM_POINTS - 1].voltage)
    return socMap[SOC_NUM_POINTS - 1].soc;

  float v1, v2, soc1, soc2, t;

  for (int i = 0; i < SOC_NUM_POINTS - 1; i++) {
    if (voltage <= socMap[i].voltage && voltage > socMap[i + 1].voltage) {
      v1 = socMap[i].voltage;
      v2 = socMap[i + 1].voltage;
      soc1 = socMap[i].soc;
      soc2 = socMap[i + 1].soc;
      t = (voltage - v2) / (v1 - v2); // linear interpolation
      return (soc2 + t * (soc1 - soc2));
    }
  }
  return 0.0f; // fallback
}


uint8_t readChargeStatus(void) {
  uint8_t chargeStatus;
  int pinState;
  int timeout;

  pinState = digitalRead(PIN_CHARGE_STATUS);
  //Serial.printf("charger pin = %d\n", pinState);
  timeout = millis();
  // When disconnected, the charger status flickers @250 Hz
  // Check if change of state, poll with timeout
  // If a change occurs within 5ms, the charger is probing the battery
  // without success and switches between low and hi-z indicating the battery
  // is most likely disconnected
  while((millis() - timeout) < 5 ) {
    if(pinState != digitalRead(PIN_CHARGE_STATUS)) {
      chargeStatus = BATTERY_DISCONNECTED;
      return chargeStatus;
      break;
    }
  }
  if(pinState == HIGH){
    chargeStatus = CHARGING_FINISHED;
  }  
  else {
    chargeStatus = CHARGING_IN_PROGRESS;
  }  
  return chargeStatus;
}

void setModemSleep() {
    WiFi.setSleep(true);  // Wifi will be re-enabled next time a packet is sent
    if (!setCpuFrequencyMhz(riot.getCpuDoze())){
        Serial.println("Not valid frequency!");
    }
}
 
void wakeModemSleep() {
    WiFi.setSleep(false);
    if(!setCpuFrequencyMhz(riot.getCpuSpeed())) {
        Serial.println("Not valid frequency!");
    }
}

void setWiFiPowerSavingMode(){
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);     // reduced listening interval
    // esp_wifi_set_ps(WIFI_PS_NONE);       // no power saving
    // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // default - delayed rx
}

void disableWiFi(){
    //adc_power_off(); //to fix
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
}

void format() {
  FFat.format();
    if (!FFat.begin()) {
      Serial.println("FFat Mount Failed permanently");
      die();
    }
    restoreDefaults(true);
}


// Performs an auto-test of the board, after booting normally and intializing
#define ACC_TEST_LINE         10 
#define X_SCOPE_LINE          30
#define Y_SCOPE_LINE          46
#define Z_SCOPE_LINE          62
#define SCOPE_SCALE           5
#define SCOPE_OFFSET          8
#define DIAG_IO_OFFSET        7
#define ANALOG_SCALE_ACC      5.f
#define BETWEEN_LINES         16
#define TEST_ADC_IO_VOLTAGE   1000    // About 3V
#define SAMPLE_DELAY          0

bool autoTest() {
  uint8_t result = 0;
  uint8_t isMovingX, isMovingY, isMovingZ;
  char str[MAX_STRING_LEN];
  float testArray[3], testArray_1[3];
  int maxSamples = u8g2.getWidth();
  bool accOK, gyroOK, magOK, baroOK, analogOK, gpioOK;
  
  if(riot.isDebug())
    Serial.printf("%s Running Board auto-test\n", TEXT_DEBUG_LOG);

  accOK = gyroOK = magOK = baroOK = analogOK = gpioOK = false;


  u8g2.begin();
  u8g2.clearDisplay();
  u8g2.clearBuffer();
  u8g2.setFont(MEDIUM_FONT);
  u8g2.drawStr(0, ACC_TEST_LINE, "Testing ACC");
  u8g2.drawStr(0, X_SCOPE_LINE, "X");
  u8g2.drawStr(0, Y_SCOPE_LINE, "Y");
  u8g2.drawStr(0, Z_SCOPE_LINE, "Z");

  isMovingX = isMovingY = isMovingZ = 0;
  
  for(int i = 0 ; i < (maxSamples-SCOPE_OFFSET) ; i++) {
    motion.grab();
    motion.compute();
    testArray[0] = clipData(motion.accX - testArray_1[0], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaX = %f / scaledX = %f\n", motion.accX - testArray_1[0], testArray[0]);
    testArray_1[0] = motion.accX;
    if(testArray[0] != 0.f)
      isMovingX++;
    u8g2.drawPixel(i+SCOPE_OFFSET, X_SCOPE_LINE-SCOPE_SCALE+testArray[0]);
    
    testArray[1] = clipData(motion.accY - testArray_1[1], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaY = %f / scaledY = %f\n", motion.accY - testArray_1[1], testArray[1]);
    testArray_1[1] = motion.accY;
    if(testArray[1] != 0.f)
      isMovingY++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Y_SCOPE_LINE-SCOPE_SCALE+testArray[1]);
      
    testArray[2] = clipData(motion.accZ - testArray_1[2], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaZ = %f / scaledZ = %f\n", motion.accZ - testArray_1[2], testArray[2]);
    testArray_1[2] = motion.accZ;
    if(testArray[2] != 0.f)
      isMovingZ++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Z_SCOPE_LINE-SCOPE_SCALE+testArray[2]);

    u8g2.sendBuffer();
    delay(SAMPLE_DELAY);  
  }

  if((isMovingX+isMovingY+isMovingZ) > maxSamples) {
    accOK = true;
    Serial.printf("Accelerometer OK\n");
    u8g2.drawStr(0, Z_SCOPE_LINE, "Accelerometer OK");
    u8g2.sendBuffer();
  }
  delay(100);

  u8g2.clearDisplay();
  u8g2.clearBuffer();
  u8g2.setFont(MEDIUM_FONT);
  u8g2.drawStr(0, ACC_TEST_LINE, "Testing GYRO");
  u8g2.drawStr(0, X_SCOPE_LINE, "X");
  u8g2.drawStr(0, Y_SCOPE_LINE, "Y");
  u8g2.drawStr(0, Z_SCOPE_LINE, "Z");

  isMovingX = isMovingY = isMovingZ = 0;
  
  for(int i = 0 ; i < (maxSamples-SCOPE_OFFSET) ; i++) {
    motion.grab();
    motion.compute();
    testArray[0] = clipData(motion.gyrX - testArray_1[0], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaX = %f / scaledX = %f\n", motion.gyrX - testArray_1[0], testArray[0]);
    testArray_1[0] = motion.gyrX;
    if(testArray[0] != 0.f)
      isMovingX++;
    u8g2.drawPixel(i+SCOPE_OFFSET, X_SCOPE_LINE-SCOPE_SCALE+testArray[0]);
    
    testArray[1] = clipData(motion.gyrY - testArray_1[1], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaY = %f / scaledY = %f\n", motion.gyrY - testArray_1[1], testArray[1]);
    testArray_1[1] = motion.gyrY;
    if(testArray[1] != 0.f)
      isMovingY++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Y_SCOPE_LINE-SCOPE_SCALE+testArray[1]);
      
    testArray[2] = clipData(motion.gyrZ - testArray_1[2], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaZ = %f / scaledZ = %f\n", motion.gyrZ - testArray_1[2], testArray[2]);
    testArray_1[2] = motion.gyrZ;
    if(testArray[2] != 0.f)
      isMovingZ++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Z_SCOPE_LINE-SCOPE_SCALE+testArray[2]);

    u8g2.sendBuffer();
    delay(SAMPLE_DELAY);  
  }

  if((isMovingX+isMovingY+isMovingZ) > maxSamples) {
    gyroOK = true;
    Serial.printf("Gyro OK\n");
    u8g2.drawStr(0, Z_SCOPE_LINE, "Gyro OK");
    u8g2.sendBuffer();
  }
  delay(100);

  u8g2.clearDisplay();
  u8g2.clearBuffer();
  u8g2.setFont(MEDIUM_FONT);
  u8g2.drawStr(0, ACC_TEST_LINE, "Testing MAG");
  u8g2.drawStr(0, X_SCOPE_LINE, "X");
  u8g2.drawStr(0, Y_SCOPE_LINE, "Y");
  u8g2.drawStr(0, Z_SCOPE_LINE, "Z");

  isMovingX = isMovingY = isMovingZ = 0;
  
  for(int i = 0 ; i < (maxSamples-SCOPE_OFFSET) ; i++) {
    motion.grab();
    motion.compute();
    testArray[0] = clipData(motion.magX - testArray_1[0], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaX = %f / scaledX = %f\n", motion.magX - testArray_1[0], testArray[0]);
    testArray_1[0] = motion.magX;
    if(testArray[0] != 0.f)
      isMovingX++;
    u8g2.drawPixel(i+SCOPE_OFFSET, X_SCOPE_LINE-SCOPE_SCALE+testArray[0]);
    
    testArray[1] = clipData(motion.magY - testArray_1[1], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaY = %f / scaledY = %f\n", motion.magY - testArray_1[1], testArray[1]);
    testArray_1[1] = motion.magY;
    if(testArray[1] != 0.f)
      isMovingY++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Y_SCOPE_LINE-SCOPE_SCALE+testArray[1]);
      
    testArray[2] = clipData(motion.magZ - testArray_1[2], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaZ = %f / scaledZ = %f\n", motion.magZ - testArray_1[2], testArray[2]);
    testArray_1[2] = motion.magZ;
    if(testArray[2] != 0.f)
      isMovingZ++;
    u8g2.drawPixel(i+SCOPE_OFFSET, Z_SCOPE_LINE-SCOPE_SCALE+testArray[2]);

    u8g2.sendBuffer();
    delay(SAMPLE_DELAY);  
  }

  if((isMovingX+isMovingY+isMovingZ) > maxSamples) {
    magOK = true;
    Serial.printf("Mag OK\n");
    u8g2.drawStr(0, Z_SCOPE_LINE, "Mag OK");
    u8g2.sendBuffer();
  }
  delay(100);

  // Barometer
  u8g2.clearDisplay();
  u8g2.clearBuffer();
  u8g2.setFont(MEDIUM_FONT);
  u8g2.drawStr(0, ACC_TEST_LINE, "Testing BARO");
  u8g2.drawStr(0, X_SCOPE_LINE, "P");
 
  isMovingX = 0;
  
  for(int i = 0 ; i < (maxSamples-SCOPE_OFFSET) ; i++) {
    motion.grab();
    motion.compute();
    testArray[0] = clipData(motion.pressure - testArray_1[0], ANALOG_SCALE_ACC, SCOPE_SCALE);
    if(riot.isDebug())
      Serial.printf("deltaX = %f / scaledX = %f\n", motion.pressure - testArray_1[0], testArray[0]);
    testArray_1[0] = motion.pressure;
    if(testArray[0] != 0.f)
      isMovingX++;
    u8g2.drawPixel(i+SCOPE_OFFSET, X_SCOPE_LINE-SCOPE_SCALE+testArray[0]);

    u8g2.sendBuffer();
    delay(SAMPLE_DELAY);  
  }

  if((isMovingX) > maxSamples/3) {
    baroOK = true;
    Serial.printf("Baro OK\n");
    u8g2.drawStr(0, Z_SCOPE_LINE, "Baro OK");
    u8g2.sendBuffer();
  }

  // Analog Inputs
  u8g2.clearDisplay();
  u8g2.clearBuffer();
  u8g2.setFont(MEDIUM_FONT);
  u8g2.drawStr(0, ACC_TEST_LINE, "Testing I/Os");
  u8g2.drawStr(0, X_SCOPE_LINE, " ");
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);
  pinMode(A5, INPUT);
  pinMode(A0, OUTPUT);
  digitalWrite(A0,HIGH);
  pinMode(A2, OUTPUT);
  digitalWrite(A2,HIGH);
  pinMode(A4, OUTPUT);
  digitalWrite(A4,HIGH);
  isMovingX = isMovingY = 0;
  if(analogRead(A1) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(analogRead(A3) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(analogRead(A5) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A1,HIGH);
  pinMode(A3, OUTPUT);
  digitalWrite(A3,HIGH);
  pinMode(A5, OUTPUT);
  digitalWrite(A5,HIGH);
  if(analogRead(A0) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(analogRead(A2) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(analogRead(A4) > TEST_ADC_IO_VOLTAGE) {
    isMovingX++;
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingX * DIAG_IO_OFFSET, X_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
    
  pinMode(40, INPUT);
  pinMode(38, INPUT);
  pinMode(39, OUTPUT);
  digitalWrite(39, HIGH);
  pinMode(41, OUTPUT);
  digitalWrite(41, HIGH);
  if(digitalRead(40)) {
    isMovingY++;
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(digitalRead(38)) {
    isMovingY++;
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  pinMode(39, INPUT);
  pinMode(41, INPUT);
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
  if(digitalRead(39)) {
    isMovingY++;
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  
  if(digitalRead(41)) {
    isMovingY++;
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "O");
  }
  else {
    u8g2.drawStr(isMovingY * DIAG_IO_OFFSET, Y_SCOPE_LINE, "X");
  }
  u8g2.sendBuffer();
  if(isMovingX == 6 && isMovingY == 4) {
    analogOK = gpioOK = true;
    Serial.printf("Analog + GPIO OK\n");
    u8g2.drawStr(0, Z_SCOPE_LINE, "ANALOG & I/O OK");
    u8g2.sendBuffer();
  }

   delay(100);
  
  if(accOK && gyroOK && magOK && baroOK && analogOK && gpioOK) {
    Serial.printf("autotest completed\n");
    Serial0.printf("testok\n");
    setLedColor(Green);
    delay(100);
    return true;
  }
  else {
    Serial.printf("autotest failed\n");
    Serial0.printf("testnok\n");
    setLedColor(Red);
  }
  return(false);
}

float clipData(float val, float analogScale, int scopeScale) {
  val = fmap(val, -1.f * ANALOG_SCALE_ACC, ANALOG_SCALE_ACC, SCOPE_SCALE, -1.f * SCOPE_SCALE);
  val = constrain(val, -1 * SCOPE_SCALE, SCOPE_SCALE);
  return(val);  
}

/*bool isAlive() {
  
}*/

//////////////////////////////////////////////////////////////////////////////////////////////
// File & Dir handling + helpers

bool checkFile(char *filename) {
  FILINFO fno;
  FRESULT fr1 = f_stat(filename, &fno);
  return(fr1 == FR_OK);
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("- failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
        Serial.print("  DIR : ");
        Serial.println(file.name());
        if(levels){
            listDir(fs, file.path(), levels -1);
        }
    } else {
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("\tSIZE: ");
        Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

bool printFile(char *pathname) {
  FIL TheFile;
  UINT read;
  char c;

   // Open file
  if (f_open(&TheFile, pathname, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    return false;
  }
  
  while((f_read(&TheFile, &c, 1, &read) == FR_OK && read != 0)) {
    Serial.printf("%c", c);
  }

  f_close(&TheFile);
  return true;
}

// perform the actual update from a given stream
void performUpdate(Stream &updateSource, size_t updateSize) {
  //bool UpdateClass::begin(size_t size, int command, int ledPin, uint8_t ledOn, const char *label)
  if (Update.begin(updateSize)) {
    // Get update progress report in % and sweep a rainbow on the onboard pixel
    size_t written = Update.writeStream(updateSource);
    if (written == updateSize) {
      Serial.printf("Wrote %d bytes\n", written);
    } else {
      Serial.printf("Wrote only : %d bytes / %d\n", written, updateSize);
    }
    if (Update.end()) {
      Serial.println("OTA done!");
      if (Update.isFinished()) {
        Serial.println("Update successfully completed. Rebooting.");
      } else {
        Serial.println("Update not finished? Something went wrong!");
      }
    } else {
      Serial.printf("Error Occurred. Error #: %d", Update.getError());
    }

  } else {
    Serial.println("Not enough space to begin OTA");
  }
}

// check given FS for valid update.bin and perform update if available
void updateFromFS(fs::FS &fs) {
  File updateBin = fs.open(FW_UPDATE_FILE);
  if(updateBin) {
    setLedColor(Aqua);
    delay(250);
    setLedColor(White);
    delay(250);
    setLedColor(Black);
    delay(250);
    setLedColor(White);
    if (updateBin.isDirectory()) {
      Serial.printf("Error, %s is not a file", FW_UPDATE_FILE);
      updateBin.close();
      return;
    }

    size_t updateSize = updateBin.size();

    if (updateSize > 0) {
      Serial.println("Try to start update");
      performUpdate(updateBin, updateSize);
    } else {
      Serial.println("Error, file is empty");
    }

    updateBin.close();
    setLedColor(Green);
    // when finished remove the binary from sd card to indicate end of the process
    // TODO : Instead rename the file after booting, if an update has been executed : test the reboot reason with resetWithReason.
    fs.remove(FW_UPDATE_FILE);
    delay(250);
    reset();
  } else {
    Serial.println("No fw update file found, skipping fw update");
  }
}
