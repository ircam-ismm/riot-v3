
#include "routines.h"

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
    if (!setCpuFrequencyMhz(80)){
        Serial.println("Not valid frequency!");
    }
}
 
void wakeModemSleep() {
    setCpuFrequencyMhz(240);
    //setCpuFrequencyMhz(160);
    //setCpuFrequencyMhz(80);
}


void format() {
  FFat.format();
  if (!FFat.begin()) {
    Serial.println("FFat Mount Failed permanently");
    die();
  }
  restoreDefaults(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// File & Dir handling + helpers
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
    riot.version(true);   // log version to file
    delay(250);
    reset();
  } else {
    Serial.println("No fw update file found, skipping fw update");
  }
}
