
#include "textfile.h"

configurationFile::configurationFile() {
  _writable = false;
  _file_name = NULL;
  preload_size = preload_offset = 0;
}

configurationFile::~configurationFile() {
}

bool configurationFile::preload() {
  UINT read;

  if (f_read(&_file, &preload_buffer, CONFIG_PRELOAD_SIZE, &read) != FR_OK || read == 0)
    return false;

  preload_size = read;
  preload_offset = 0;
  return true;
}

bool configurationFile::readLine(char* line) {
  char c;
  uint32_t idx = 0;

  while (idx < CONFIG_MAX_LINE_LEN) {
    if (preload_offset == preload_size) {
      if (!preload())
        return false;
    }

    c = preload_buffer[preload_offset++];

    if (c == '\r' || c == '\0')
      continue;

    if (c == '\n')
      break;

    line[idx++] = c;
  }

  if (idx == CONFIG_MAX_LINE_LEN) {
    // Line is too long, return an error
    Serial.println("Line too long, skipping...\n");
    return false;
  }
  else
    // Ensure null termination
    line[idx] = 0;

  return true;
}

void configurationFile::removeWhiteSpace(char* str) {
  char* dst = str;

  while (*str) {
    if (*str != ' ' && *str != '\t')    // Remove space and tabs
      *(dst++) = *str;

    str++;
  }
  *dst = '\0';
}

bool configurationFile::begin(const char* path, bool writable) {
  f_close(&_file);

  // Open file
  if (f_open(&_file, path, FA_OPEN_EXISTING | FA_READ) != FR_OK) {
    return false;
  }
  _writable = writable;

  if (_writable && !_file_name) {
    _file_name = (char*) malloc(strlen(path));
    if (!_file_name) {
      f_close(&_file);
      return false;
    }

    strcpy(_file_name, path);
  }
  preload_size = preload_offset = 0;

  return true;
}

void configurationFile::end() {
  f_close(&_file);
  preload_size = preload_offset = 0;
  //printf("[CONFIG] closed file\n");
}

void configurationFile::setCallback(parsingCallback *cb) {
  parseSyntax = cb;
}

bool configurationFile::parseConfigFile(bool debug) {
  if (!begin(CONFIG_FILE, false))
    return (false);
  setCallback(parseConfigCallback);
  while (readLine(stringBuffer)) {
    removeWhiteSpace(stringBuffer);
    if (!parseSyntax(stringBuffer)) {
      if(debug)
        Serial.printf("[%s] : Unknown parameter \"%s\" \n", CONFIG_FILE, stringBuffer);
    }
  } // EOF
  end();
  return (true);
}

configurationFile configFile;

////////////////////////////////////////////////////////////////:
// C functions / helpers

void clearString(char* str, int size) {
  if(!size)
    size = strlen(str);
  memset(str ,0,size);
}

/////////////////////////////////////////////////////
// Look for the '=' sign then tries to find a value
int skipToValue(char *line, char separator, bool strip) {
  unsigned char i = 0;
  while (line[i] != separator) {
    if (i >= CONFIG_MAX_LINE_LEN) {
      Serial.printf("Syntax error - '%c' sign is missing\n", separator);
      //printf("%s\n",line);
      return (0);
    }
    if (line[i] == '\0')
      return (0);
    i++;
  }
  i++;
  if (strip) { // strips further separators to reach the value
    while (line[i] == separator) {
      if (i >= CONFIG_MAX_LINE_LEN) {
        Serial.printf("Line too long");
        //printf("%s\n",line);
        return (0);
      }
      i++;
    }
  }
  return (i);
}

/////////////////////////////////////////////////////////////////////////////////
// Look for the ',' sign in a list of values (or another separator)
int skipToNextValue(char *line, int startIndex, char separator, bool strip) {
  int i = startIndex;
  while (line[i] != separator) {
    if (i >= CONFIG_MAX_LINE_LEN) {
      //Serial.printf("Syntax error - '%c' separator sign is missing\n", separator);
      //Serial.printf("%s\n",line);
      return (0);
    }
    if ((line[i] == '\0'))
      return (0);
    i++;
  }
  i++;
  if (strip) { // strips further separators to reach the value
    while (line[i] == separator) {
      if (i >= CONFIG_MAX_LINE_LEN) {
        Serial.printf("Line too long");
        //Serial.printf("%s\n",line);
        return (0);
      }
      i++;
    }
  }
  return (i);
}


bool isComment(char *line) {
  // 'C++' like comment
  if (!strncmp(TEXT_COMMENT, line, strlen(TEXT_COMMENT))) {
    // Print the comment : useful to make the difference of several
    // config file in the multiple sound bank directories
    if(riot.isDebug())
      Serial.printf("%s '%s'\n", TEXT_COMMENT_LOG, line);
    //strcpy(CommentString,line+2);
    return (true);
  }

  // 'emacs' styled comment
  if (!strncmp(TEXT_COMMENT2, line, strlen(TEXT_COMMENT2))) {
    // Print the comment : useful to make the difference of several
    // config file in the multiple sound bank directories
    if(riot.isDebug())
      Serial.printf("%s '%s'\n", TEXT_COMMENT_LOG, line);
    //strcpy(CommentString,line+2);
    return (true);
  }

  return (false);
}

void purgeCRLF(char *TheString, int TheSize) {
  // purge CR and LF
  for (int i = 0 ; i < TheSize ; i ++) {
    if ((TheString[i] == '\n') || (TheString[i] == '\r'))
      TheString[i] = '\0';
  }
}

void padString(char *TheString, char TheChar, int PaddingSize) {
  int i = 0;
  // finds the end of the string
  while (TheString[i] != '\0')
    i++;
  // Pads with proposed char
  while (i < PaddingSize) {
    TheString[i] = TheChar;
    i++;
  }
  TheString[i] = '\0'; // End of String terminator
}


bool skipLine(char *line) {
  if (!strlen(line))
    return (true);
  if (isComment(line))
    return (true);
  return(false);
}

void eol(char* str, uint8_t howmany) {
  if (!howmany)
    return;
    
  for (int i = 0; i < howmany ; i++) {
    strcat(str, TEXT_FILE_EOL);
  }
}


// Uses the string / line from the serial command prompt and parses it with the
// same callback as the text file parser (callback)
bool processSerial(char *str) {
  bool ok = false;
  
  if(parseConfigCallback(str)) {
    ok = true;  
  }
  else if(!ok && riot.isDebug())
    Serial.printf("Rx [%s]\n", str);  // debugs this only to USB serial 
      
  return (ok);
}


  
bool parseConfigCallback(char *line) {
  int i, j, index, val;
  byte temp_ip[IPV4_SIZE];
  IPAddress tempIP;

  if(skipLine(line))
    return (true);

  // Debug
  //Serial.printf("Cmd: %s - OK\n", line);

  // Send current config to the configuration app
  if(!strncmp(TEXT_GET_CONFIG, line, strlen(TEXT_GET_CONFIG))) {
    // Outputs all the configuration  
    Serial.printf("%s %d\n", TEXT_DHCP, riot.isDHCP());
    Serial.printf("%s %s\n", TEXT_SSID, riot.getSSID());
    Serial.printf("%s %d\n", TEXT_WIFI_MODE, riot.getOperatingMode());
    Serial.printf("%s %s\n", TEXT_PASSWORD, riot.getPassword());
    Serial.printf("%s %s\n", TEXT_MDNS, riot.getBonjour());
    
    tempIP = riot.getOwnIP(); 
    Serial.printf("%s %u.%u.%u.%u\n", TEXT_OWNIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3] );
    tempIP = riot.getDestIP(); 
    Serial.printf("%s %u.%u.%u.%u\n", TEXT_DESTIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    tempIP = riot.getGatewayIP(); 
    Serial.printf("%s %u.%u.%u.%u\n", TEXT_GATEWAY, tempIP[0], tempIP[1],tempIP[2],tempIP[3]);
    tempIP = riot.getSubnetMask();
    Serial.printf("%s %u.%u.%u.%u\n", TEXT_MASK, tempIP[0], tempIP[1], tempIP[2], tempIP[3] );
    Serial.printf("%s %u\n", TEXT_PORT, riot.getDestPort());
    Serial.printf("%s %u\n", TEXT_RECEIVE_PORT, riot.getReceivePort());

    Serial.printf("%s %u\n", TEXT_MASTER_ID, riot.getID());
    Serial.printf("%s %u\n", TEXT_SAMPLE_RATE, motion.getSampleRate());
    Serial.printf("%s %d\n", TEXT_WIFI_POWER, riot.getWifiPower());
    Serial.printf("%s %u\n", TEXT_REMOTE, riot.isOSCinput());
    Serial.printf("%s %u\n", TEXT_FORCE_CONFIG, riot.isForcedConfig());
    Serial.printf("%s %u\n", TEXT_CALIBRATION, riot.getCalibrationTimer());
    Serial.printf("%s %u\n", TEXT_CHARGE_MODE, riot.getChargingMode());

    Serial.printf("%s %u\n", TEXT_CPU_SPEED, riot.getCpuSpeed());
    Serial.printf("%s %u\n", TEXT_CPU_DOZE, riot.getCpuDoze());
    
    Serial.printf("%s %f\n", TEXT_DECLINATION, motion.getDeclination());
    Serial.printf("%s %u\n", TEXT_ORIENTATION, motion.getOrientation());
    Serial.printf("%s %u\n", TEXT_BNO_ORIENT, bno055.orientation);

    Serial.printf("%s %u\n", TEXT_ACC_RANGE, lsm6d.getAccRange());
    Serial.printf("%s %u\n", TEXT_GYRO_RANGE, lsm6d.getGyroRange());
    Serial.printf("%s %u\n", TEXT_MAG_RANGE, lis3mdl.getRange());
    Serial.printf("%s %f\n" ,TEXT_GYRO_GATE, motion.getGyroGate());
    Serial.printf("%s %u\n", TEXT_GYRO_HPF, lsm6d.getGyroHpf());
  
    Serial.printf("%s %u\n", TEXT_BARO_MODE, bmp390.getSamplingMode());
    Serial.printf("%s %f\n", TEXT_BARO_REF, bmp390.getRefAltitude());    
    Serial.printf("%s %u\n", TEXT_SLOW_BOOT, riot.getSlowBoot());
    Serial.printf("%s ", TEXT_LED_COLOR); riot.getPixelColor().print();
          
    // All offsets as lists + rotation matrix
    Serial.printf("%s %d\n", TEXT_ACC_OFFSETX, motion.getAccelBiasRaw(X_AXIS));
    Serial.printf("%s %d\n", TEXT_ACC_OFFSETY, motion.getAccelBiasRaw(Y_AXIS));
    Serial.printf("%s %d\n", TEXT_ACC_OFFSETZ, motion.getAccelBiasRaw(Z_AXIS));
   
    Serial.printf("%s %d\n", TEXT_GYRO_OFFSETX, motion.getGyroBiasRaw(X_AXIS));
    Serial.printf("%s %d\n", TEXT_GYRO_OFFSETY, motion.getGyroBiasRaw(Y_AXIS));
    Serial.printf("%s %d\n", TEXT_GYRO_OFFSETZ, motion.getGyroBiasRaw(Z_AXIS));
    
    Serial.printf("%s %d\n", TEXT_MAG_OFFSETX, motion.getMagBiasRaw(X_AXIS));
    Serial.printf("%s %d\n", TEXT_MAG_OFFSETY, motion.getMagBiasRaw(Y_AXIS));
    Serial.printf("%s %d\n", TEXT_MAG_OFFSETZ, motion.getMagBiasRaw(Z_AXIS));

    // Soft Iron Matrix
    float *pVect;
    pVect = motion.getSoftIronMatrixRow(X_AXIS);
    Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX1, pVect[0], pVect[1], pVect[2]);
    pVect = motion.getSoftIronMatrixRow(Y_AXIS);
    Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX2, pVect[0], pVect[1], pVect[2]);
    pVect = motion.getSoftIronMatrixRow(Z_AXIS);
    Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX1, pVect[0], pVect[1], pVect[2]);
   
    Serial.printf("%s %f\n", TEXT_BETA, motion.getBeta()); 
      
    Serial.printf("refresh\n");
    return(true);
  }
  
  // Ping / Echo question/answer from the GUI
  else if(!strncmp(TEXT_DEBUG, line, strlen(TEXT_DEBUG))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setDebugMode(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_DEBUG, riot.isDebug());
    return true;
  }

  // Ping / Echo question/answer from the GUI
  else if(!strncmp(TEXT_PING, line, strlen(TEXT_PING))) {
    Serial.printf("%s\n", TEXT_ECHO);  // a simple ASCII echo answer to let the GUI know the COM port is the right one
    return true;
  }

  else if(!strncmp(TEXT_SAVE_CONFIG, line,strlen(TEXT_SAVE_CONFIG))) { // Saves config to FLASH
    storeConfig();
    return(true);
  }
  else if(!strncmp(TEXT_REBOOT, line,strlen(TEXT_REBOOT))) { // Saves config to FLASH
    // Reboot is needed to use new settings - force reboot with the watchdog or another technique or wait for the reset command
    reset();
    return(true);
  }

  else if(!strncmp(TEXT_WIFI_RSSI, line,strlen(TEXT_WIFI_RSSI))) { // Saves config to FLASH
    // Reboot is needed to use new settings - force reboot with the watchdog or another technique or wait for the reset command
    riot.getRSSI();
    return(true);
  }
  
  else if(!strncmp(TEXT_WIFI_MODE, line,strlen(TEXT_WIFI_MODE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setOperatingMode(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_WIFI_MODE, riot.getOperatingMode());
    return(true);
  }
  
  else if(!strncmp(TEXT_DHCP, line,strlen(TEXT_DHCP))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setUseDHCP(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_DHCP, riot.isDHCP());
    return(true);
  }
  else if(!strncmp(TEXT_SSID, line,strlen(TEXT_SSID))) {
    index = skipToValue(line);
    riot.setSSID(&line[index]);
    if(riot.isDebug())
      Serial.printf("%s %s\n", TEXT_SSID, riot.getSSID());
    return(true);
  }
  else if(!strncmp(TEXT_PASSWORD, line,strlen(TEXT_PASSWORD))) {
    index = skipToValue(line);
    riot.setPassword(&line[index]);
    if(riot.isDebug())
      Serial.printf("%s %s\n", TEXT_PASSWORD, riot.getPassword());
    return(true);
  }
  else if(!strncmp(TEXT_OWNIP, line,strlen(TEXT_OWNIP))) {
    index = skipToValue(line);
    tempIP.fromString(&line[index]);
    riot.setOwnIP(tempIP);
    if(riot.isDebug())
      Serial.printf("%s %u.%u.%u.%u\n",TEXT_OWNIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    return(true);
  } 
  else if(!strncmp(TEXT_DESTIP, line, strlen(TEXT_DESTIP))) {
    index = skipToValue(line);
    tempIP.fromString(&line[index]);
    riot.setDestIP(tempIP);
    if(riot.isDebug())
      Serial.printf("%s %u.%u.%u.%u\n",TEXT_DESTIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    
    return(true);
  } 
  else if(!strncmp(TEXT_GATEWAY, line, strlen(TEXT_GATEWAY))) {
    index = skipToValue(line);
    tempIP.fromString(&line[index]);
    riot.setGatewayIP(tempIP);
    if(riot.isDebug())
      Serial.printf("%s %u.%u.%u.%u\n",TEXT_GATEWAY, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    return(true);
  } 
  else if(!strncmp(TEXT_MASK, line, strlen(TEXT_MASK))) {
    index = skipToValue(line);
    tempIP.fromString(&line[index]);
    riot.setSubnetMask(tempIP);
    if(riot.isDebug())
      Serial.printf("%s %u.%u.%u.%u\n",TEXT_MASK, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
    return(true);
  } 
  // mDNS name
  else if(!strncmp(TEXT_MDNS, line, strlen(TEXT_MDNS))) {
    index = skipToValue(line);
    riot.setBonjour(&line[index]);
    if(riot.isDebug())
      Serial.printf("%s %s.local\n", TEXT_MDNS, riot.getBonjour());
    
    return(true);
  } 
  
  else if(!strncmp(TEXT_PORT, line, strlen(TEXT_PORT))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setDestPort(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_PORT, riot.getDestPort());
    
    return(true);
  } 
  else if(!strncmp(TEXT_RECEIVE_PORT, line, strlen(TEXT_RECEIVE_PORT))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setReceivePort(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_RECEIVE_PORT, riot.getReceivePort());
    
    return(true);
  } 
  else if(!strncmp(TEXT_MASTER_ID, line, strlen(TEXT_MASTER_ID))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    riot.setID(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_MASTER_ID, riot.getID());
    return(true);
  }
  else if(!strncmp(TEXT_SAMPLE_RATE, line, strlen(TEXT_SAMPLE_RATE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, MIN_SAMPLERATE, MAX_SAMPLERATE);
    motion.setSampleRate(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_SAMPLE_RATE, motion.getSampleRate());
    return(true);
  } 
  else if(!strncmp(TEXT_REMOTE, line, strlen(TEXT_REMOTE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, false, true);
    riot.setOscInput(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_REMOTE, riot.isOSCinput());
    return(true);
  } 
  else if(!strncmp(TEXT_FORCE_CONFIG, line, strlen(TEXT_FORCE_CONFIG))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, false, true);
    riot.setForcedConfigMode(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_FORCE_CONFIG, riot.isForcedConfig());
    return(true);
  } 
  else if(!strncmp(TEXT_WIFI_POWER, line, strlen(TEXT_WIFI_POWER))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,WIFI_POWER_MINUS_1dBm , WIFI_POWER_19_5dBm);
    riot.setWifiPower((wifi_power_t)val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_WIFI_POWER, riot.getWifiPower());
    return(true);
  } 
  else if(!strncmp(TEXT_CALIBRATION, line, strlen(TEXT_CALIBRATION))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , 20000);
    riot.setCalibrationTimer(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_CALIBRATION, riot.getCalibrationTimer());
    return(true);
  }
  else if(!strncmp(TEXT_DECLINATION, line, strlen(TEXT_DECLINATION))) {
    index = skipToValue(line);
    float angle = atof(&line[index]);
    angle = constrain(angle,0 , 90.f);  // Max observed declination is 26-30° max
    motion.setDeclination(angle);
    if(riot.isDebug())
      Serial.printf("%s %f\n", TEXT_DECLINATION, motion.getDeclination());
    return(true);
  }

  else if(!strncmp(TEXT_ORIENTATION, line, strlen(TEXT_ORIENTATION))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , MAX_BOARD_ORIENTATION);
    motion.setOrientation(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_ORIENTATION, motion.getOrientation());
    return(true);
  }

  else if(!strncmp(TEXT_BNO_ORIENT, line, strlen(TEXT_BNO_ORIENT))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , 7);
    bno055.SetPos(val);
    bno055.Set_Mode(NDOF);  // 9DoF fusion
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_BNO_ORIENT, bno055.orientation);
    return(true);
  }

  else if(!strncmp(TEXT_ACC_RANGE, line, strlen(TEXT_ACC_RANGE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, ACC_2G, ACC_16G);
    lsm6d.setAccRange(val);
    motion.begin();
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_ACC_RANGE, lsm6d.getAccRange());
    return(true);
  }
  else if(!strncmp(TEXT_GYRO_RANGE, line, strlen(TEXT_GYRO_RANGE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, GYRO_250DPS, GYRO_2000DPS);
    lsm6d.setGyroRange(val);
    motion.begin();
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_RANGE, lsm6d.getGyroRange());
    return(true);
  }
  else if(!strncmp(TEXT_MAG_RANGE, line, strlen(TEXT_MAG_RANGE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val, MAG_4GAUSS, MAG_16GAUSS);
    lis3mdl.setRange(val);
    motion.begin();
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_MAG_RANGE, lis3mdl.getRange());
    return(true);
  }  
  else if(!strncmp(TEXT_GYRO_GATE, line, strlen(TEXT_GYRO_GATE))) {
    index = skipToValue(line);
    float gate = atof(&line[index]);
    motion.setGyroGate(gate);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_GATE, motion.getGyroGate());
    return(true);
  }  
  else if(!strncmp(TEXT_GYRO_HPF, line, strlen(TEXT_GYRO_HPF))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    lsm6d.setGyroHpf(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_HPF, lsm6d.getGyroHpf());
    return(true);
  }  
  else if(!strncmp(TEXT_BARO_MODE, line, strlen(TEXT_BARO_MODE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , BARO_MAX_SAMPLING_MODE);
    bmp390.setSamplingMode(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_BARO_MODE, bmp390.getSamplingMode());
    return(true);
  }
  else if(!strncmp(TEXT_BARO_REF, line, strlen(TEXT_BARO_REF))) {
    index = skipToValue(line);
    float alt = atof(&line[index]);
    bmp390.setRefAltitude(alt);
    if(riot.isDebug())
      Serial.printf("%s %f\n", TEXT_BARO_REF, bmp390.getRefAltitude());
    return(true);
  }
  else if(!strncmp(TEXT_CHARGE_MODE, line, strlen(TEXT_CHARGE_MODE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , MAX_CHARGE_MODE);
    riot.setChargingMode(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_CHARGE_MODE, riot.getChargingMode());
    return(true);
  }
  else if(!strncmp(TEXT_CPU_SPEED, line, strlen(TEXT_CPU_SPEED))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,80 , 240);
    riot.setCpuSpeed(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_CPU_SPEED, riot.getCpuSpeed());
    return(true);
  }
  else if(!strncmp(TEXT_CPU_DOZE, line, strlen(TEXT_CPU_DOZE))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,80 , 240);
    riot.setCpuDoze(val);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_CPU_DOZE, riot.getCpuDoze());
    return(true);
  }
  else if(!strncmp(TEXT_LED_COLOR, line, strlen(TEXT_LED_COLOR))) {
    index = skipToValue(line);
    if (index) {
      int tempVal;
      int colorArray[3];
      CRGBW8 color;
      char c = line[index];
      if (isDigit(c)) { // Normal color definition
        for (i = 0 ; i < 3 ; i++) {
          if (index) {
            tempVal = atoi(&line[index]);
            //Serial.printf("Index=%d\n",index);
            //Serial.printf("index = %d - color[%d]=%d\n",index, i, tempVal);
            colorArray[i] = constrain(tempVal, 0, 255);
            index = skipToNextValue(line, index);
          }
        }
        color = CRGBW8(colorArray[0], colorArray[1], colorArray[2]);
        riot.setPixelColor(color);
      } // End of Classic Color Definition
      else { // using a color name from the dictionnary
        // Nothing to check, if we don't find the color, we get black
        color = getColorFromDictionary(&line[index]);
        riot.setPixelColor(color);
      }
      if(riot.isDebug()) {
        Serial.printf("%s \n", TEXT_LED_COLOR);
        (riot.getPixelColor()).print();
      }
    }
    return (true);
  }
  
  else if(!strncmp(TEXT_SLOW_BOOT, line, strlen(TEXT_SLOW_BOOT))) {
    index = skipToValue(line);
    val = atoi(&line[index]);
    val = constrain(val,0 , MAX_SLOW_BOOT);
    if( val != riot.getSlowBoot()) { // avoids flash wear
      riot.setSlowBoot(val);
      riot.writeSlowBoot();
    }
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_SLOW_BOOT, riot.getSlowBoot());
    return(true);
  }

  else if(!strncmp(TEXT_ACC_OFFSETX, line, strlen(TEXT_ACC_OFFSETX))) {
    index = skipToValue(line);
    motion.setAccelBias(atoi(&line[index]), X_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_ACC_OFFSETX, motion.getAccelBiasRaw(X_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_ACC_OFFSETY, line, strlen(TEXT_ACC_OFFSETY))) {
    index = skipToValue(line);
    motion.setAccelBias(atoi(&line[index]), Y_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_ACC_OFFSETY, motion.getAccelBiasRaw(Y_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_ACC_OFFSETZ, line, strlen(TEXT_ACC_OFFSETZ))) {
    index = skipToValue(line);
    motion.setAccelBias(atoi(&line[index]), Z_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_ACC_OFFSETZ, motion.getAccelBiasRaw(Z_AXIS));
    return(true);
  }  
  else if(!strncmp(TEXT_GYRO_OFFSETX, line, strlen(TEXT_GYRO_OFFSETX))) {
    index = skipToValue(line);
    motion.setGyroBias(atoi(&line[index]), X_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_OFFSETX, motion.getGyroBiasRaw(X_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_GYRO_OFFSETY, line, strlen(TEXT_GYRO_OFFSETY))) {
    index = skipToValue(line);
    motion.setGyroBias(atoi(&line[index]), Y_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_OFFSETY, motion.getGyroBiasRaw(Y_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_GYRO_OFFSETZ, line, strlen(TEXT_GYRO_OFFSETZ))) {
    index = skipToValue(line);
    motion.setGyroBias(atoi(&line[index]), Z_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_GYRO_OFFSETZ, motion.getGyroBiasRaw(Z_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_MAG_OFFSETX, line, strlen(TEXT_MAG_OFFSETX))) {
    index = skipToValue(line);
    motion.setMagBias(atoi(&line[index]), X_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_MAG_OFFSETX, motion.getMagBiasRaw(X_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_MAG_OFFSETY, line, strlen(TEXT_MAG_OFFSETY))) {
    index = skipToValue(line);
    motion.setMagBias(atoi(&line[index]), Y_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_MAG_OFFSETY, motion.getMagBiasRaw(Y_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_MAG_OFFSETZ, line, strlen(TEXT_MAG_OFFSETZ))) {
    index = skipToValue(line);
    motion.setMagBias(atoi(&line[index]), Z_AXIS);
    if(riot.isDebug())
      Serial.printf("%s %d\n", TEXT_MAG_OFFSETZ, motion.getMagBiasRaw(Z_AXIS));
    return(true);
  }
  else if(!strncmp(TEXT_SOFT_IRON_MATRIX1, line, strlen(TEXT_SOFT_IRON_MATRIX1))) {
    float vect[3];
    index = skipToValue(line);
    if(index) {
      for(int i = 0; i < 3; i++) {
        vect[i] = atof(&line[index]);
        index = skipToNextValue(line, index);
      }
    }
    motion.setSoftIronMatrix(vect, X_AXIS);
    if(riot.isDebug())
      Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX1, vect[0], vect[1], vect[2]);
    return(true);
  }
  else if(!strncmp(TEXT_SOFT_IRON_MATRIX2, line, strlen(TEXT_SOFT_IRON_MATRIX2))) {
    float vect[3];
    index = skipToValue(line);
    if(index) {
      for(int i = 0; i < 3; i++) {
        vect[i] = atof(&line[index]);
        index = skipToNextValue(line, index);
      }
    }
    motion.setSoftIronMatrix(vect, Y_AXIS);
    if(riot.isDebug())
      Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX2, vect[0], vect[1], vect[2]);
    return(true);
  }
  else if(!strncmp(TEXT_SOFT_IRON_MATRIX3, line, strlen(TEXT_SOFT_IRON_MATRIX3))) {
    float vect[3];
    index = skipToValue(line);
    if(index) {
      for(int i = 0; i < 3; i++) {
        vect[i] = atof(&line[index]);
        index = skipToNextValue(line, index);
      }
    }
    motion.setSoftIronMatrix(vect, Z_AXIS);
    if(riot.isDebug())
      Serial.printf("%s [ %f %f %f ]\n", TEXT_SOFT_IRON_MATRIX3, vect[0], vect[1], vect[2]);
    return(true);
  }
  
  else if(!strncmp(TEXT_BETA, line, strlen(TEXT_BETA))) {
    index = skipToValue(line);
    motion.setBeta(atof(&line[index]));
    if(riot.isDebug())
      Serial.printf("%s %f\n", TEXT_BETA, motion.getBeta());
    return(true);
  }
  else if(!strncmp(TEXT_DEFAULTS, line, strlen(TEXT_DEFAULTS))) {
    // Re open in write mode
    restoreDefaults(false);
    return(true);
  }

  else if(!strncmp(TEXT_FORMAT, line, strlen(TEXT_FORMAT))) {
    format();
    return(true);
  }
  else if(!strncmp(TEXT_AUTO_TEST, line, strlen(TEXT_AUTO_TEST))) {
    autoTest();
    return(true);
  }
  else if(!strncmp(TEXT_CALIBRATE, line, strlen(TEXT_CALIBRATE))) {
    // re enable calibration timer
    riot.setCalibrationTimer(riot.getCalibrationTimer());
    motion.nextStep(true);  // overrides switch action
    return(true);
  }
  else if(!strncmp(TEXT_AUTOCAL_MAG, line, strlen(TEXT_AUTOCAL_MAG))) {    
    motion.runAutoCalMag();
    return(true);
  }
  else if(!strncmp(TEXT_AUTOCAL_MOTION, line, strlen(TEXT_AUTOCAL_MOTION))) {    
    motion.runAutoCalMotion();
    return(true);
  }
 
  else if(!strncmp(TEXT_VERSION, line, strlen(TEXT_VERSION))) {
    Serial.printf("%s\n", riot.getVersion());
    return(true);
  }
  else if(!strncmp(TEXT_WIFI, line, strlen(TEXT_WIFI))) {
    riot.printCurrentNet();
    riot.printWifiData();
    return(true);
  }
  else if(!strncmp(TEXT_GO_COMMAND, line, strlen(TEXT_GO_COMMAND))) {
    motion.nextStep(true);  // Proceed with calibration - emulates the switch press
    return(true);
  }
  else if(!strncmp(TEXT_CANCEL_COMMAND, line, strlen(TEXT_CANCEL_COMMAND))) {
    motion.cancel(true);  // Cancel calibration - emulates the switch press
    return(true);
  }
  else if(!strncmp(TEXT_LOG_MOTION, line, strlen(TEXT_LOG_MOTION))) {
    index = skipToValue(line);
    riot.setLogMotion(atof(&line[index]));
    return(true);
  }
  else if(!strncmp(TEXT_LOG_MAG, line, strlen(TEXT_LOG_MAG))) {
    index = skipToValue(line);
    riot.setLogMag(atof(&line[index]));
    return(true);
  }
  else if(!strncmp(TEXT_VBATT, line, strlen(TEXT_VBATT))) {
    index = skipToValue(line);
    Serial.printf("%s %f volts\n", TEXT_VBATT, readBatteryVoltage());
    return(true);
  }
  else if(!strncmp(TEXT_VUSB, line, strlen(TEXT_VUSB))) {
    index = skipToValue(line);
    Serial.printf("%s %f volts\n", TEXT_VUSB, readUsbVoltage());
    return(true);
  }
  return(false);
}



bool storeConfig(void) {
  FIL file;
  UINT write;
  int totalWrite = 0;
  char stringBuffer[MAX_PATH_LEN];
  char *fileBuffer;
  IPAddress tempIP;

  int writeTime = millis();
  
  // Open file
  sprintf(stringBuffer, "%s", CONFIG_FILE);
  if (f_open(&file, stringBuffer, FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
    return false;

  if (riot.isDebug())
    Serial.printf("%s Saving %s\n", TEXT_FILE_LOG, stringBuffer);

  fileBuffer = new char[CONFIG_MAX_LINE_LEN];

  f_lseek(&file, 0); // rewinds
  f_truncate(&file);
  
  //////////////////////////////////////////////////////////////////////:
  // all general config params
  memset(fileBuffer, '\0', CONFIG_MAX_LINE_LEN);
  sprintf(fileBuffer, "//R-IoT Configuration - fw: %s%s", riot.getVersion(), TEXT_FILE_EOL);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_DEBUG, riot.isDebug());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_WIFI_MODE, riot.getOperatingMode());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_STRING, TEXT_SSID, riot.getSSID());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_STRING, TEXT_PASSWORD, riot.getPassword());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_STRING, TEXT_MDNS, riot.getBonjour());
  strcat(fileBuffer, stringBuffer);
  
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_DHCP, riot.isDHCP());
  strcat(fileBuffer, stringBuffer);  
  
  tempIP = riot.getOwnIP();
  sprintf(stringBuffer, "%s=%u.%u.%u.%u\r\n", TEXT_OWNIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
  strcat(fileBuffer, stringBuffer);
  tempIP = riot.getDestIP();
  sprintf(stringBuffer, "%s=%u.%u.%u.%u\r\n", TEXT_DESTIP, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
  strcat(fileBuffer, stringBuffer);
  tempIP = riot.getGatewayIP();
  sprintf(stringBuffer, "%s=%u.%u.%u.%u\r\n", TEXT_GATEWAY, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
  strcat(fileBuffer, stringBuffer);
  tempIP = riot.getSubnetMask();
  sprintf(stringBuffer, "%s=%u.%u.%u.%u\r\n", TEXT_MASK, tempIP[0], tempIP[1], tempIP[2], tempIP[3]);
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_PORT, riot.getDestPort());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_RECEIVE_PORT, riot.getReceivePort());
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_MASTER_ID, riot.getID());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_WIFI_POWER, riot.getWifiPower());
  strcat(fileBuffer, stringBuffer); 
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_SAMPLE_RATE, motion.getSampleRate());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_REMOTE, riot.isOSCinput());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_FORCE_CONFIG, riot.isForcedConfig());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_CALIBRATION, riot.getCalibrationTimer());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_CHARGE_MODE, riot.getChargingMode());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, "%s=%u,%u,%u\r\n", TEXT_LED_COLOR, riot.getPixelColor()[RED], riot.getPixelColor()[GREEN], riot.getPixelColor()[BLUE]);
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_CPU_SPEED, riot.getCpuSpeed());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_CPU_DOZE, riot.getCpuDoze());
  strcat(fileBuffer, stringBuffer);
  
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_FLOAT, TEXT_DECLINATION, motion.getDeclination());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_ORIENTATION, motion.getOrientation());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_BNO_ORIENT, bno055.orientation);
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_ACC_RANGE, lsm6d.getAccRange());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_GYRO_RANGE, lsm6d.getGyroRange());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_MAG_RANGE, lis3mdl.getRange());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_FLOAT, TEXT_GYRO_GATE, motion.getGyroGate());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_GYRO_HPF, lsm6d.getGyroHpf());
  strcat(fileBuffer, stringBuffer);
    
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM, TEXT_BARO_MODE, bmp390.getSamplingMode());
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_FLOAT, TEXT_BARO_REF, bmp390.getRefAltitude());
  strcat(fileBuffer, stringBuffer);
   
  f_write(&file, fileBuffer, strlen(fileBuffer), &write);
  totalWrite += write;
  f_sync(&file);
  memset(fileBuffer, '\0', CONFIG_MAX_LINE_LEN);

   // Calibration data
  sprintf(fileBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_ACC_OFFSETX, motion.getAccelBiasRaw(X_AXIS));
  //strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_ACC_OFFSETY, motion.getAccelBiasRaw(Y_AXIS));
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_ACC_OFFSETZ, motion.getAccelBiasRaw(Z_AXIS));
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_GYRO_OFFSETX, motion.getGyroBiasRaw(X_AXIS));
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_GYRO_OFFSETY, motion.getGyroBiasRaw(Y_AXIS));
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_GYRO_OFFSETZ, motion.getGyroBiasRaw(Z_AXIS));
  strcat(fileBuffer, stringBuffer);

  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_MAG_OFFSETX, motion.getMagBiasRaw(X_AXIS));
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_MAG_OFFSETY, motion.getMagBiasRaw(Y_AXIS));
  strcat(fileBuffer, stringBuffer);
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_SIGNED, TEXT_MAG_OFFSETZ, motion.getMagBiasRaw(Z_AXIS));
  strcat(fileBuffer, stringBuffer);

  f_write(&file, fileBuffer, strlen(fileBuffer), &write);
  totalWrite += write;
  f_sync(&file);
  memset(fileBuffer, '\0', CONFIG_MAX_LINE_LEN);

  // Soft Iron Matrix storage
  float *pf;
  pf = motion.getSoftIronMatrixRow(X_AXIS);
  sprintf(stringBuffer, "%s=%f,%f,%f%s", TEXT_SOFT_IRON_MATRIX1, pf[0],pf[1],pf[2], TEXT_FILE_EOL);
  strcat(fileBuffer, stringBuffer);
  pf = motion.getSoftIronMatrixRow(Y_AXIS);
  sprintf(stringBuffer, "%s=%f,%f,%f%s", TEXT_SOFT_IRON_MATRIX2, pf[0],pf[1],pf[2], TEXT_FILE_EOL);
  strcat(fileBuffer, stringBuffer);
  pf = motion.getSoftIronMatrixRow(Z_AXIS);
  sprintf(stringBuffer, "%s=%f,%f,%f%s", TEXT_SOFT_IRON_MATRIX3, pf[0],pf[1],pf[2], TEXT_FILE_EOL);
  strcat(fileBuffer, stringBuffer);
  
  sprintf(stringBuffer, TEXT_FILE_SINGLE_PARAM_FLOAT, TEXT_BETA, motion.getBeta());
  strcat(fileBuffer, stringBuffer);

  eol(fileBuffer, 4);
  
  f_write(&file, fileBuffer, strlen(fileBuffer), &write);
  totalWrite += write;
  writeTime = millis() - writeTime;
  Serial.printf("%s R-IoT Config saved in %dms - Wrote %d bytes\n", TEXT_FILE_LOG, writeTime, totalWrite);
  f_close(&file);
  delete[] fileBuffer;
 
  return(true);
}


// Assumes the file is already opened for writing
void restoreDefaults(bool save) {
  if(riot.isDebug())
    Serial.printf("%s Restoring default values\n", TEXT_FILE_LOG);
  motion.init();
  lsm6d.setAccRange(ACC_8G);
  lsm6d.setGyroRange(GYRO_2000DPS);
  lis3mdl.setRange(MAG_4GAUSS);  
  riot.init();
  if(save)
    storeConfig();
}
