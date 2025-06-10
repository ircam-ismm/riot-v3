
#ifndef _TEXTFILE_H
#define _TEXTFILE_H

#include "main.h"
#include "routines.h"
#include "textfile.h"
#include "sensors.h"
#include "motion.h"
#include "riot.h"

#define TEXT_FILE_SINGLE_PARAM          "%s=%u\r\n"
#define TEXT_FILE_SINGLE_PARAM_SIGNED   "%s=%d\r\n"
#define TEXT_FILE_SINGLE_PARAM_STRING   "%s=%s\r\n"
#define TEXT_FILE_SINGLE_PARAM_FLOAT    "%s=%f\r\n"

#define TEXT_FILE_EOL                   "\r\n"
#define TEXT_FILE_THREE_PARAMS          "%s=%d,%d,%d\r\n"

#define TEXT_COMMENT                "//"
#define TEXT_COMMENT2               "##"

// Serial or configuration file strings
#define TEXT_DEBUG                "debug"
#define TEXT_WIFI_MODE            "mode"
#define TEXT_SSID                 "ssid"
#define TEXT_OWNIP                "ownip"
#define TEXT_DESTIP               "destip"
#define TEXT_GATEWAY              "gateway"
#define TEXT_DNS                  "dns"
#define TEXT_MASK                 "mask"
#define TEXT_PORT                 "port"
#define TEXT_RECEIVE_PORT         "rxport"
#define TEXT_MDNS                 "mdns"
#define TEXT_MASTER_ID            "masterid"
#define TEXT_SAMPLE_RATE          "samplerate"
#define TEXT_PASSWORD             "pass"
#define TEXT_DHCP                 "dhcp"
#define TEXT_STANDALONE           "standalone"
#define TEXT_FORMAT               "format"
#define TEXT_SAVE_CONFIG          "savecfg"
#define TEXT_GET_CONFIG           "cfgrequest"
#define TEXT_PING                 "ping"
#define TEXT_ECHO                 "echo"
#define TEXT_AUTO_TEST            "autotest"
#define TEXT_DEFAULTS             "defaults"
#define TEXT_REBOOT               "reset"
#define TEXT_CALIBRATE            "calibrate"
#define TEXT_AUTOCAL_MAG          "autocalmag"
#define TEXT_AUTOCAL_MOTION       "autocalmotion"
#define TEXT_REMOTE               "remote"    // accepts OSC remote commands
#define TEXT_WIFI_POWER           "power"
#define TEXT_CPU_SPEED            "cpu"
#define TEXT_CPU_DOZE             "doze"
#define TEXT_WIFI_RSSI            "rssi"
#define TEXT_VERSION              "version"
#define TEXT_FORCE_CONFIG         "forceconfig"
#define TEXT_CALIBRATION          "calibration"
#define TEXT_CHARGE_MODE          "charger"
#define TEXT_DECLINATION          "declination"
#define TEXT_ORIENTATION          "orientation"
#define TEXT_BNO_ORIENT           "bno_orient"
#define TEXT_SLEEP                "sleep"    // Command to force the device to go to sleep mode
#define TEXT_WIFI                 "wifi"
#define TEXT_SLOW_BOOT            "slowboot"
#define TEXT_ACC_RANGE            "accrange"
#define TEXT_GYRO_RANGE           "gyrorange"
#define TEXT_MAG_RANGE            "magrange"
#define TEXT_GYRO_GATE            "gyrogate"
#define TEXT_GYRO_HPF             "gyrohpf"
#define TEXT_BARO_MODE            "baromode"
#define TEXT_BARO_REF             "baroref"
#define TEXT_PLI_LOW_HIGH         "plilh"
#define TEXT_GO_COMMAND           "GO"
#define TEXT_CANCEL_COMMAND       "CANCEL"
#define TEXT_LOG_MOTION           "logmotion"
#define TEXT_LOG_MAG              "logmag"
#define TEXT_LED_COLOR            "ledcolor"
#define TEXT_VBATT                "battery"
#define TEXT_VUSB                 "usb"

// Offsets & calibration matrix
#define TEXT_ACC_OFFSETX    "acc_offsetx"
#define TEXT_ACC_OFFSETY    "acc_offsety"
#define TEXT_ACC_OFFSETZ    "acc_offsetz"

#define TEXT_GYRO_OFFSETX   "gyr_offsetx"
#define TEXT_GYRO_OFFSETY   "gyr_offsety"
#define TEXT_GYRO_OFFSETZ   "gyr_offsetz"

#define TEXT_MAG_OFFSETX    "mag_offsetx"
#define TEXT_MAG_OFFSETY    "mag_offsety"
#define TEXT_MAG_OFFSETZ    "mag_offsetz"

#define TEXT_SOFT_IRON_MATRIX1   "soft_matrix1"
#define TEXT_SOFT_IRON_MATRIX2   "soft_matrix2"
#define TEXT_SOFT_IRON_MATRIX3   "soft_matrix3"


#define TEXT_BETA           "beta"

#define TEXT_ERROR_LOG      "[ERROR]"
#define TEXT_COMMENT_LOG    "[COMMENT]"
#define TEXT_DEBUG_LOG      "[DEBUG]"
#define TEXT_LOG_CONFIG     "[CONFIG]"
#define TEXT_LOG_WIFI       "[WIFI]"
#define TEXT_LOG_OSC        "[OSC]"
#define TEXT_FILE_LOG       "[FILE]"



#define CONFIG_MAX_LINE_LEN    2048
#define CONFIG_PRELOAD_SIZE    1024     // Reads 2 sector in a row (for SD), helps with access time. Increase to 

typedef bool (parsingCallback)(char* line);

// List of all C parsing callbacks
bool parseConfigCallback(char *line);

// C function for basic string parsing (legacy of previous code / boards)
// Called from C parsing callback, hence not in class
bool isComment(char *line);
bool parseConfigCallback(char *line);
int skipToValue(char *line, char separator = '=', bool strip = false);
int skipToNextValue(char *line, int startIndex, char separator = ',', bool strip = false);
void clearString(char* str, int size);
void purgeCRLF(char *str, int size);
void padString(char *str, char c, int padSize);
bool skipLine(char *line);
void eol(char* str, uint8_t howmany = 1);
bool storeConfig();
void configRequest();
void restoreDefaults(bool save);
bool processSerial(char *str) ;

// Configuration file handling
class configurationFile {
public:  

  configurationFile();
  ~configurationFile();

  bool begin(const char* path, bool writable);
  void end();
  void rewind() { f_lseek(&_file, 0); }
  bool readLine(char* line);
  void removeWhiteSpace(char* str);
  void convertTabs(char* str, char replaceWith);
  bool setFileRWPointer(uint32_t pos);
  uint32_t getFileRWPointer();
  void setCallback(parsingCallback *cb = NULL);
  bool parseConfigFile(bool debug = false);
    
  FIL _file;
  char stringBuffer[CONFIG_MAX_LINE_LEN];  
  bool _writable;
  char* _file_name;
  
  bool preload();
  uint8_t preload_buffer[CONFIG_PRELOAD_SIZE];
  uint32_t preload_size;
  uint32_t preload_offset;

  parsingCallback* parseSyntax;
  
};

extern configurationFile configFile;

#endif
