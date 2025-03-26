#ifndef _MAIN_H
#define _MAIN_H

// A place for global objects and defines

#include <Arduino.h>
#include <stdio.h>
#include <strings.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h> // Access to mDNS related functionalities
#include <WebServer.h>
#include <Update.h>
#include <U8g2lib.h>
#include "./src/ota.h "
#include "./src/functions.h"
#include "./src/colors.h"
#include "./src/Switches.h"

// FFAT + MSD libs
#include "FS.h"
#include "FFat.h"
#include "ff.h"
#include "diskio.h"
#include "USB.h"
#include "USBMSC.h"

#define IPV4_SIZE       4
#define MAC_SIZE        6
#define MAX_SERIAL_LEN  200
#define MAX_STRING_LEN  200
#define MAX_PATH_LEN    80


extern WiFiUDP udpPacket;
extern WiFiUDP configPacket;
extern WebServer httpServer;

extern WiFiClient client;
extern CRGBW8 blinkColor;
// Mass storage driver using TinyUSB
extern USBMSC MSC;

extern char serialBuffer[MAX_SERIAL_LEN];
extern unsigned char serialIndex;

// OLED 128x64 pixels support
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;

#define VERSION "IRCAM R-IoT"
#define FW_VERSION_MAJOR  "3"
#define FW_VERSION_MINOR  "18"
#define FW_VERSION_PATCH  "3"

#define FW_UPDATE_FILE    "/update.bin"

////////////////////////////////////////////////////////////////////////////////////////////////

/*   BOARD LAYOUT
 *   +-------------------------------------------------------------------------+
 *   |     Switch    O     O     O     O     O     O     O     O     O     O   |
 *   |              BAT   GND  VUSB  3.3V    A5    A4    A3    A2    A1    A0  |
 *   |                                                                         |
 *   |                                                                         |
 *   |--+                                                                      |
 *   |  |  USB-C                     R-IOT v3                                  |
 *   |  |                                                                      |
 *   |--+                                                                      |
 *   |                                                                         |
 *   |                                                                         |
 *   |   39    40    41    38  RESET  3.3V  SCL   SDA   TXO   RXI   GND  PIX/0 |
 *   |   O     O     O     O     O     O     O     O     O     O     O     O   |
 *   +-------------------------------------------------------------------------+
 *   
 */

////////////////////////////////////////////////////////////////////////////////////////////////
// IO definitions
#define SWITCH_INPUT      38    // This is the mode switch (configuration / AP mode or normal usage as wifi station). Also exported in the OSC message
#define PIN_OPTION_SW     38    // Exported option switch
#define SWITCH2_INPUT     41    //  Free to use on the side of the board, configured as an input with pullup in the FW and exported in the OSC message
#define REMOTE_OUTPUT     40    //  Free to use on the side of the board, configured as an output in the FW and controlled by OSC message

#define ANALOG_INPUT      A0    // A0-A5 available for measuring 12 bit voltages
#define ANALOG2_INPUT     A1    
#define PIN_BATT_VOLTAGE  10
#define PIN_USB_VOLTAGE   12    // on ADC too
#define PIN_CHARGE_STATUS 13    // I/O (not ADC)
#define BATT_INPUT        PIN_BATT_VOLTAGE

#define PIN_NEOPIXEL      0     // GPIO 0 is also hooked to the "flash" onboard tactile switch. Can be used to get in bootloader mode by shorting it
#define PIN_SWITCH_GND    11    // Switched ground for the battery voltage divider (disabled during sleep mode to avoid current draw)

#define PIN_MOSI          35    // SPI (standard ESP32 pins)
#define PIN_SCK           36
#define PIN_MISO          37

#define BATTERY_VOLTAGE_SCALE         0.00238476f   // 3.3f * 2.96f / 4096.f 
#define USB_VOLTAGE_SCALE             0.00121655f   // 3.3f * 1.51f / 4096.f
#define ANALOG_INPUT_VOLTAGE_SCALE    0.00080566f   // 3.3f / 4096.f

// DEFAULTS
#define DEFAULT_MSD_VOLUME_NAME   "RIOT3-MSD"
#define DEFAULT_UDP_PORT          8888
#define DEFAULT_UDP_SERVICE_PORT  7777
#define DEFAULT_SSID              "riot"
#define DEFAULT_AP_PASSWORD       "riot1234"
#define DEFAULT_SAMPLE_RATE       5
#define DEFAULT_ID                0
#define CONFIG_FILE               "config.txt"
#define VERSION_FILE              "version.txt"
#define CONFIG_MODE_TIMEOUT       2000          // Time to press on the switch to start the webserver      

#define DECLINATION    1.83   // Paris declination 01/02/2025

// WIFI & WEB preferences
#define MAX_CLIENTS    5

typedef union uWord {
  int16_t Value;
  unsigned char Val[sizeof(int16_t)];
} Word;

typedef union udWord {
  uint32_t Value;
  unsigned char Val[sizeof(uint32_t)];
} dWord;

#endif


/*
 * The MIT License (MIT)
 *
 * Copyright (c) Henry Gabryjelski
 * Copyright (c) Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef COMPILE_DATE_H
#define COMPILE_DATE_H

// Help enable build to be deterministic.
// Allows Ghostfat to generate 100% reproducible images across compilations.
// Reproducible builds are also important for other reasons.
// See generally, https://reproducible-builds.org/
#ifndef COMPILE_DATE
  #define COMPILE_DATE __DATE__
#endif
#ifndef COMPILE_TIME
  #define COMPILE_TIME __TIME__
#endif

#define COMPILE_YEAR_INT ((( \
  (COMPILE_DATE [ 7u] - '0')  * 10u + \
  (COMPILE_DATE [ 8u] - '0')) * 10u + \
  (COMPILE_DATE [ 9u] - '0')) * 10u + \
  (COMPILE_DATE [10u] - '0'))

#define COMPILE_MONTH_INT  ( \
    (COMPILE_DATE [2u] == 'n' && COMPILE_DATE [1u] == 'a') ?  1u  /*Jan*/ \
  : (COMPILE_DATE [2u] == 'b'                            ) ?  2u  /*Feb*/ \
  : (COMPILE_DATE [2u] == 'r' && COMPILE_DATE [1u] == 'a') ?  3u  /*Mar*/ \
  : (COMPILE_DATE [2u] == 'r'                            ) ?  4u  /*Apr*/ \
  : (COMPILE_DATE [2u] == 'y'                            ) ?  5u  /*May*/ \
  : (COMPILE_DATE [2u] == 'n'                            ) ?  6u  /*Jun*/ \
  : (COMPILE_DATE [2u] == 'l'                            ) ?  7u  /*Jul*/ \
  : (COMPILE_DATE [2u] == 'g'                            ) ?  8u  /*Aug*/ \
  : (COMPILE_DATE [2u] == 'p'                            ) ?  9u  /*Sep*/ \
  : (COMPILE_DATE [2u] == 't'                            ) ? 10u  /*Oct*/ \
  : (COMPILE_DATE [2u] == 'v'                            ) ? 11u  /*Nov*/ \
  :                                                          12u  /*Dec*/ )

#define COMPILE_DAY_INT ( \
   (COMPILE_DATE [4u] == ' ' ? 0 : COMPILE_DATE [4u] - '0') * 10u + \
   (COMPILE_DATE [5u] - '0')                                             \
   )

// __TIME__ expands to an eight-character string constant
// "23:59:01", or (if cannot determine time) "??:??:??"
#define COMPILE_HOUR_INT ( \
   (COMPILE_TIME [0u] == '?' ? 0 : COMPILE_TIME [0u] - '0') * 10u \
 + (COMPILE_TIME [1u] == '?' ? 0 : COMPILE_TIME [1u] - '0')       )

#define COMPILE_MINUTE_INT ( \
   (COMPILE_TIME [3u] == '?' ? 0 : COMPILE_TIME [3u] - '0') * 10u \
 + (COMPILE_TIME [4u] == '?' ? 0 : COMPILE_TIME [4u] - '0')       )

#define COMPILE_SECONDS_INT ( \
   (COMPILE_TIME [6u] == '?' ? 0 : COMPILE_TIME [6u] - '0') * 10u \
 + (COMPILE_TIME [7u] == '?' ? 0 : COMPILE_TIME [7u] - '0')       )


#define COMPILE_DOS_DATE ( \
  ((COMPILE_YEAR_INT  - 1980u) << 9u) | \
  ( COMPILE_MONTH_INT          << 5u) | \
  ( COMPILE_DAY_INT            << 0u) )

#define COMPILE_DOS_TIME ( \
  ( COMPILE_HOUR_INT    << 11u) | \
  ( COMPILE_MINUTE_INT  <<  5u) | \
  ( COMPILE_SECONDS_INT <<  0u) )

#endif // COMPILE_DATE_H
