

//////////////////////////////////////////////////////////////////////////////////////////////
// General (sub)routines (C) not specifically used within a class. Somes are hooks, others
// are generic C functions

#ifndef _ROUTINES_H
#define _ROUTINES_H

#include <Arduino.h>
#include <stdarg.h>
#include "main.h"
#include "textfile.h"
#include "riot.h"
#include "osc.h"

// GPIOs 0-31
#define PIN_CLEAR(_pin)                 GPIO.out_w1tc = (uint32_t)(1<<_pin)
#define PIN_SET(_pin)                   GPIO.out_w1ts = (uint32_t)(1<<_pin)

// GPIOs 32+
#define PIN1_CLEAR(_pin)                 GPIO.out1_w1tc.data = (uint32_t)(1<<(_pin-32))
#define PIN1_SET(_pin)                   GPIO.out1_w1ts.data = (uint32_t)(1<<(_pin-32))


void setLedColor(CRGBW8 color);
void printToOSC(char *StringMessage);
void die();
void restoreDefaults(bool save = false);
void reset();

int readBatteryRaw(void);
float readBatteryVoltage(void);
float readUsbVoltage(void);
uint8_t readChargeStatus(void);

void setModemSleep();
void wakeModemSleep();
void format();

// file & dir helpers
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void deleteFile(fs::FS &fs, const char * path);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
void readFile(fs::FS &fs, const char * path);
bool printFile(char *pathname);

void updateFromFS(fs::FS &fs);
void performUpdate(Stream &updateSource, size_t updateSize);




#endif
