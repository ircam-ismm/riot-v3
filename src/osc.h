#ifndef _SIMPLE_OSC_H
#define _SIMPLE_OSC_H

// despite we're allocating dynamically now, we need to put a limit to
// stay under the size of a UDP packet until we add bundles
#define MAX_OSC_BUFFER_SIZE  1024

#include "main.h"
// light OSC parser / Micro OSC
#include <MicroOscUdp.h>

extern MicroOscUdp<1024> oscUdp;
void receivedOscMessage( MicroOscMessage& message);

//#define DEBUG_OSC     1

#define STRING_BUNDLE_OSC       "#bundle"
#define OSC_BUFFER_OVERHEAD     10

// A simplified class for OSC messages to send mono type formatted lists
class simpleOSC {

public:

  void begin(char *oscAddress, const char* typeTagString);
  void end();
  void rewind();
  void pad(bool force = false);
  void createString(char *oscAddress, char *stringMessage);
  void addShort(int16_t val);
  void addWord(Word theWord);
  void addInt(int32_t val);
  void addFloat(float val);
  uint8_t* getDataPointer() { return _pData; }
  uint8_t* getBuffer() { return _buf; }
  uint32_t getSize() { return _packetSize; }


private:
  void pad();

  uint8_t *_buf;
  uint8_t *_pData;        // to recall where data are, to insert them
  uint8_t *_pBuf;         // on going pointer when building the packet up
  uint32_t _packetSize;   // in bytes
  uint32_t _slots;        // how many dataslots in the message / OSC buffer
  bool _initialized = false;
};


// A bundle is mostly the concatenation of several OSC message + their size, built with the class above
// This class copies the contents of the message to a single, large buffer containing them all
// plus the #bundle header (padded)
class simpleBundle {

public:
  void begin(uint32_t buffSize);
  void end();
  void rewind();
  bool addMessage(uint8_t *buff, uint32_t buffSize);
  uint32_t getSize() { return _packetSize; }
  uint8_t* getDataPointer() { return _pData; }
  uint8_t* getBuffer() { return _buf; }


private:
  void pad(bool force = false);
  
  uint8_t *_buf;
  uint8_t *_pData;    // to recall where data start, to insert them
  uint8_t *_pBuf;     // on going pointer when building the packet up
  uint32_t _packetSize;   // in bytes
  uint64_t timetag = 0; // Immediate execution
  uint32_t net_timetag = htonl(timetag);

  bool _initialized = false;
};

extern simpleBundle bundleOSC;
extern simpleOSC rawSensors;
extern simpleOSC accelerometerOSC, gyroscopeOSC, magnetometerOSC, barometerOSC, temperatureOSC, gravityOSC, headingOSC, quaternionsOSC, eulerOSC, controlOSC, analogInputsOSC, bno055OSC;
extern simpleOSC printOscMessage;


#endif
