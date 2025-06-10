#ifndef _SIMPLE_OSC_H
#define _SIMPLE_OSC_H

/* Copyright (c) 2014-present IRCAM – Centre Pompidou (France, Paris)

    All rights reserved.

    Emmanuel FLETY - IRCAM - PIP Team / 2017-2025 
    
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


#include "main.h"

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
extern simpleOSC accelerometerOSC, gyroscopeOSC, magnetometerOSC, barometerOSC, temperatureOSC, gravityOSC, headingOSC, quaternionsOSC, eulerOSC, controlOSC, batteryOSC, analogInputsOSC, bno055EulerOSC, bno055QuatOSC;
extern simpleOSC printOscMessage;


#endif
