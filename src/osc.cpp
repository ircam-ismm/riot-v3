// Basic handling of an OSC packet

#include <stdio.h>
#include <string.h>
#include "osc.h"

// This class should rely on main.h see how to make it totally independant. Check if libs above are needed 
// for compilation

////////////////////////////////////////////////////////////////////////////////////
// Pre-allocate and fills the OSC structures
// adds the address and the type tags like "/address/subaddress ,iiiii"
// and finds the total packet size and begining of the data for further simplified
// update. This saves a lot of time by NOT re computing the OSC structure
// and padding, and limits data updates to RAM moves.
void simpleOSC::begin(char *oscAddress, const char* typeTagString) {
  int i;

  _slots = strlen(typeTagString);
  // Allocate the packet buffer with a rough estimate of the size + overhead
  int rawSize = strlen(oscAddress) + 1 + (_slots * sizeof(int32_t)) + (2 * _slots) + 20;
  if(rawSize%4)
    rawSize = rawSize + 4;
  end();
  _buf = new uint8_t[rawSize];
#ifdef DEBUG_OSC  
  Serial.printf("Allocated (raw) size for OSC buffer = %d\n", rawSize);
#endif
  _pBuf = _buf;
  _packetSize = strlen(oscAddress);
  strcpy((char*)_pBuf, oscAddress);
  _pBuf += _packetSize;

  pad(true);
 
  // adds the comma char (separator for typetags)
  *_pBuf = ',';
  _pBuf++;
  _packetSize++;
  
  // adds the type tags - could be memcpy() here
  for(i = 0 ; i < _slots ; i++) {
    *_pBuf = typeTagString[i];
    _pBuf++;  
  }
  _packetSize += _slots;

  pad(true);
  
  // Stores where the actual data start
  _pData = _pBuf;
  
  // Adds size of data
  _packetSize += _slots * sizeof(uint32_t);
  _pBuf += _slots * sizeof(uint32_t);
	
  // Dumb check as at that point it's supposed to be 4-byte aligned
  pad(false);
  
  *(_pBuf+1) = '\0';   // Safety terminator of the packet so that any strlen() works properly
  _initialized = true;
#ifdef DEBUG_OSC  
  Serial.printf("[OSC] Calculated OSC packet size = %d\n", _packetSize);
#endif  
  // Eventually add here a warning if we have miscomputed the buffer size and have no packet size overhead
  // + trigger some exception (like end() the packet)
  if(_packetSize > rawSize) {
    Serial.printf("[OSC] OSC buffer undersized. Need %d bytes, have %d bytes\n", _packetSize, rawSize);
    end();
  }
}


void simpleOSC::end() {
  delete _buf; 
  _packetSize = 0;
  _initialized = false;
  
}

void simpleOSC::pad(bool force) {
  // In certain locations of the OSC packet, 
  // We can't stop on an aligned %4, as we need at least one zero terminator, like for the address
  // or the type tags location
  if(force) {
    if(!(_packetSize%4)) {
      *_pBuf = '\0';
      _packetSize++;
      _pBuf++;
    }
  }
  
  // 4 byte padding / stuffing
  while(_packetSize%4) {
    *_pBuf = '\0';
    _packetSize++;
    _pBuf++;
  }  
}



// An OSC text message creator 
void simpleOSC::createString(char *oscAddress, char *stringMessage) {
  // Allocate the packet buffer with a rough estimate of the size + overhead
  int rawSize = strlen(oscAddress) + 1 + 1 + strlen(stringMessage) + OSC_BUFFER_OVERHEAD;
  if(rawSize%4)
    rawSize = rawSize + 4;
    
  end();
  _buf = new uint8_t[rawSize];
#ifdef DEBUG_OSC    
  Serial.printf("Allocated (raw) size for OSC buffer = %d\n", rawSize);
#endif  
  _pBuf = _buf;
  _packetSize = strlen(oscAddress);
  strcpy((char*)_pBuf, oscAddress);
  _pBuf += _packetSize;
  
  pad(true);
  
  // adds the comma char (separator for typetags)
  *_pBuf = ',';
  _pBuf++;
  _packetSize++;
  // adds the string type tags
  *_pBuf = 's';
  _pBuf++;
  _packetSize++;
  
  // We can't stop on an aligned %4, as we need at least one zero terminator in the address
  pad(true);

  // Stores where the actual data start
  _pData = _pBuf;
  
  // Adds size of data (string length)
  strcpy((char*)_pBuf, stringMessage);
  _packetSize += strlen(stringMessage);
  _pBuf += strlen(stringMessage);

  // There is a mandatory string terminator that isn't inserted by the
  // string copy above (as it stops without including it)	
  *_pBuf = '\0';
  _pBuf++;
  _packetSize++;

  // Eventually pads the string to %4
  pad(false);
  
  *(_pBuf+1) = '\0';   // Paranoïa safety terminator of the packet so that any strlen() works properly
#ifdef DEBUG_OSC  
  Serial.printf("Calculated OSC packet size = %d\n", _packetSize);
#endif  
  _initialized = true;
}

// Assumes the data pointer was set to the expected location with rewind() or skipTo()
// OSC is big endian. Source endianness isn't made generic here and only on purpose with the 
// processor architecture used (ARM32)
void simpleOSC::addShort(int16_t val) {
  if(!_initialized)
    return;

  uint32_t net_value;
  int32_t cast = (int32_t)val;
  memcpy(&net_value, &cast, sizeof(int32_t));
  net_value = htonl(net_value);
  memcpy(_pBuf, &net_value, sizeof(net_value));
  _pBuf += sizeof(int32_t);   // even with as short, space used remains 32-bit wide
}

// Adds a 16 bit value from the Word (union) struct / data type
void simpleOSC::addWord(Word theWord) {

  uint32_t net_value;
  int32_t cast = (int32_t)theWord.Value;
  memcpy(&net_value, &cast, sizeof(int32_t));
  net_value = htonl(net_value);
  memcpy(_pBuf, &net_value, sizeof(net_value));
  _pBuf += sizeof(int32_t);   // even with as short, space used remains 32-bit wide
}

void simpleOSC::addInt(int32_t val) {
  uint32_t net_value;
  memcpy(&net_value, &val, sizeof(int32_t));
  net_value = htonl(net_value);
  memcpy(_pBuf, &net_value, sizeof(net_value));
  _pBuf += sizeof(int32_t);   // even with as short, space used remains 32-bit wide
}

void simpleOSC::addFloat(float val) {
  uint32_t net_value;
  memcpy(&net_value, &val, sizeof(float));
  net_value = htonl(net_value);
  memcpy(_pBuf, &net_value, sizeof(net_value));
  _pBuf += sizeof(float);   // even with as short, space used remains 32-bit wide
}


void simpleOSC::rewind() {
  if(!_initialized)
    return;

  if(_pBuf && _pData)
    _pBuf = _pData;
}



/////////////////////////////////////////////////////////////////
// Bundle

////////////////////////////////////////////////////////////////////////////////////
// a bundle is mostly the concatenation of several OSC messages, however we must 
// build it in a contiguous memory space so that the UDP packet sender can transmit
// it. As a result, OSC messages are created on the side then added when time comes,
// after filling / forging them with the dataset.
// buffsize must account for the buffer size of each added OSC message + a uint32_t size
// for each of them
void simpleBundle::begin(uint32_t buffSize) {
  uint64_t timetag = 0; // Immediate execution - timestamps are implemented in each OSC message
  
  if(buffSize%4)
    buffSize = buffSize + 4;
  end();
  _packetSize = strlen(STRING_BUNDLE_OSC);
  
  // Allocate the packet buffer with a rough estimate of the size + overhead
  buffSize += _packetSize + sizeof(timetag) + OSC_BUFFER_OVERHEAD; 
  _buf = new uint8_t[buffSize];
#ifdef DEBUG_OSC  
  Serial.printf("Allocated (raw) size for OSC Bundle = %d\n", buffSize);
#endif
  _pBuf = _buf;
  strcpy((char*)_pBuf, STRING_BUNDLE_OSC);
  _pBuf += _packetSize;
  pad(true);
  
  memcpy(_pBuf, &timetag, sizeof(timetag));
  _pBuf += sizeof(timetag);
  _packetSize += sizeof(timetag);
  _pData = _pBuf;
  _initialized = true;
}


void simpleBundle::end() {
  delete _buf; 
  _packetSize = 0;
  _initialized = false;
  
}

void simpleBundle::pad(bool force) {
  // In certain locations of the OSC packet, 
  // We can't stop on an aligned %4, as we need at least one zero terminator, like for the address
  // or the type tags location
  if(force) {
    if(!(_packetSize%4)) {
      *_pBuf = '\0';
      _packetSize++;
      _pBuf++;
    }
  }
  
  // 4 byte padding / stuffing
  while(_packetSize%4) {
    *_pBuf = '\0';
    _packetSize++;
    _pBuf++;
  }  
}

void simpleBundle::rewind() {
  if(!_initialized)
    return;

  if(_pBuf && _pData)
    _pBuf = _pData;

  _packetSize = _pData - _buf;  
}

// Be sure to call rewind() before adding the first message, then
// concatenate them all.
bool simpleBundle::addMessage(uint8_t *buff, uint32_t buffSize) {
  if(!_initialized)
    return false;
    
  if(buffSize%4) {
    Serial.printf("[OSC] error : size not padded correctly %d bytes\n", buffSize);
    return false;
  }
  uint32_t net_value = buffSize;
  net_value = htonl(net_value);
  memcpy(_pBuf, &net_value, sizeof(net_value));
  _pBuf += sizeof(buffSize);
  _packetSize += sizeof(buffSize);
  
  memcpy(_pBuf, buff, buffSize);
  _pBuf += buffSize;
  _packetSize += buffSize;
  return true;
}


simpleBundle bundleOSC;
simpleOSC rawSensors;
simpleOSC accelerometerOSC, gyroscopeOSC, magnetometerOSC, barometerOSC, temperatureOSC, gravityOSC, headingOSC, quaternionsOSC, eulerOSC, controlOSC, batteryOSC, analogInputsOSC, bno055EulerOSC, bno055QuatOSC;
simpleOSC printOscMessage;
