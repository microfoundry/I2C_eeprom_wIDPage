#pragma once
//
//    FILE: I2C_eeprom.h
//  AUTHOR: Rob Tillaart
// VERSION: 1.8.3
// PURPOSE: Arduino Library for external I2C EEPROM 24LC256 et al.
//     URL: https://github.com/RobTillaart/I2C_EEPROM.git


#include "Arduino.h"
#include "Wire.h"


#define I2C_EEPROM_VERSION          (F("1.8.3"))

#define I2C_DEVICESIZE_M24M02      264630
#define I2C_DEVICESIZE_M24M01      131072
#define I2C_DEVICESIZE_M24512       65536
#define I2C_DEVICESIZE_M24256       32768 // The only ONE tested
#define I2C_DEVICESIZE_M24128       16384
#define I2C_DEVICESIZE_M24C64        8192
#define I2C_DEVICESIZE_M24C32        4096
#define I2C_DEVICESIZE_M24C16        2048
#define I2C_DEVICESIZE_M24C08        1024
#define I2C_DEVICESIZE_M24C04         512
#define I2C_DEVICESIZE_M24C02         256


//  AT24C32 has a WriteCycle Time of max 20 ms
//  so one need to set I2C_WRITEDELAY to 20000.
//  can also be done on command line.
//  (see private _waitEEReady() function)
#ifndef I2C_WRITEDELAY
#define I2C_WRITEDELAY              5000
#endif

#ifndef UNIT_TEST_FRIEND
#define UNIT_TEST_FRIEND
#endif

//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define SPRN Serial.print
#define SPRNL Serial.println
#define SPRNH Serial.print
#define SPRNLH Serial.println
#else
#define SPRN(MSG)
#define SPRNH(MSG,MSG2)
#define SPRNL(MSG)
#define SPRNLH(MSG,MSG2)
#endif

class I2C_eeprom
{
public:
  /**
    * Initializes the EEPROM with a default deviceSize of I2C_DEVICESIZE_M24256  (32K EEPROM)
    */
  I2C_eeprom(const uint8_t deviceAddress, TwoWire *wire = &Wire);

  /**
    * Initializes the EEPROM for the given device address.
    *
    * It will try to guess page size and address word size based on the size of the device.
    *
    * @param deviceAddress Byte address of the device.
    * @param deviceSize    Max size in bytes of the device (divide your device size in Kbits by 8)
    * @param hasIDPage     ST "D" series devices with additional localable ID page
    * @param wire          Select alternative Wire interface
    */
  I2C_eeprom(const uint8_t deviceAddress, const uint32_t deviceSize, bool hasIDPage, TwoWire *wire = &Wire);

  //  use default I2C pins.
  bool     begin(int8_t writeProtectPin = -1);
  bool     isConnected(bool testIDPage = false);
  uint8_t  getAddress(bool IDPage = false);


  //  writes a byte to memoryAddress
  //  returns I2C status, 0 = OK
  int      writeByte(const uint16_t memoryAddress, const uint8_t value, bool IDPage = false);
  //  writes length bytes from buffer to EEPROM
  //  returns I2C status, 0 = OK
  int      writeBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage = false);
  //  set length bytes in the EEPROM to the same value.
  //  returns I2C status, 0 = OK
  int      setBlock(const uint16_t memoryAddress, const uint8_t value, const uint16_t length, bool IDPage = false);


  //  returns the value stored in memoryAddress
  uint8_t  readByte(const uint16_t memoryAddress, bool IDPage = false);
  //  reads length bytes into buffer
  //  returns bytes read.
  uint16_t readBlock(const uint16_t memoryAddress, uint8_t * buffer, const uint16_t length, bool IDPage = false);
  bool     verifyBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage = false);

  //  updates a byte at memoryAddress, writes only if there is a new value.
  //  return 0 if data is same or written OK, error code otherwise.
  int      updateByte(const uint16_t memoryAddress, const uint8_t value, bool IDPage = false);
  //  updates a block in memory, writes only if there is a new value.
  //  only to be used when you expect to write same buffer multiple times.
  //  If _perByteCompare is TRUE (default), returns bytes written.
  //  Otherwise if length < BUFFERLENGTH, will return length as all will be written
  //  Else if length > BUFFERLENGTH, will return a total of each chunk of BUFFERLENGTH than changed and potential remainder
  uint16_t updateBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage = false);
  void     setPerByteCompare(bool b);
  bool     getPerByteCompare();

  //  same functions as above but with verify
  //  return false if write or verify failed.
  bool     writeByteVerify(const uint16_t memoryAddress, const uint8_t value, bool IDPage = false);
  bool     writeBlockVerify(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage = false);
  bool     setBlockVerify(const uint16_t memoryAddress, const uint8_t value, const uint16_t length, bool IDPage = false);
  bool     updateByteVerify(const uint16_t memoryAddress, const uint8_t value, bool IDPage = false);
  bool     updateBlockVerify(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage = false);


  //  Meta data functions
  uint32_t determineSize(const bool debug = false);
  uint32_t determineSizeNoWrite();
  uint32_t getDeviceSize();
  uint8_t  getPageSize();
  uint8_t  getPageSize(uint32_t deviceSize);
  uint32_t getLastWrite();


  //  for overruling and debugging.
  //  forces a power of 2
  uint32_t setDeviceSize(uint32_t deviceSize);  //  returns set size
  uint8_t  setPageSize(uint8_t pageSize);       //  returns set size


  //  TWR = WriteCycleTime
  //  5 ms is minimum, one can add extra ms here to adjust timing of both read() and write()
  void     setExtraWriteCycleTime(uint8_t ms);
  uint8_t  getExtraWriteCycleTime();


  //  WRITEPROTECT
  //  works only if WP pin is defined in begin().
  //  see readme.md
  inline bool hasWriteProtectPin();
  void     allowWrite();
  void     preventWrite();
  void     setAutoWriteProtect(bool b);
  bool     getAutoWriteProtect();


  // ID Page specific
  // Should be obvious the this will only work if it's an STMicroelectronics "-D" device WITH Identification Page feature.
  uint8_t lockIDPage();
  bool    isIDPageLocked();

private:
  uint8_t  _deviceAddress;
  uint8_t  _idPageDeviceAddress = 0;
  uint32_t _lastWrite  = 0;  //  for waitEEReady
  uint32_t _deviceSize = 0;
  uint8_t  _pageSize   = 0;
  uint8_t  _extraTWR   = 0;  //  milliseconds


  //  24LC32..24LC512 use two bytes for memory address
  //  24LC01..24LC16  use one-byte addresses + part of device address
  bool     _isAddressSizeTwoWords;

  void     _beginTransmission(const uint16_t memoryAddress, bool IDPage = false);

  //  returns I2C status, 0 = OK
  //  TODO incrBuffer is an implementation name, not a functional name.
  int      _pageBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, const bool incrBuffer, bool IDPage = false);
  //  returns I2C status, 0 = OK
  int      _WriteBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint8_t length, bool IDPage = false);
  //  returns bytes read.
  uint8_t  _ReadBlock(const uint16_t memoryAddress, uint8_t * buffer, const uint8_t length, bool IDPage = false);
  //  compare bytes in EEPROM.
  bool     _verifyBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint8_t length, bool IDPage = false);

  //  to optimize the write latency of the EEPROM
  void     _waitEEReady(bool IDPage = false);

  TwoWire * _wire;

  bool     _debug = false;

  int8_t   _writeProtectPin = -1;
  bool     _autoWriteProtect = false;
  bool     _perByteCompare = true;
  bool     _hasIDPage = false;

  UNIT_TEST_FRIEND;
};


//  -- END OF FILE --
