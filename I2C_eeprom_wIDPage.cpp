//
//    FILE: I2C_eeprom.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 1.8.3
// PURPOSE: Arduino Library for external I2C EEPROM M24256 et al.
//     URL: https://github.com/RobTillaart/I2C_EEPROM.git


#include "I2C_eeprom_wIDPage.h"

#define EN_AUTO_WRITE_PROTECT           1  // IF WP pin is supplied then _autoWriteProtect is enabled by default
#define ALLOW_IDPAGE_LOCK               0
#define I_ACK_IDPAGE_CANT_BE_UNLOCKED   0
#define PER_BYTE_COMPARE                1

//  Not used directly
#define I2C_PAGESIZE_M24512           128
#define I2C_PAGESIZE_M24256            64
#define I2C_PAGESIZE_M24128            64
#define I2C_PAGESIZE_M24C64            32
#define I2C_PAGESIZE_M24C32            32
#define I2C_PAGESIZE_M24C16            16
#define I2C_PAGESIZE_M24C08            16
#define I2C_PAGESIZE_M24C04            16
#define I2C_PAGESIZE_M24C02             8


//  I2C buffer needs max 2 bytes for EEPROM address
//  1 byte for EEPROM register address is available in transmit buffer
#if defined(ESP32) || defined(ESP8266) || defined(PICO_RP2040)
#define I2C_BUFFERSIZE           128
#else
#define I2C_BUFFERSIZE           30   //  AVR, STM
#endif


////////////////////////////////////////////////////////////////////
//
//  PUBLIC FUNCTIONS
//
I2C_eeprom::I2C_eeprom(const uint8_t deviceAddress, TwoWire * wire) :
            I2C_eeprom(deviceAddress, I2C_PAGESIZE_M24256, wire)
{
}


I2C_eeprom::I2C_eeprom(const uint8_t deviceAddress, const uint32_t deviceSize, bool hasIDPage, TwoWire * wire)
{
  _deviceAddress = deviceAddress;
  _hasIDPage = hasIDPage;
  if(hasIDPage) _idPageDeviceAddress =_deviceAddress+8;
  _deviceSize = setDeviceSize(deviceSize);
  _pageSize = getPageSize(_deviceSize);
  _wire = wire;

  //  Chips 16 Kbit (2048 Bytes) or smaller only have one-word addresses.
  this->_isAddressSizeTwoWords = deviceSize > I2C_DEVICESIZE_M24C16;
}


bool I2C_eeprom::begin(int8_t writeProtectPin)
{
  //  if (_wire == 0) SPRNL("zero");  //  test #48
  _lastWrite = 0;
  _writeProtectPin = writeProtectPin;
  if (_writeProtectPin >= 0)
  {
    _autoWriteProtect = EN_AUTO_WRITE_PROTECT;
    pinMode(_writeProtectPin, OUTPUT);
    preventWrite();
  }
  return isConnected();
}


bool I2C_eeprom::isConnected(bool testIDPage)
{
  _wire->beginTransmission(testIDPage ? _idPageDeviceAddress : _deviceAddress);
  return (_wire->endTransmission() == 0);
}


uint8_t I2C_eeprom::getAddress(bool IDPage)
{
  return IDPage ? _idPageDeviceAddress : _deviceAddress;
}

uint8_t I2C_eeprom::lockIDPage() {
  if(ALLOW_IDPAGE_LOCK && I_ACK_IDPAGE_CANT_BE_UNLOCKED) {
    uint16_t memoryAddress = 0x400;
    const uint8_t data = {0b00000010};
    int rv = _WriteBlock(memoryAddress, &data, 1, true);
    return rv;
  }
  return 13; // Unlucky dawg...
}

bool I2C_eeprom::isIDPageLocked() {
  if (_autoWriteProtect)
  {
    digitalWrite(_writeProtectPin, LOW);
  }

  _wire->beginTransmission(_idPageDeviceAddress);
  _wire->write(0x01);
  int rv = _wire->endTransmission();
  Serial.print("Is Locked Test wire return code: ");
  Serial.println(rv);
  _wire->beginTransmission(_idPageDeviceAddress);
  _wire->endTransmission();
  
  if (_autoWriteProtect)
  {
    digitalWrite(_writeProtectPin, HIGH);
  }
  
  return rv;
}


/////////////////////////////////////////////////////////////
//
//  WRITE SECTION
//

//  returns I2C status, 0 = OK
int I2C_eeprom::writeByte(const uint16_t memoryAddress, const uint8_t data, bool IDPage)
{
  int rv = _WriteBlock(memoryAddress, &data, 1, IDPage);
  return rv;
}


//  returns I2C status, 0 = OK
int I2C_eeprom::setBlock(const uint16_t memoryAddress, const uint8_t data, const uint16_t length, bool IDPage)
{
  uint8_t buffer[I2C_BUFFERSIZE];
  for (uint8_t i = 0; i < I2C_BUFFERSIZE; i++)
  {
    buffer[i] = data;
  }
  int rv = _pageBlock(memoryAddress, buffer, length, false, IDPage);
  return rv;
}


//  returns I2C status, 0 = OK
int I2C_eeprom::writeBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage)
{
  int rv = _pageBlock(memoryAddress, buffer, length, true, IDPage);
  return rv;
}


/////////////////////////////////////////////////////////////
//
//  READ SECTION
//

//  returns the value stored in memoryAddress
uint8_t I2C_eeprom::readByte(const uint16_t memoryAddress, bool IDPage)
{
  uint8_t rdata;
  _ReadBlock(memoryAddress, &rdata, 1, IDPage);
  int rv = _wire->endTransmission();
  if (rv != 0)
  {
//    if (_debug)
//    {
//      SPRN("mem addr r: ");
//      SPRNH(memoryAddress, HEX);
//      SPRN("\t");
//      SPRNL(rv);
//    }
    return 0;  //  error
  }
  return rdata;
}


//  returns bytes read.
uint16_t I2C_eeprom::readBlock(const uint16_t memoryAddress, uint8_t * buffer, const uint16_t length, bool IDPage)
{
  uint16_t addr = memoryAddress;
  uint16_t len = length;
  uint16_t rv = 0;
  while (len > 0)
  {
    uint8_t cnt = I2C_BUFFERSIZE;
    if (cnt > len) cnt = len;
    rv     += _ReadBlock(addr, buffer, cnt, IDPage);
    addr   += cnt;
    buffer += cnt;
    len    -= cnt;
  }
  return rv;
}


//  returns true or false.
bool I2C_eeprom::verifyBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage)
{
  uint16_t addr = memoryAddress;
  uint16_t len = length;
  while (len > 0)
  {
    uint8_t cnt = I2C_BUFFERSIZE;
    if (cnt > len) cnt = len;
    if (_verifyBlock(addr, buffer, cnt, IDPage) == false)
    {
      return false;
    }
    addr   += cnt;
    buffer += cnt;
    len    -= cnt;
  }
  return true;
}


/////////////////////////////////////////////////////////////
//
//  UPDATE SECTION
//

//  returns 0 == OK
int I2C_eeprom::updateByte(const uint16_t memoryAddress, const uint8_t data, bool IDPage)
{
  if (data == readByte(memoryAddress, IDPage)) return 0;
  return writeByte(memoryAddress, data, IDPage);
}


//  returns bytes written.
uint16_t I2C_eeprom::updateBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage)
{
  uint16_t addr = memoryAddress;
  uint16_t len = length;
  uint16_t rv = 0;
  
  if (_perByteCompare) {
    Serial.println("Performing BYTE SIZE updates");
    uint16_t writeCnt = 0;

    // Read the original data block from the EEPROM.
    uint8_t origBuf[length];
    _ReadBlock(addr, origBuf, length, IDPage);

    // Temporary buffer to hold changes.
    uint8_t writeBuf[length];
    memset(writeBuf, 0, length); // Clear the write buffer.
    
    uint16_t diffCount = 0;
    uint16_t startDiffAddr = 0;

    // Iterate over each byte to find differences.
    for (uint16_t i = 0; i < len; ++i) {
      if (buffer[i] != origBuf[i]) {
        // Start buffering when the first difference is found.
        if (diffCount == 0) {
          startDiffAddr = addr + i; // Save the starting address of the difference.
        }
        writeBuf[diffCount++] = buffer[i]; // Add differing byte to buffer.
      } else {
        // If there was a difference and now it stops, write the buffered changes.
        if (diffCount > 0) {
          rv += diffCount;
          _pageBlock(startDiffAddr, writeBuf, diffCount, true, IDPage);
          diffCount = 0; // Reset difference count after writing.
          writeCnt++;
        }
      }
    }

    // Check if there are any remaining differences to write after the loop.
    if (diffCount > 0) {
      rv += diffCount;
      _pageBlock(startDiffAddr, writeBuf, diffCount, true, IDPage);
      writeCnt++;
    }
    Serial.print("EEPROM Write cycles: ");
    Serial.println(writeCnt);

    return rv;
  } 
  else 
  {
    Serial.println("Performing BUFFERSIZE updates");
    while (len > 0)
    {
      uint8_t buf[I2C_BUFFERSIZE];
      uint8_t cnt = I2C_BUFFERSIZE;

      if (cnt > len) cnt = len;
      _ReadBlock(addr, buf, cnt);
      if (memcmp(buffer, buf, cnt) != 0)
      {
        rv   += cnt; // update rv to actual number of bytes written due to failed compare
        _pageBlock(addr, buffer, cnt, true, IDPage);
      }
      addr   += cnt;
      buffer += cnt;
      len    -= cnt;
    }
    return rv;    
  }
}


/////////////////////////////////////////////////////////////
//
//  VERIFY SECTION
//

//  return false if write or verify failed.
bool I2C_eeprom::writeByteVerify(const uint16_t memoryAddress, const uint8_t value, bool IDPage)
{
  if (writeByte(memoryAddress, value, IDPage) != 0 ) return false;
  uint8_t data = readByte(memoryAddress, IDPage);
  return (data == value);
}


//  return false if write or verify failed.
bool I2C_eeprom::writeBlockVerify(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage)
{
  if (writeBlock(memoryAddress, buffer, length, IDPage) != 0) return false;
  return verifyBlock(memoryAddress, buffer, length, IDPage);
}


//  return false if write or verify failed.
bool I2C_eeprom::setBlockVerify(const uint16_t memoryAddress, const uint8_t value, const uint16_t length, bool IDPage)
{
  if (setBlock(memoryAddress, value, length, IDPage) != 0) return false;
  uint8_t * data = (uint8_t *) malloc(length);
  if (data == NULL) return false;
  if (readBlock(memoryAddress, data, length) != length) return false;
  for (uint16_t i = 0; i < length; i++)
  {
    if (data[i] != value)
    {
      free(data);
      return false;
    }
  }
  free(data);
  return true;
}


//  return false if write or verify failed.
bool I2C_eeprom::updateByteVerify(const uint16_t memoryAddress, const uint8_t value, bool IDPage)
{
  if (updateByte(memoryAddress, value, IDPage) != 0 ) return false;
  uint8_t data = readByte(memoryAddress, IDPage);
  return (data == value);
}


//  return false if write or verify failed.
bool I2C_eeprom::updateBlockVerify(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, bool IDPage)
{
  if (updateBlock(memoryAddress, buffer, length, IDPage) != length) return false;
  return verifyBlock(memoryAddress, buffer, length, IDPage);
}


/////////////////////////////////////////////////////////////
//
//  METADATA SECTION
//

//  returns size in bytes
//  returns 0 if not connected
//
//   tested for
//   2 byte address
//   M24512      64 KB    YES
//   M24256      32 KB    YES
//   M24128      16 KB    YES
//   M24C64       8 KB    YES
//   M24C32       4 KB    YES* - no hardware test, address scheme identical to M24C64.
//
//   1 byte address (uses part of deviceAddress byte)
//   M24C16       2 KB    YES
//   M24C08       1 KB    YES
//   M24C04      512 B    YES
//   M24C02      256 B    YES
uint32_t I2C_eeprom::determineSize(const bool debug)
{
  // try to read a byte to see if connected
  if (! isConnected()) return 0;

  uint8_t patAA = 0xAA;
  uint8_t pat55 = 0x55;

  for (uint32_t size = 128; size <= 65536; size *= 2)
  {
    bool folded = false;

    //  store old values
    bool addressSize = _isAddressSizeTwoWords;
    _isAddressSizeTwoWords = size > I2C_DEVICESIZE_M24C16;  // 2048
    uint8_t buf = readByte(size);

    //  test folding
    uint8_t cnt = 0;
    writeByte(size, pat55);
    if (readByte(0) == pat55) cnt++;
    writeByte(size, patAA);
    if (readByte(0) == patAA) cnt++;
    folded = (cnt == 2);
    if (debug)
    {
      SPRNH(size, HEX);
      SPRN('\t');
      SPRNLH(readByte(size), HEX);
    }

    //  restore old values
    writeByte(size, buf);
    _isAddressSizeTwoWords = addressSize;

    if (folded) return size;
  }
  return 0;
}

//  new 1.8.1 #61
//  updated 1.8.2 #63
//
// Returns:
//  0 if device size cannot be determined or device is not online
//  1 if device has default bytes in first dataFirstBytes bytes [0-BUFSIZE]
//      Write some dataFirstBytes to the first bytes and retry or use the determineSize method
//  2 if device has all the same bytes in first dataFirstBytes bytes [0-BUFSIZE]
//      Write some random dataFirstBytes to the first bytes and retry or use the determineSize method
//  >= 128 Device size in bytes
uint32_t I2C_eeprom::determineSizeNoWrite()
{
  #define BUFSIZE (32)
  //  try to read a byte to see if connected
  if (!isConnected()) return 0;

  bool addressSize = _isAddressSizeTwoWords;
  _isAddressSizeTwoWords = true; //Otherwise reading large EEPROMS fails
  bool isModifiedFirstSector = false;
  bool dataIsDifferent = false;

  byte dataFirstBytes[BUFSIZE];
  byte dataMatch[BUFSIZE];
  readBlock(0, dataFirstBytes, BUFSIZE);

  for (uint8_t pos = 0; pos < BUFSIZE; pos++)
  {
      if (dataIsDifferent || pos == 0)
      {
          //ignore futher comparison if dataFirstBytes is not the same in buffer
          //Ignore first byte
      }
      else if (dataFirstBytes[pos - 1] != dataFirstBytes[pos])
      {
          dataIsDifferent = true;
      }

      if (dataFirstBytes[pos] != 0xFF && dataFirstBytes[pos] != 0x00)
      {
          //Default dataFirstBytes value is 0xFF or 0x00
          isModifiedFirstSector = true;
      }

      if (dataIsDifferent && isModifiedFirstSector)
          break;
  }

  if (!isModifiedFirstSector)
  {
      //Cannot determine diff, at least one of the first bytes within 0 - len [BUFSIZE] needs to be changed.
      //to something other than 0x00 and 0xFF
      _isAddressSizeTwoWords = addressSize;
      return 1;
  }
  if (!dataIsDifferent)
  {
      //Data in first bytes within 0 - len [BUFSIZE] are all the same.
      _isAddressSizeTwoWords = addressSize;
      return 2;
  }

  //Read from largest to smallest size
  for (uint32_t size = 32768; size >= 64; size /= 2)
  {
    _isAddressSizeTwoWords = (size >= I2C_DEVICESIZE_M24C16);  //  == 2048

    // Try to read last byte of the block, should return length of 0 when fails for single byte devices
    // Will return the same dataFirstBytes as initialy read on other devices as the datapointer could not be moved to the requested position
    delay(2);
    uint16_t bSize = readBlock(size, dataMatch, BUFSIZE);

    if (bSize == BUFSIZE && memcmp(dataFirstBytes, dataMatch, BUFSIZE) != 0)
    {
        //Read is perfomed just over size (size + BUFSIZE), this will only work for devices with mem > size; therefore return size * 2
        _isAddressSizeTwoWords = addressSize;
        return size * 2;
    }
  }
  _isAddressSizeTwoWords = addressSize;
  return 0;
}


uint32_t I2C_eeprom::getDeviceSize()
{
  return _deviceSize;
}


uint8_t I2C_eeprom::getPageSize()
{
  return _pageSize;
}


uint8_t I2C_eeprom::getPageSize(uint32_t deviceSize)
{
    //  determine page size from device size
    //  based on M24XX data sheets.
    //  Identification Page size is Equal to base memory page size
    if (deviceSize <= I2C_DEVICESIZE_M24C16) return 16;
    if (deviceSize <= I2C_DEVICESIZE_M24C64) return 32;
    if (deviceSize <= I2C_DEVICESIZE_M24256) return 64;
    //  I2C_DEVICESIZE_M24512 or larger
    return 128;
}


uint32_t I2C_eeprom::getLastWrite()
{
  return _lastWrite;
}


uint32_t I2C_eeprom::setDeviceSize(uint32_t deviceSize)
{
  uint32_t size = 128;
  //  force power of 2.
  while ((size <= 65536) && ( size <= deviceSize))
  {
    _deviceSize = size;
    size *= 2;
  }
  //  Chips 16 Kbit (2048 Bytes) or smaller only have one-word addresses.
  this->_isAddressSizeTwoWords = _deviceSize > I2C_DEVICESIZE_M24C16;
  return _deviceSize;
}


uint8_t I2C_eeprom::setPageSize(uint8_t pageSize)
{
  // force power of 2.
  if (pageSize >= 128) {
      _pageSize = 128;
  }
  else if (pageSize >= 64) {
      _pageSize = 64;
  }
  else if (pageSize >= 32) {
      _pageSize = 32;
  }
  else {
      _pageSize = 16;
  }
  return _pageSize;
}


void I2C_eeprom::setExtraWriteCycleTime(uint8_t ms)
{
  _extraTWR = ms;
}


uint8_t I2C_eeprom::getExtraWriteCycleTime()
{
  return _extraTWR;
}


//
//  WRITEPROTECT
//
bool I2C_eeprom::hasWriteProtectPin()
{
  return (_writeProtectPin >= 0);
}


void I2C_eeprom::allowWrite()
{
  if (hasWriteProtectPin())
  {
    digitalWrite(_writeProtectPin, LOW);
  }
}


void I2C_eeprom::preventWrite()
{
  if (hasWriteProtectPin())
  {
    digitalWrite(_writeProtectPin, HIGH);
  }
}


void I2C_eeprom::setAutoWriteProtect(bool b)
{
  if (hasWriteProtectPin())
  {
    _autoWriteProtect = b;
  }
}


bool I2C_eeprom::getAutoWriteProtect()
{
  return _autoWriteProtect;
}


void I2C_eeprom::setPerByteCompare(bool b)
{
  _perByteCompare = b;
}  

////////////////////////////////////////////////////////////////////
//
//  PRIVATE
//

//  _pageBlock aligns buffer to page boundaries for writing.
//  and to I2C buffer size
//  returns 0 = OK otherwise error
int I2C_eeprom::_pageBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint16_t length, const bool incrBuffer, bool IDPage)
{
  uint16_t addr = memoryAddress;
  uint16_t len = length;

  // Check if IDPage is true and length from the specified address is larger than the single ID page boundary
  if (IDPage && (addr + len > this->_pageSize)) {
    return 11; // Trying to crank it up to 11, and it will wrap when crossing page boundary
  }

  // Check if the entire write operation stays within the EEPROM's memory limits
  if (addr + len > this->_deviceSize) {
    return 12; // Error code 12 for attempting to write beyond the EEPROM's memory limits
  }
  
  while (len > 0)
  {
    uint8_t bytesUntilPageBoundary = this->_pageSize - addr % this->_pageSize;

    uint8_t cnt = I2C_BUFFERSIZE;
    if (cnt > len) cnt = len;
    if (cnt > bytesUntilPageBoundary) cnt = bytesUntilPageBoundary;

    int rv = _WriteBlock(addr, buffer, cnt, IDPage);
    if (rv != 0) return rv;

    addr += cnt;
    if (incrBuffer) buffer += cnt;
    len -= cnt;
  }
  return 0;
}


//  supports one and two bytes addresses
void I2C_eeprom::_beginTransmission(const uint16_t memoryAddress, bool IDPage)
{
  if (this->_isAddressSizeTwoWords)
  {
    _wire->beginTransmission(IDPage ? _idPageDeviceAddress : _deviceAddress);
    //  Address High Byte
    _wire->write((memoryAddress >> 8));
  }
  else
  {
    uint8_t addr = (IDPage ? _idPageDeviceAddress : _deviceAddress) | ((memoryAddress >> 8) & 0x07);
    _wire->beginTransmission(addr);
  }

  //  Address Low Byte
  //  (or single byte for chips 16K or smaller that have one-word addresses)
  _wire->write((memoryAddress & 0xFF));
}


//  pre: length <= this->_pageSize  && length <= I2C_BUFFERSIZE;
//  returns 0 = OK otherwise error
int I2C_eeprom::_WriteBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint8_t length, bool IDPage)
{
  _waitEEReady();
  if (_autoWriteProtect)
  {
    digitalWrite(_writeProtectPin, LOW);
  }

  this->_beginTransmission(memoryAddress, IDPage);
  _wire->write(buffer, length);
  int rv = _wire->endTransmission();

  if (_autoWriteProtect)
  {
    digitalWrite(_writeProtectPin, HIGH);
  }

  _lastWrite = micros();

  yield();     // For OS scheduling

//  if (rv != 0)
//  {
//    if (_debug)
//    {
//      SPRN("mem addr w: ");
//      SPRNH(memoryAddress, HEX);
//      SPRN("\t");
//      SPRNL(rv);
//    }
//    return -(abs(rv));  // error
//  }
  return rv;
}


//  pre: buffer is large enough to hold length bytes
//  returns bytes read
uint8_t I2C_eeprom::_ReadBlock(const uint16_t memoryAddress, uint8_t * buffer, const uint8_t length, bool IDPage)
{
  _waitEEReady();

  this->_beginTransmission(memoryAddress, IDPage);
  int rv = _wire->endTransmission(false);
  if (rv != 0)
  {
//    if (_debug)
//    {
//      SPRN("mem addr r: ");
//      SPRNH(memoryAddress, HEX);
//      SPRN("\t");
//      SPRNL(rv);
//    }
    return 0;  //  error
  }

  //  readBytes will always be equal or smaller to length
  uint8_t readBytes = 0;
  if (this->_isAddressSizeTwoWords)
  {
    readBytes = _wire->requestFrom((IDPage ? _idPageDeviceAddress : _deviceAddress), length);
  }
  else
  {
    uint8_t addr = (IDPage ? _idPageDeviceAddress : _deviceAddress) | ((memoryAddress >> 8) & 0x07);
    readBytes = _wire->requestFrom(addr, length);
  }
  yield();     //  For OS scheduling
  uint8_t cnt = 0;
  while (cnt < readBytes)
  {
    buffer[cnt++] = _wire->read();
  }
  return readBytes;
}


//  compares content of EEPROM with buffer.
//  returns true if equal.
bool I2C_eeprom::_verifyBlock(const uint16_t memoryAddress, const uint8_t * buffer, const uint8_t length, bool IDPage)
{
  _waitEEReady();

  this->_beginTransmission(memoryAddress, IDPage);
  int rv = _wire->endTransmission();
  if (rv != 0)
  {
//    if (_debug)
//    {
//      SPRN("mem addr r: ");
//      SPRNH(memoryAddress, HEX);
//      SPRN("\t");
//      SPRNL(rv);
//    }
    return false;  //  error
  }

  //  readBytes will always be equal or smaller to length
  uint8_t readBytes = 0;
  if (this->_isAddressSizeTwoWords)
  {
    readBytes = _wire->requestFrom((IDPage ? _idPageDeviceAddress : _deviceAddress), length);
  }
  else
  {
    uint8_t addr = (IDPage ? _idPageDeviceAddress : _deviceAddress) | ((memoryAddress >> 8) & 0x07);
    readBytes = _wire->requestFrom(addr, length);
  }
  yield();     //  For OS scheduling
  uint8_t cnt = 0;
  while (cnt < readBytes)
  {
    if (buffer[cnt++] != _wire->read())
    {
      return false;
    }
  }
  return true;
}


void I2C_eeprom::_waitEEReady()
{
  //  Wait until EEPROM gives ACK again.
  //  this is a bit faster than the hardcoded 5 milliSeconds
  //  TWR = WriteCycleTime
  uint32_t waitTime = I2C_WRITEDELAY + _extraTWR * 1000UL;
  while ((micros() - _lastWrite) <= waitTime)
  {
    if (isConnected()) return;
    //  TODO remove pre 1.7.4 code
    // _wire->beginTransmission(_deviceAddress);
    // int x = _wire->endTransmission();
    // if (x == 0) return;
    yield();     //  For OS scheduling
  }
  return;
}


//  -- END OF FILE --

