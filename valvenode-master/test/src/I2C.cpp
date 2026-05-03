//
//  I2C.cpp
//  coopMgr
//
//  Created by Vincent Moscaritolo on 9/10/21.
//

#include "I2C.hpp"
#include <errno.h>
#include <sys/ioctl.h>                                                  // Serial Port IO Controls
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include "LogMgr.hpp"


// I2C definitions

#define I2C_SLAVE    0x0703
#define I2C_SMBUS    0x0720    /* SMBus-level access */

#define I2C_SMBUS_READ    1
#define I2C_SMBUS_WRITE    0

// SMBus transaction types

#define I2C_SMBUS_QUICK            0
#define I2C_SMBUS_BYTE            1
#define I2C_SMBUS_BYTE_DATA        2
#define I2C_SMBUS_WORD_DATA        3
#define I2C_SMBUS_PROC_CALL        4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7        /* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX    32    /* As specified in SMBus standard */
#define I2C_SMBUS_I2C_BLOCK_MAX    32    /* Not specified but we use same structure */

// Structures used in the ioctl() calls

union i2c_smbus_data
{
  uint8_t  byte ;
  uint16_t word ;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;    // block [0] is used for length + one more for PEC
} ;

struct i2c_smbus_ioctl_data
{
  char read_write ;
  uint8_t command ;
  int size ;
  union i2c_smbus_data *data ;
} ;

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ::ioctl (fd, I2C_SMBUS, &args) ;
}

#ifndef I2C_BUS_DEV_FILE_PATH
#define I2C_BUS_DEV_FILE_PATH "/dev/i2c-1"
#endif /* I2C_SLAVE */

 
I2C::I2C(){
    _isSetup = false;
    _fd = -1;
    _devAddr = 00;
}


I2C::~I2C(){
    stop();
    
}
 
bool I2C::begin(uint8_t    devAddr){
    int error = 0;

    return begin(devAddr, error);
}


bool I2C::begin(uint8_t    devAddr,   int &error){
    static const char *ic2_device = "/dev/i2c-1";
 
    _isSetup = false;
    int fd ;

    if((fd = open( ic2_device, O_RDWR)) <0) {
        LOGT_ERROR("I2C(%02x) begin: failed %s", devAddr, strerror(errno));
         error = errno;
        return false;
    }
    
    if (::ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        LOGT_ERROR("I2C(%02x) I2C_SLAVE: failed %s", devAddr, strerror(errno));
  
        error = errno;
        return false;
    }

    _fd = fd;
    _isSetup = true;
    _devAddr = devAddr;
    
    return _isSetup;
}


void I2C::stop(){
    
    if(_isSetup){
        close(_fd);
        _devAddr = 00;
    }
    
    _isSetup = false;
}

bool I2C::isAvailable(){

    return _isSetup;
}

bool I2C::writeByte(uint8_t regAddr, uint8_t b1){
    
    if(!_isSetup) return false;

    union i2c_smbus_data data = {.byte = b1};
  
    if(i2c_smbus_access (_fd, I2C_SMBUS_WRITE, regAddr, I2C_SMBUS_BYTE_DATA, &data) < 0){
        
        LOGT_ERROR("I2C_SMBUS_WRITE BYTE(%02x,%02X) failed %s",_devAddr, regAddr, strerror(errno));
 
          return false;
    }
    
    return   true;
}


bool I2C::writeBytes(uint8_t regAddr, uint8_t size, uint8_t* block){
    
    if(!_isSetup) return false;
    
    if(size > 0 && this->writeByte(regAddr, block[0])){
        for(uint8_t i = 1; i < size; i++ ){
            if( i2c_smbus_access(_fd, I2C_SMBUS_WRITE, block[i], I2C_SMBUS_BYTE, NULL)  < 0) return false;
        }
        return  true;
    }
    
    return   false;
}

bool I2C::stdWriteBytes(uint8_t size, uint8_t* block ){
    
    if(!_isSetup) return false;
  
    bool status =  write(_fd, block, size) == size;
    
    if(!status){
        LOGT_ERROR("I2C STD WRITE(%02x) %d bytes failed %s", _devAddr,
                    size, strerror(errno));
    }
    return status;
}



bool I2C::writeWord(uint8_t regAddr, uint16_t word){

    if(!_isSetup) return false;

    union i2c_smbus_data data = {.word = word};
 
    if(i2c_smbus_access (_fd, I2C_SMBUS_WRITE, regAddr, I2C_SMBUS_WORD_DATA, &data) < 0){
        
        LOGT_ERROR("I2C_SMBUS_WRITE WORD(%02x,%02X) failed %s",_devAddr,  regAddr, strerror(errno));
  
        return false;
    }
    
    return   true;
}

bool I2C::readByte(uint8_t regAddr,  uint8_t& byte){
    
    if(!_isSetup) return false;

    union i2c_smbus_data data;
    
    if(i2c_smbus_access (_fd, I2C_SMBUS_READ, regAddr, I2C_SMBUS_BYTE_DATA, &data) < 0){
        
        LOGT_ERROR("I2C_SMBUS_READ BYTE(%02x,%02X) failed %s", _devAddr, regAddr, strerror(errno));

        return false;
    }

    byte = data.byte & 0xFF;
    return true;
}

bool I2C::readWord(uint8_t regAddr,  uint16_t& word){

    if(!_isSetup) return false;

    union i2c_smbus_data data;
    
    if(i2c_smbus_access (_fd, I2C_SMBUS_READ, regAddr, I2C_SMBUS_WORD_DATA, &data) < 0){
        
        LOGT_ERROR("I2C_SMBUS_READ WORD(%02x,%02X) failed %s", _devAddr, regAddr, strerror(errno));

        return false;
    }

    word = data.word & 0xFFFF;
    return true;
}
 


bool I2C::readBlock(uint8_t regAddr, uint8_t size, i2c_block_t & block ){

    if(!_isSetup) return false;

    union i2c_smbus_data data;

    memset(data.block, 0, sizeof(data.block));
    data.block[0] = size + 1;

    if(i2c_smbus_access (_fd, I2C_SMBUS_READ, regAddr, I2C_SMBUS_I2C_BLOCK_DATA, &data) < 0){
   
        LOGT_ERROR("I2C_SMBUS_READ BLOCK(%02x,%02X) failed %s", _devAddr, regAddr, strerror(errno));

        return false;
    }

    memcpy(block, data.block, sizeof(block));
    
    return true;
}

 

bool I2C::readBytes(uint8_t regAddr, uint8_t size, uint8_t* block ){

    if(!_isSetup) return false;
 
    uint8_t data;
    
    for(uint8_t offset = 0; offset < size; offset++ )
        if(readByte(regAddr + offset, data))
            *(block+offset) = data;
        else
            return false;
           
           
    return true;
}

bool I2C::stdReadBytes(uint8_t size, uint8_t* block ){
    
    if(!_isSetup) return false;
  
    bool status =  read(_fd, block, size) == size;
    
    if(!status){
        LOGT_ERROR("I2C STD READ(%02x) %d bytes failed %s", _devAddr,
                    size, strerror(errno));
    }
    return status;
}



bool I2C::smbQuick(){
    if(!_isSetup) return false;
  
    union i2c_smbus_data data;
    
    if(i2c_smbus_access (_fd, I2C_SMBUS_WRITE, 0, I2C_SMBUS_QUICK, &data) < 0){
   
        LOGT_ERROR("I2C_SMBUS_QUICK(%02x) failed %s", _devAddr, strerror(errno));

        return false;
    }

    return true;
}



// MARK: -   I2C tool

bool I2C::getI2CAddressMap(std::vector<uint8_t> &addrs) {
 
    uint8_t first = 0x08;
    uint8_t last  =  0x77;
    
    uint8_t address;
     
    static const char *ic2_device = "/dev/i2c-1";
    int        fd;
    
     addrs.clear();
    fd = open( ic2_device, O_RDWR);
    
    if(fd == -1){
        LOGT_ERROR("Could Not open %s - %d %s\n", ic2_device, errno, strerror(errno));
        return false;
    }

    // table body
    // addresses 0x00 through 0x77
    for (address = first; address <= last; address++) {
        
        uint8_t b;
 
        int error = ioctl(fd, I2C_SLAVE, address);
        ssize_t count = read(fd,&b, 1);

        if (error == 0 && count == 1) {
            // device found
            addrs.push_back(address);
        }
    }
   
    close (fd);
    return true;
}
