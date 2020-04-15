
/*
 * Authors:
tcmichals
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern "C"
{
    #include "platform.h"
}

#if defined(USE_I2C) && !defined(SOFT_I2C)

extern "C"
{
#include "build/debug.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

}

#include <vector>
#include <thread>
#include <mutex>
#include <memory.h>
#include <boost/asio.hpp>
#include "target_linux.h"

#include "linux/limits.h"
#include <sys/types.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

//bus_i2c


/* bus_i2c_hal.c */
i2cDevice_t i2cDevice[I2CDEV_COUNT];

uint16_t i2cErrorCount =0 ;

#define CLOCKSPEED 800000    // i2c clockspeed 400kHz default (conform specs), 800kHz  and  1200kHz (Betaflight default)


typedef struct
{
 int fd;
 int errorCount;
 const char path[PATH_MAX];
     
}linuxI2CDevice_t;

#define STRINGIFY(s) XSTRINGIFY(s)
#define XSTRINGIFY(s) #s
  
#pragma message "Device count " STRINGIFY(I2CDEV_COUNT)
const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {};
linuxI2CDevice_t i2cLinuxHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        -1,
        0,
        "/dev/i2c-0",
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .fd = -1,
        .errorCount = 0,
        .path = "/dev/i2c-1",
    },
#endif
#ifdef USE_I2C_DEVICE_3
    { 
        .fd = -1,
        .errorCount = 0,
        .path = "/dev/i2c-2",
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .fd = -1,
        .errorCount = 0,
        .path = "/dev/i2c-3",
    },
#endif
};

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    (void)device;
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return false;
}


// Blocking write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

// Non-blocking write
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    int8_t count = 0;
    uint8_t buf[128];
    linuxI2CDevice_t *pI2CHw = &i2cLinuxHardware[device];
        
    if (len_ > 127) {
        fprintf(stderr, "Byte write count (%d) > 127\n", len_);
        return i2cHandleHardwareFailure(device);
    }

    if (pI2CHw->fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return i2cHandleHardwareFailure(device);
    }
    
    if (ioctl(pI2CHw->fd, I2C_SLAVE, addr_) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        return i2cHandleHardwareFailure(device);
    }
    
    buf[0] = reg_;
    memcpy(buf+1,data,len_);
    count = write(pI2CHw->fd, buf, len_+1);
    
    if (count < 0) 
    {
        fprintf(stderr, "%s Failed to write device(%d): %s\n", __func__,len_, strerror(errno));
        return i2cHandleHardwareFailure(device);
    } 
    else if (count != len_+1) 
    {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", len_+1, count);
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Blocking read
bool i2cRead(I2CDevice device, 
            uint8_t addr_, 
            uint8_t reg_, 
            uint8_t len, 
            uint8_t* buf)
{
    
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }
   int8_t count = 0;
    linuxI2CDevice_t *pI2CHw = &i2cLinuxHardware[device];
       
    if (pI2CHw->fd < 0)
        return false;
    
    if (ioctl(pI2CHw->fd, I2C_SLAVE, addr_) < 0) 
    {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        return(false);
    }
    
    if (write(pI2CHw->fd, &reg_, 1) != 1) 
    {
        fprintf(stderr, "%s Failed to write reg: %s\n", __func__, strerror(errno));
        return i2cHandleHardwareFailure(device);
    }
    count = read(pI2CHw->fd, buf, len);

    if (count < 0) 
    {
        fprintf(stderr, "Failed to read device(%d): %s\n", count, strerror(errno));
        return i2cHandleHardwareFailure(device);
    } 
    else if (count != len) 
    {
        fprintf(stderr, "Short read  from device, expected %d, got %d\n", len, count);
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

// Non-blocking read
bool i2cReadBuffer( I2CDevice device, 
                    uint8_t addr_, 
                    uint8_t reg_, 
                    uint8_t len, 
                    uint8_t* buf)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    int8_t count = 0;
    linuxI2CDevice_t *pI2CHw = &i2cLinuxHardware[device];
       
    if (pI2CHw->fd < 0)
    {
         return i2cHandleHardwareFailure(device);
    }
    
    if (ioctl(pI2CHw->fd, I2C_SLAVE, addr_) < 0) 
    {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        return i2cHandleHardwareFailure(device);
    }
    
    if (write(pI2CHw->fd, &reg_, 1) != 1) 
    {
        fprintf(stderr, "%s Failed to write reg: %s\n", __func__, strerror(errno));
         return i2cHandleHardwareFailure(device);
    }
    count = read(pI2CHw->fd, buf, len);

    if (count < 0) 
    {
        fprintf(stderr, "Failed to read device(%d): %s\n", count, strerror(errno));
        return i2cHandleHardwareFailure(device);
    } 
    else if (count != len) 
    {
        fprintf(stderr, "Short read  from device, expected %d, got %d\n", len, count);
        return i2cHandleHardwareFailure(device);
    }

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    UNUSED(device);
    UNUSED(error);
    return false;
}

void i2cInit(I2CDevice device)
{
    std::cout << __PRETTY_FUNCTION__ << " " << __LINE__ << std::endl;
    if (device == I2CINVALID) {
        return;
    }

    linuxI2CDevice_t *pI2CHw = &i2cLinuxHardware[device];

    pI2CHw->fd = open(pI2CHw->path, O_RDWR);
    
    if( pI2CHw->fd  < 0)
        return;
   
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}





#endif