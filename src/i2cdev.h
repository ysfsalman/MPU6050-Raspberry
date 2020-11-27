#ifndef I2CDEV_H
#define I2CDEV_H

// include i2c dependencies for linux
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


class i2cdev{
    // initiate and run communication to an i2c device
    public:
        i2cdev(int address);
        ~i2cdev();
        int i2cRead(char* data, int length); 
        int i2cWrite(char *data, int length);
    private:
        int i2cHandle;
};


#endif