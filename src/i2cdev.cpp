#include "i2cdev.h"

i2cdev::i2cdev(int address){
    const char* i2cbus = "/dev/i2c-1";
    // open i2c bus        
    i2cHandle = open(i2cbus, O_RDWR);        
    if(i2cHandle < 0){
        printf("--- Failed to open I2C bus \n");
    }
    // initiate connection with the i2c device
    if(ioctl(i2cHandle, I2C_SLAVE, address)<0){
        printf("--- I2C access failed \n");
    }
}

i2cdev::~i2cdev(){
    close(i2cHandle);
}

int i2cdev::i2cRead(char* data, int length){
    int byteCount = read(i2cHandle, data, length);
    // error handler
    if (byteCount != length){
        printf("--- Error reading from I2C device\n");
    }        
    return byteCount;
}

int i2cdev::i2cWrite(char *data, int length){
    int byteCount = write(i2cHandle, data, length);
    if (byteCount != length){
        printf("--- Error writing to I2C device\n");
    }  
    return byteCount;
}   