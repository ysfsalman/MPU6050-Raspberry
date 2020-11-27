#include "mpu6050.h"


MPU6050::MPU6050():i2cdev{MPU6050_I2C_ADDRESS}{
    // initialize 
    initialize();
}

void MPU6050::initialize(){
    // set configs
    // set power management
    setConfig(MPU6050_PWR_MGMT_1,0x00);
    // set resolution
    setConfig(MPU6050_GYRO_CONFIG,0x00); // FS_SEL 0 : 250 deg/s
    setConfig(MPU6050_ACCEL_CONFIG,0x00); // AFS_SEL 0 : 2 g/s
    // callibration
    calibrate();
}

void MPU6050::meanSensor(int16_t **meanArray){
    uint16_t sample_size    = 1000;
    uint16_t discard_sample = 100;
    int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
    int32_t buff_accX=0, buff_accY=0, buff_accZ=0;
    int32_t buff_gyroX=0, buff_gyroY=0, buff_gyroZ=0;    
    
    // get mean value
    for(uint16_t i = 0; i < (sample_size + discard_sample); i++){
        // accel
        accX = getAccelX();
        accY = getAccelY();
        accZ = getAccelZ();
        // gyro
        gyroX = getRotationX();
        gyroY = getRotationY();
        gyroZ = getRotationZ();
        if (i > discard_sample && i <= (sample_size + discard_sample)){
            // accel
            buff_accX += accX;
            buff_accY += accY;
            buff_accZ += accZ;
            // gyro
            buff_gyroX += gyroX;
            buff_gyroY += gyroY;
            buff_gyroZ += gyroZ;
        }        
    }
    // accel
    *meanArray[0] = buff_accX/sample_size;
    *meanArray[1]= buff_accY/sample_size;
    *meanArray[2] = buff_accZ/sample_size;
    // gyro
    *meanArray[3] = buff_gyroX/sample_size;
    *meanArray[4]= buff_gyroY/sample_size;
    *meanArray[5] = buff_gyroZ/sample_size;
    //printf("mean value: %i, %i, %i\n", *meanArray[0], *meanArray[1], *meanArray[2]);
    //printf("done-------------------------------------\n");
    
}

void MPU6050::calibrate(){    
    // method are taken from i2cdevlib by Jeff Rowberg
    uint8_t accel_deadzone = 8;     //Accelerometer error allowed
    uint8_t gyro_deadzone = 1;     //Giro error allowed
    
    int16_t mean_accX, mean_accY, mean_accZ;
    int16_t mean_gyroX, mean_gyroY, mean_gyroZ;

    int16_t *meanArr[6] = {&mean_accX, &mean_accY, &mean_accZ,
                            &mean_gyroX, &mean_gyroY, &mean_gyroZ};

    // set initial offset
    setOffset(0,0,0,0,0,0);

    // get mean sensor
    meanSensor(meanArr);

    // set offset
    int16_t ax_offset=-mean_accX/8;
    int16_t ay_offset=-mean_accY/8;
    int16_t az_offset=(16384-mean_accZ)/8;
    int16_t gx_offset=-mean_gyroX/4;
    int16_t gy_offset=-mean_gyroY/4;
    int16_t gz_offset=-mean_gyroZ/4;

    while(1){
        int ready = 0;
        setOffset(ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset);        
        meanSensor(meanArr);
        
        if (abs(mean_accX)<=accel_deadzone) ready++;
        else ax_offset=ax_offset-mean_accX/accel_deadzone;

        if (abs(mean_accY)<=accel_deadzone) ready++;
        else ay_offset=ay_offset-mean_accY/accel_deadzone;

        if (abs(16384-mean_accZ)<=accel_deadzone) ready++;
        else az_offset=az_offset+(16384-mean_accZ)/accel_deadzone;
        
        if (abs(mean_gyroX)<=gyro_deadzone) ready++;
        else gx_offset=gx_offset-mean_gyroX/(gyro_deadzone+1);

        if (abs(mean_gyroY)<=gyro_deadzone) ready++;
        else gy_offset=gy_offset-mean_gyroY/(gyro_deadzone+1);

        if (abs(mean_gyroZ)<=gyro_deadzone) ready++;
        else gz_offset=gz_offset-mean_gyroZ/(gyro_deadzone+1);
        
        if (ready==6) break;
    }
}

void MPU6050::setConfig(char regAddress, char value){
    char config[2];
    config[0] = regAddress;
    config[1] = value;
    i2cWrite(config, 2);
}

void MPU6050::set2BytesConfig(char regAddress, int16_t value){
    char config[3];
    config[0] =  regAddress;
    config[1] = (char)((value>>8) & 0xFF);
    config[2] = (char)(value & 0xFF);
    i2cWrite(config, 3);
}

void MPU6050::setOffset(int16_t aXoff, int16_t aYoff, int16_t aZoff, 
                        int16_t gXoff, int16_t gYoff, int16_t gZoff){
    set2BytesConfig(MPU6050_XA_OFFSET_H, aXoff);
    set2BytesConfig(MPU6050_YA_OFFSET_H, aYoff);
    set2BytesConfig(MPU6050_ZA_OFFSET_H, aZoff);    
    set2BytesConfig(MPU6050_XG_OFFSET_H, gXoff);
    set2BytesConfig(MPU6050_YG_OFFSET_H, gYoff);
    set2BytesConfig(MPU6050_ZG_OFFSET_H, gZoff);
}

int16_t MPU6050::get2Bytes(char regAddress){
    // initalize variables
    char buffer[2];
    int16_t twoBytesVal;
    // request to target register
    i2cWrite(&regAddress, 1);
    // acquire value
    i2cRead(buffer, 2);
    twoBytesVal = (buffer[0]<<8) | buffer[1];
    return twoBytesVal;
}

// get accelerometer and gyroscope value
int16_t MPU6050::getAccelX(){
    return get2Bytes(MPU6050_ACCEL_XOUT_H);
}

int16_t MPU6050::getAccelY(){
    return get2Bytes(MPU6050_ACCEL_YOUT_H);
}

int16_t MPU6050::getAccelZ(){
    return get2Bytes(MPU6050_ACCEL_ZOUT_H);
}

int16_t MPU6050::getRotationX(){
    return get2Bytes(MPU6050_GYRO_XOUT_H);
}

int16_t MPU6050::getRotationY(){
    return get2Bytes(MPU6050_GYRO_YOUT_H);
}

int16_t MPU6050::getRotationZ(){
    return get2Bytes(MPU6050_GYRO_ZOUT_H);
}

// relative angle change
// consider bias later
float MPU6050::getAccRoll(){
    return (float)(atan2(getAccelY(),getAccelZ())*RAD_TO_DEG;    
}

float MPU6050::getAccYaw(){
    return (float)(atan2(getAccelX(),getAccelY())*RAD_TO_DEG;
}

float MPU6050::getAccPitch(){
    return (float)(atan2(getAccelZ(),getAccelX())*RAD_TO_DEG;    
}


void MPU6050::runAngle(){
    // start thread
    std::thread t(updateGyroOrientation);
}

float MPU6050::updateGyroOrientation(){    
    // current_angle  = initial angle;

    // last_time = get time;
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    lastUpdate = std::chrono::system_clock::now();
    while(true){
        // sleep every iteration
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // duration = get time - last_time
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();
        // get gyro_roll_value = gyro speed * converter * duration
        
        // get acc_roll_value = atan2(y,z)
        // current_angle  =  0.98*(gyro_roll_value+current_angle) + 0.02 * acc_roll_value
        lastUpdate = std::chrono::system_clock::now();
    }

    
