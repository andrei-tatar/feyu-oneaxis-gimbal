/*
MPU.cpp
BananaPi Bit Team
juwan@banana-pi.com (Juwan·C)
wanghao@banana-pi.com (Hulk Wang)

We modified it based on https://github.com/bolderflight/MPU9250
The main purpose is to adapt the MPU9250 driver library of BPI-BIT (espressif32). 

Copyright (c) 2017 BananaPi Bit Team

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "mpu.h"
#include "serial.h"
#include <math.h>
#include <string.h>

MPU6500::MPU6500(SpiDriver &bus) : _spi(bus)
{
}

/* starts communication with the MPU */
int MPU6500::begin()
{
    while (true)
    {
        int res = whoAmI();
        Serial.printf("whoAmI() %d\r\n", res);
        // _i2c->reset();
        if (res == 112)
        {
            break;
        }
    }
    // select clock source to gyro
    if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
    {
        return -1;
    }
    // enable I2C master mode
    if (writeRegister(USER_CTRL, I2C_MST_EN) < 0)
    {
        return -2;
    }
    // set the I2C bus speed to 400 kHz
    if (writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0)
    {
        return -3;
    }
    // reset the MPU
    writeRegister(PWR_MGMNT_1, PWR_RESET);

    // wait for MPU to come back up
    delay(1);
    // select clock source to gyro
    if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
    {
        return -4;
    }
    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    if (whoAmI() != 112)
    {
        return -5;
    }
    // enable accelerometer and gyro
    if (writeRegister(PWR_MGMNT_2, SEN_ENABLE) < 0)
    {
        return -6;
    }
    // setting accel range to 16G as default
    if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0)
    {
        return -7;
    }
    _accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
    _accelRange = ACCEL_RANGE_16G;
    // setting the gyro range to 2000DPS as default
    if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0)
    {
        return -8;
    }
    _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
    _gyroRange = GYRO_RANGE_2000DPS;
    // setting bandwidth to 184Hz as default
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
    {
        return -9;
    }
    if (writeRegister(CONFIG, GYRO_DLPF_184) < 0)
    { // setting gyro bandwidth to 184Hz
        return -10;
    }
    _bandwidth = DLPF_BANDWIDTH_184HZ;
    // setting the sample rate divider to 0 as default
    if (writeRegister(SMPDIV, 0x00) < 0)
    {
        return -11;
    }
    _srd = 0;

    // select clock source to gyro
    if (writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) < 0)
    {
        return -19;
    }

    // estimate gyro bias
    if (calibrateGyro() < 0)
    {
        return -20;
    }
    // successful init, return 1
    return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU6500::setAccelRange(AccelRange range)
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch (range)
    {
    case ACCEL_RANGE_2G:
    {
        // setting the accel range to 2G
        if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G) < 0)
        {
            return -1;
        }
        _accelScale = G * 2.0f / 32767.5f; // setting the accel scale to 2G
        break;
    }
    case ACCEL_RANGE_4G:
    {
        // setting the accel range to 4G
        if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G) < 0)
        {
            return -1;
        }
        _accelScale = G * 4.0f / 32767.5f; // setting the accel scale to 4G
        break;
    }
    case ACCEL_RANGE_8G:
    {
        // setting the accel range to 8G
        if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G) < 0)
        {
            return -1;
        }
        _accelScale = G * 8.0f / 32767.5f; // setting the accel scale to 8G
        break;
    }
    case ACCEL_RANGE_16G:
    {
        // setting the accel range to 16G
        if (writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) < 0)
        {
            return -1;
        }
        _accelScale = G * 16.0f / 32767.5f; // setting the accel scale to 16G
        break;
    }
    }
    _accelRange = range;
    return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU6500::setGyroRange(GyroRange range)
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch (range)
    {
    case GYRO_RANGE_250DPS:
    {
        // setting the gyro range to 250DPS
        if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS) < 0)
        {
            return -1;
        }
        _gyroScale = 250.0f / 32767.5f * _d2r; // setting the gyro scale to 250DPS
        break;
    }
    case GYRO_RANGE_500DPS:
    {
        // setting the gyro range to 500DPS
        if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS) < 0)
        {
            return -1;
        }
        _gyroScale = 500.0f / 32767.5f * _d2r; // setting the gyro scale to 500DPS
        break;
    }
    case GYRO_RANGE_1000DPS:
    {
        // setting the gyro range to 1000DPS
        if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS) < 0)
        {
            return -1;
        }
        _gyroScale = 1000.0f / 32767.5f * _d2r; // setting the gyro scale to 1000DPS
        break;
    }
    case GYRO_RANGE_2000DPS:
    {
        // setting the gyro range to 2000DPS
        if (writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) < 0)
        {
            return -1;
        }
        _gyroScale = 2000.0f / 32767.5f * _d2r; // setting the gyro scale to 2000DPS
        break;
    }
    }
    _gyroRange = range;
    return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU6500::setDlpfBandwidth(DlpfBandwidth bandwidth)
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch (bandwidth)
    {
    case DLPF_BANDWIDTH_184HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
        { // setting accel bandwidth to 184Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_184) < 0)
        { // setting gyro bandwidth to 184Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_92HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92) < 0)
        { // setting accel bandwidth to 92Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_92) < 0)
        { // setting gyro bandwidth to 92Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_41HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41) < 0)
        { // setting accel bandwidth to 41Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_41) < 0)
        { // setting gyro bandwidth to 41Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_20HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20) < 0)
        { // setting accel bandwidth to 20Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_20) < 0)
        { // setting gyro bandwidth to 20Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_10HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10) < 0)
        { // setting accel bandwidth to 10Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_10) < 0)
        { // setting gyro bandwidth to 10Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_5HZ:
    {
        if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5) < 0)
        { // setting accel bandwidth to 5Hz
            return -1;
        }
        if (writeRegister(CONFIG, GYRO_DLPF_5) < 0)
        { // setting gyro bandwidth to 5Hz
            return -2;
        }
        break;
    }
    }
    _bandwidth = bandwidth;
    return 1;
}

/* sets the sample rate divider to values other than default */
int MPU6500::setSrd(uint8_t srd)
{
    // use low speed SPI for register setting
    _useSPIHS = false;

    /* setting the sample rate divider */
    if (writeRegister(SMPDIV, srd) < 0)
    { // setting the sample rate divider
        return -4;
    }
    _srd = srd;
    return 1;
}

/* enables the data ready interrupt */
int MPU6500::enableDataReadyInterrupt()
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    /* setting the interrupt */
    if (writeRegister(INT_PIN_CFG, INT_PULSE_50US) < 0)
    { // setup interrupt, 50 us pulse
        return -1;
    }
    if (writeRegister(INT_ENABLE, INT_RAW_RDY_EN) < 0)
    { // set to data ready
        return -2;
    }
    return 1;
}

/* disables the data ready interrupt */
int MPU6500::disableDataReadyInterrupt()
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    if (writeRegister(INT_ENABLE, INT_DISABLE) < 0)
    { // disable interrupt
        return -1;
    }
    return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU6500::enableWakeOnMotion(float womThresh_mg, LpAccelOdr odr)
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    // reset the MPU
    writeRegister(PWR_MGMNT_1, PWR_RESET);
    // wait for MPU to come back up
    delay(1);
    if (writeRegister(PWR_MGMNT_1, 0x00) < 0)
    { // cycle 0, sleep 0, standby 0
        return -1;
    }
    if (writeRegister(PWR_MGMNT_2, DIS_GYRO) < 0)
    { // disable gyro measurements
        return -2;
    }
    if (writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) < 0)
    { // setting accel bandwidth to 184Hz
        return -3;
    }
    if (writeRegister(INT_ENABLE, INT_WOM_EN) < 0)
    { // enabling interrupt to wake on motion
        return -4;
    }
    if (writeRegister(MOT_DETECT_CTRL, (ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0)
    { // enabling accel hardware intelligence
        return -5;
    }
    _womThreshold = (uint8_t)floor(womThresh_mg * 255 / 1020);
    if (writeRegister(WOM_THR, _womThreshold) < 0)
    { // setting wake on motion threshold
        return -6;
    }
    if (writeRegister(LP_ACCEL_ODR, (uint8_t)odr) < 0)
    { // set frequency of wakeup
        return -7;
    }
    if (writeRegister(PWR_MGMNT_1, PWR_CYCLE) < 0)
    { // switch to accel low power mode
        return -8;
    }
    return 1;
}

/* configures and enables the FIFO buffer  */
int MPU6500FIFO::enableFifo(bool accel, bool gyro, bool temp)
{
    // use low speed SPI for register setting
    _useSPIHS = false;
    if (writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0)
    {
        return -1;
    }
    if (writeRegister(FIFO_EN, (accel * FIFO_ACCEL) | (gyro * FIFO_GYRO) | (temp * FIFO_TEMP)) < 0)
    {
        return -2;
    }
    _enFifoAccel = accel;
    _enFifoGyro = gyro;
    _enFifoTemp = temp;
    _fifoFrameSize = accel * 6 + gyro * 6 + temp * 2;
    return 1;
}

/* reads the most current data from MPU and stores in buffer */
int MPU6500::readSensor()
{
    _useSPIHS = true; // use the high speed SPI for data readout
    // grab the data from the MPU
    if (readRegisters(ACCEL_OUT, 21, _buffer) < 0)
    {
        return -1;
    }
    // combine into 16 bit values
    _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
    _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
    _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
    _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];

    // transform and convert to float values
    _ax = (((float)(tX[0] * _axcounts + tX[1] * _aycounts + tX[2] * _azcounts) * _accelScale) - _axb) * _axs;
    _ay = (((float)(tY[0] * _axcounts + tY[1] * _aycounts + tY[2] * _azcounts) * _accelScale) - _ayb) * _ays;
    _az = (((float)(tZ[0] * _axcounts + tZ[1] * _aycounts + tZ[2] * _azcounts) * _accelScale) - _azb) * _azs;
    _gx = ((float)(tX[0] * _gxcounts + tX[1] * _gycounts + tX[2] * _gzcounts) * _gyroScale) - _gxb;
    _gy = ((float)(tY[0] * _gxcounts + tY[1] * _gycounts + tY[2] * _gzcounts) * _gyroScale) - _gyb;
    _gz = ((float)(tZ[0] * _gxcounts + tZ[1] * _gycounts + tZ[2] * _gzcounts) * _gyroScale) - _gzb;
    _t = ((((float)_tcounts) - _tempOffset) / _tempScale) + _tempOffset;
    return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float MPU6500::getAccelX_mss()
{
    return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float MPU6500::getAccelY_mss()
{
    return _ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float MPU6500::getAccelZ_mss()
{
    return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float MPU6500::getGyroX_rads()
{
    return _gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float MPU6500::getGyroY_rads()
{
    return _gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float MPU6500::getGyroZ_rads()
{
    return _gz;
}

/* returns the die temperature, C */
float MPU6500::getTemperature_C()
{
    return _t;
}

/* reads data from the MPU FIFO and stores in buffer */
int MPU6500FIFO::readFifo()
{
    _useSPIHS = true; // use the high speed SPI for data readout
    // get the fifo size
    readRegisters(FIFO_COUNT, 2, _buffer);
    _fifoSize = (((uint16_t)(_buffer[0] & 0x0F)) << 8) + (((uint16_t)_buffer[1]));
    // read and parse the buffer
    for (size_t i = 0; i < _fifoSize / _fifoFrameSize; i++)
    {
        // grab the data from the MPU
        if (readRegisters(FIFO_READ, _fifoFrameSize, _buffer) < 0)
        {
            return -1;
        }
        if (_enFifoAccel)
        {
            // combine into 16 bit values
            _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
            _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
            _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
            // transform and convert to float values
            _axFifo[i] = (((float)(tX[0] * _axcounts + tX[1] * _aycounts + tX[2] * _azcounts) * _accelScale) - _axb) * _axs;
            _ayFifo[i] = (((float)(tY[0] * _axcounts + tY[1] * _aycounts + tY[2] * _azcounts) * _accelScale) - _ayb) * _ays;
            _azFifo[i] = (((float)(tZ[0] * _axcounts + tZ[1] * _aycounts + tZ[2] * _azcounts) * _accelScale) - _azb) * _azs;
            _aSize = _fifoSize / _fifoFrameSize;
        }
        if (_enFifoTemp)
        {
            // combine into 16 bit values
            _tcounts = (((int16_t)_buffer[0 + _enFifoAccel * 6]) << 8) | _buffer[1 + _enFifoAccel * 6];
            // transform and convert to float values
            _tFifo[i] = ((((float)_tcounts) - _tempOffset) / _tempScale) + _tempOffset;
            _tSize = _fifoSize / _fifoFrameSize;
        }
        if (_enFifoGyro)
        {
            // combine into 16 bit values
            _gxcounts = (((int16_t)_buffer[0 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[1 + _enFifoAccel * 6 + _enFifoTemp * 2];
            _gycounts = (((int16_t)_buffer[2 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[3 + _enFifoAccel * 6 + _enFifoTemp * 2];
            _gzcounts = (((int16_t)_buffer[4 + _enFifoAccel * 6 + _enFifoTemp * 2]) << 8) | _buffer[5 + _enFifoAccel * 6 + _enFifoTemp * 2];
            // transform and convert to float values
            _gxFifo[i] = ((float)(tX[0] * _gxcounts + tX[1] * _gycounts + tX[2] * _gzcounts) * _gyroScale) - _gxb;
            _gyFifo[i] = ((float)(tY[0] * _gxcounts + tY[1] * _gycounts + tY[2] * _gzcounts) * _gyroScale) - _gyb;
            _gzFifo[i] = ((float)(tZ[0] * _gxcounts + tZ[1] * _gycounts + tZ[2] * _gzcounts) * _gyroScale) - _gzb;
            _gSize = _fifoSize / _fifoFrameSize;
        }
    }
    return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU6500FIFO::getFifoAccelX_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _axFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU6500FIFO::getFifoAccelY_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _ayFifo, _aSize * sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU6500FIFO::getFifoAccelZ_mss(size_t *size, float *data)
{
    *size = _aSize;
    memcpy(data, _azFifo, _aSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void MPU6500FIFO::getFifoGyroX_rads(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gxFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void MPU6500FIFO::getFifoGyroY_rads(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gyFifo, _gSize * sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void MPU6500FIFO::getFifoGyroZ_rads(size_t *size, float *data)
{
    *size = _gSize;
    memcpy(data, _gzFifo, _gSize * sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU6500FIFO::getFifoTemperature_C(size_t *size, float *data)
{
    *size = _tSize;
    memcpy(data, _tFifo, _tSize * sizeof(float));
}

/* estimates the gyro biases */
int MPU6500::calibrateGyro()
{
    // set the range, bandwidth, and srd
    if (setGyroRange(GYRO_RANGE_250DPS) < 0)
    {
        return -1;
    }
    if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0)
    {
        return -2;
    }
    if (setSrd(19) < 0)
    {
        return -3;
    }

    // take samples and find bias
    _gxbD = 0;
    _gybD = 0;
    _gzbD = 0;
    for (size_t i = 0; i < _numSamples; i++)
    {
        readSensor();
        _gxbD += (getGyroX_rads() + _gxb) / ((double)_numSamples);
        _gybD += (getGyroY_rads() + _gyb) / ((double)_numSamples);
        _gzbD += (getGyroZ_rads() + _gzb) / ((double)_numSamples);
        delay(20);
    }
    _gxb = (float)_gxbD;
    _gyb = (float)_gybD;
    _gzb = (float)_gzbD;

    // set the range, bandwidth, and srd back to what they were
    if (setGyroRange(_gyroRange) < 0)
    {
        return -4;
    }
    if (setDlpfBandwidth(_bandwidth) < 0)
    {
        return -5;
    }
    if (setSrd(_srd) < 0)
    {
        return -6;
    }
    return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float MPU6500::getGyroBiasX_rads()
{
    return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float MPU6500::getGyroBiasY_rads()
{
    return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float MPU6500::getGyroBiasZ_rads()
{
    return _gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void MPU6500::setGyroBiasX_rads(float bias)
{
    _gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void MPU6500::setGyroBiasY_rads(float bias)
{
    _gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void MPU6500::setGyroBiasZ_rads(float bias)
{
    _gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int MPU6500::calibrateAccel()
{
    // set the range, bandwidth, and srd
    if (setAccelRange(ACCEL_RANGE_2G) < 0)
    {
        return -1;
    }
    if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0)
    {
        return -2;
    }
    if (setSrd(19) < 0)
    {
        return -3;
    }

    // take samples and find min / max
    _axbD = 0;
    _aybD = 0;
    _azbD = 0;
    for (size_t i = 0; i < _numSamples; i++)
    {
        readSensor();
        _axbD += (getAccelX_mss() / _axs + _axb) / ((double)_numSamples);
        _aybD += (getAccelY_mss() / _ays + _ayb) / ((double)_numSamples);
        _azbD += (getAccelZ_mss() / _azs + _azb) / ((double)_numSamples);
        delay(20);
    }
    if (_axbD > 9.0f)
    {
        _axmax = (float)_axbD;
    }
    if (_aybD > 9.0f)
    {
        _aymax = (float)_aybD;
    }
    if (_azbD > 9.0f)
    {
        _azmax = (float)_azbD;
    }
    if (_axbD < -9.0f)
    {
        _axmin = (float)_axbD;
    }
    if (_aybD < -9.0f)
    {
        _aymin = (float)_aybD;
    }
    if (_azbD < -9.0f)
    {
        _azmin = (float)_azbD;
    }

    // find bias and scale factor
    if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f))
    {
        _axb = (_axmin + _axmax) / 2.0f;
        _axs = G / ((abs(_axmin) + abs(_axmax)) / 2.0f);
    }
    if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f))
    {
        _ayb = (_axmin + _axmax) / 2.0f;
        _ays = G / ((abs(_aymin) + abs(_aymax)) / 2.0f);
    }
    if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f))
    {
        _azb = (_azmin + _azmax) / 2.0f;
        _azs = G / ((abs(_azmin) + abs(_azmax)) / 2.0f);
    }

    // set the range, bandwidth, and srd back to what they were
    if (setAccelRange(_accelRange) < 0)
    {
        return -4;
    }
    if (setDlpfBandwidth(_bandwidth) < 0)
    {
        return -5;
    }
    if (setSrd(_srd) < 0)
    {
        return -6;
    }
    return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float MPU6500::getAccelBiasX_mss()
{
    return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float MPU6500::getAccelScaleFactorX()
{
    return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float MPU6500::getAccelBiasY_mss()
{
    return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float MPU6500::getAccelScaleFactorY()
{
    return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float MPU6500::getAccelBiasZ_mss()
{
    return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float MPU6500::getAccelScaleFactorZ()
{
    return _azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void MPU6500::setAccelCalX(float bias, float scaleFactor)
{
    _axb = bias;
    _axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void MPU6500::setAccelCalY(float bias, float scaleFactor)
{
    _ayb = bias;
    _ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void MPU6500::setAccelCalZ(float bias, float scaleFactor)
{
    _azb = bias;
    _azs = scaleFactor;
}

bool MPU6500::available()
{
    uint8_t intStatus;
    readRegisters(0x3A, 1, &intStatus);
    return intStatus & 0x01;
}

/* writes a byte to MPU register given a register address and data */
int MPU6500::writeRegister(uint8_t subAddress, uint8_t data)
{
    /* write data to device */
    //TODO: set SPI speed to SPI_LS_CLOCK
    _spi.select();
    uint8_t buf[2] = {subAddress, data};
    _spi.transfer(buf, 2);
    _spi.release(); // end the transaction

    delay(10);

    /* read back the register */
    readRegisters(subAddress, 1, _buffer);
    /* check the read back register against the written register */
    if (_buffer[0] == data)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

/* reads registers from MPU given a starting register address, number of bytes, and a pointer to store data */
int MPU6500::readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    // begin the transaction
    if (_useSPIHS)
    {
        //TODO: set speed to SPI_HS_CLOCK
    }
    else
    {
        //TODO: set speed to SPI_LS_CLOCK
    }
    _spi.select();
    subAddress |= SPI_READ;
    _spi.transfer(&subAddress, 1); // specify the starting register address
    _spi.transfer(dest, count);
    _spi.release();
    return 1;
}

/* gets the MPU WHO_AM_I register value */
int MPU6500::whoAmI()
{
    // read the WHO AM I register
    if (readRegisters(WHO_AM_I, 1, _buffer) < 0)
    {
        return -1;
    }
    // return the register value
    return _buffer[0];
}
