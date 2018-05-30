/**
 * Copyright (C) 2017 openKorp
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

// Libraries required for i2c-bus
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include <eigen3/Eigen/Dense>

#include "MPU9250Device.h"

/**
 * Constructor for MPU9250 device interfacing through I2C.
 */
MPU9250Device::MPU9250Device(std::string const &a_deviceName, bool const &a_calibrate)
    : m_deviceFile()
    , m_addressType()
    , m_instrumentAdress()
    , m_accCalFile("acc.cal")
    , m_gyroCalFile("gyro.cal")
    , m_magCalFile("mag.cal")
    , m_gscale(MPU9250Device::G_SCALE::GFS_250DPS)
    , m_ascale(MPU9250Device::A_SCALE::AFS_2G)
    , m_mscale(MPU9250Device::M_SCALE::MFS_14BITS)
    , m_mmode(MPU9250Device::M_MODE::M_100HZ)
{
  m_deviceFile = open(a_deviceName.c_str(), O_RDWR);
  if (m_deviceFile < 0) {
    std::cerr << "[MPU9250] Failed to open the i2c bus:" << a_deviceName 
        << "." << std::endl;
  } else {
    std::cout << "[MPU9250] I2C bus " << a_deviceName 
        << " opened successfully." << std::endl;
  }
  if (a_calibrate) {
    std::vector<float> gyroCal = getGyroCalibration();
    saveGyroCalibration(gyroCal);
  } else {
    initMpu();
  }


  // std::vector<float> gyroBiasVec = loadGyroCalibration();
  // if (gyroBiasVec.empty()) {
  //   gyroBiasVec = calibrateMPU9250();
  //   setGyroOffset(gyroBiasVec); 
  //   saveGyroCalibrationFile(gyroBiasVec);
  // }

}


MPU9250Device::~MPU9250Device()
{}

void MPU9250Device::i2cWriteRegister(std::vector<uint8_t> const &a_data)
{
  if (a_data.empty()) {
    std::cerr << "[MPU9250] Failed to write on I2C bus: Input size 0." << std::endl;
  }
  uint8_t* buffer = (uint8_t *) a_data.data();
  if (write(m_deviceFile, buffer, a_data.size()) != static_cast<int32_t>(a_data.size())) {
    std::cerr << "[MPU9250] Failed to write on I2C bus: Failed to communicate." << std::endl;
  }
}

std::vector<uint8_t> MPU9250Device::i2cReadRegister(std::vector<uint8_t> const &a_addr, uint8_t const &a_length)
{
  i2cWriteRegister(a_addr);
  std::vector<uint8_t> vec(a_length);
  if (read(m_deviceFile, vec.data(), a_length) != a_length) {
    std::cerr << "[MPU9250] Failed to read I2C on bus." << std::endl;
  }
  return vec;
}

int8_t MPU9250Device::i2cAccessDevice(uint8_t const a_addr)
{
  if (ioctl(m_deviceFile, I2C_SLAVE, a_addr) < 0) {
    std::cerr << "[MPU9250] Failed to acquire bus access or talk to slave device. " << std::endl;
    return -1;
  }
  return 0;
}

void MPU9250Device::resetMpu()
{
  // wake up device
  // Clear sleep mode bit (6), enable all sensors
  uint8_t addr = MPU9250_ADDRESS;
  i2cAccessDevice(addr);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::PWR_MGMT_1, 0x01<<7});
  usleep(10000); // Wait for all registers to reset


  if (i2cReadRegister(std::vector<uint8_t>{MPU9250::WHO_AM_I_MPU9250}, 1).at(0) != 0x71) {
    std::cerr << "[MPU9250] Wrong who am I code returned. " << std::endl;
  }




  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  // reg = MPU9250::CONFIG;
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::CONFIG, 0x03});

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  // reg = MPU9250::SMPLRT_DIV;
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::SMPLRT_DIV, 0x04});

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // get current GYRO_CONFIG register value
  // uint8_t c;
  // reg = MPU9250::GYRO_CONFIG;
  // i2cReadRegister(reg, &c, 1);
  // std::vector<uint8_t> c = i2cReadRegister(std::vector<uint8_t>{MPU9250::GYRO_CONFIG}, 1);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  // c[0] = c[0] & ~0x02; // Clear Fchoice bits [1:0]
  // c[0] = c[0] & ~0x18; // Clear AFS bits [4:3]
  // c[0] = c[0] | m_gscale << 3; // Set full scale range for the gyro
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  // Write new GYRO_CONFIG value to register
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::GYRO_CONFIG, c[0]});

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  // reg = MPU9250::ACCEL_CONFIG;
  // c = i2cReadRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG}, 1);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  // c[0] = c[0] & ~0x18;  // Clear AFS bits [4:3]
  // c[0] = c[0] | m_ascale << 3; // Set full scale range for the accelerometer
  // Write new ACCEL_CONFIG register value
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG, c[0]});

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  // Get current ACCEL_CONFIG2 register value
  // reg = MPU9250::ACCEL_CONFIG2;
  // c = i2cReadRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG2}, 1);
  // c[0] = c[0] & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  // c[0] = c[0] | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // Write new ACCEL_CONFIG2 register value
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG2, c[0]});
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS, and enable
  // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
  // controlled by the Arduino as master.
  // reg = MPU9250::INT_PIN_CFG;
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::INT_PIN_CFG, 0x22});
  // Enable data ready (bit 0) interrupt
  // reg = MPU9250::INT_ENABLE;
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::INT_ENABLE, 0x01});
  // usleep(100000);
}

void MPU9250Device::initMpu()
{
  resetMpu();
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG2, MPU9250::BIT_FIFO_SIZE_1024 | 0x8});


}


std::vector<float> MPU9250Device::getGyroCalibration()
{
  i2cAccessDevice(MPU9250_ADDRESS);
  std::cout << "[MPU9250] Starting calibration...\n";
  resetMpu();

  std::vector<uint8_t> rawData;


  i2cWriteRegister(std::vector<uint8_t>{MPU9250::PWR_MGMT_1, 0x01});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::PWR_MGMT_2, 0x00});
  usleep(200000);

  i2cWriteRegister(std::vector<uint8_t>{MPU9250::INT_ENABLE, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::PWR_MGMT_1, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::I2C_MST_CTRL, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::USER_CTRL, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::USER_CTRL, 0x0C});
  usleep(15000);


  i2cWriteRegister(std::vector<uint8_t>{MPU9250::CONFIG, 0x01});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::SMPLRT_DIV, 0x04});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::GYRO_CONFIG, 0x00});
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG, 0x00});

  //Settings done
  
  // float const gyroSens  = (250.0f / 32768.0f * static_cast<float>(M_PI) / 180.0f);

  bool calibrate = true;

  float xBias = 0;
  float yBias = 0;
  float zBias = 0;

  while(calibrate){
    float const DEVIATION_THRESHOLD = 50;
    float const BIAS_THRESHOLD = 500;

    i2cWriteRegister(std::vector<uint8_t>{MPU9250::USER_CTRL, 0x40});
    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, MPU9250::FIFO_GYRO_X_EN | MPU9250::FIFO_GYRO_Y_EN | MPU9250::FIFO_GYRO_Z_EN});
    usleep(400000);

    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, 0x00});
    
    rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_COUNTH}, 2);

    int16_t fifoCount = ((uint16_t) rawData.at(0) <<  8) | rawData.at(1);

    std::cout << "[MPU9250] FIFO Count: " << fifoCount << std::endl;
    int32_t sampleCount = fifoCount/6;
    std::cout << "[MPU9250] Sample Count: " << sampleCount << std::endl;

    Eigen::VectorXf x = Eigen::VectorXf::Zero(sampleCount);
    Eigen::VectorXf y = Eigen::VectorXf::Zero(sampleCount);
    Eigen::VectorXf z = Eigen::VectorXf::Zero(sampleCount);

    // // std::vector<float> gyroBias;  
    // int32_t gyroBias[3] = {0,0,0};
    for (uint8_t i = 0; i < sampleCount; i++) {
      // int16_t gyroSampl[3] = {0,0,0};
      rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_R_W}, 6);

      x(i) = (int16_t) (((int16_t)rawData.at(0) << 8) | rawData.at(1));
      y(i) = (int16_t) (((int16_t)rawData.at(2) << 8) | rawData.at(3));
      z(i) = (int16_t) (((int16_t)rawData.at(4) << 8) | rawData.at(5));

    }
    xBias = x.mean();
    float xDeviation = std::sqrt(((x.array()-xBias).pow(2).sum()/(sampleCount-1)));
    yBias = y.mean();
    float yDeviation = std::sqrt(((y.array()-yBias).pow(2).sum()/(sampleCount-1)));
    zBias = z.mean();
    float zDeviation = std::sqrt(((z.array()-zBias).pow(2).sum()/(sampleCount-1)));
    
    calibrate = false;

    if (xDeviation > DEVIATION_THRESHOLD || yDeviation > DEVIATION_THRESHOLD || zDeviation > DEVIATION_THRESHOLD) {
      std::cout << "[MPU9250] Deviation too high: " << xDeviation << ", " 
          << yDeviation << ", " << zDeviation << ".\n" 
          << "Recalibrating..." << std::endl;
      calibrate = true;
    }
    if(std::abs(xBias) > BIAS_THRESHOLD ||  std::abs(yBias) > BIAS_THRESHOLD ||  std::abs(zBias) > BIAS_THRESHOLD) {
      std::cout << "[MPU9250] Bias too high: " << xBias << ", " 
          << yBias << ", " << zBias << ".\n" 
          << "Recalibrating..." << std::endl;
      calibrate = true;
    }
        
  }
  std::cout << "[MPU9250] Gyro calibration successful, found bias: " << xBias << ", " 
      << yBias << ", " << zBias << "." << std::endl;
  return std::vector<float>{xBias, yBias, zBias};
}


void MPU9250Device::saveGyroCalibration(std::vector<float> a_offset)
{
  if (a_offset.size() != 3) {
    std::cerr << "[MPU9250] saveGyroCalibrationFile received a vector of a length not supported." << std::endl;
  }

  std::ofstream gyroCalibrationFile(m_gyroCalFile);
  if (gyroCalibrationFile.is_open()) {
    std::cout << "[MPU9250] Saved gyro cal:" << a_offset.at(0) << "\n" << a_offset.at(1) << "\n" << a_offset.at(2) << "\n";
    gyroCalibrationFile << a_offset.at(0) << "\n" << a_offset.at(1) << "\n" << a_offset.at(2) << "\n";
  } else {
    std::cout << "[MPU9250] Unable to save calibration file. Tried to open: " + m_gyroCalFile + "\n";
  }
  gyroCalibrationFile.flush();
  gyroCalibrationFile.close();
}

void MPU9250Device::loadGyroCalibration() 
{
  std::vector<float> gyroCal{0,0,0};
  std::ifstream file(m_gyroCalFile, std::ifstream::in);
  if (file.is_open()){
    std::string line;
    for (uint8_t i = 0; i < 3; ++i) {
      std::getline(file, line);
      try{
        gyroCal.at(i) = std::stof(line);
      } catch (std::invalid_argument e) {
        std::cerr << "[MPU9250] Invalid calibration file format." << std::endl;
        file.close();
      }
    }
    std::cout << "[MPU9250] Loaded the calibration settings." << std::endl;
    std::cout << "\nLoaded:"
        << " Gyro: " << gyroCal.at(0) << ", " 
        << gyroCal.at(1) << ", " 
        << gyroCal.at(2) << std::endl;
    file.close();
  } else {
    std::cout << "[MPU9250] Could not load the calibration settings. Tried to open: " 
        << m_gyroCalFile << std::endl;
    file.close();
    std::cout << "[MPU9250] Calibrating the gyroscope instead..." << std::endl;
    

  }
}

void MPU9250Device::setGyroCalibration(std::vector<float> a_offset)
{
  (void)a_offset;
  // if (a_offset.size() != 3) {
  //   std::cerr << "[MPU9250] setGyroOffset received a vector of a length not supported." << std::endl;
  //   return -1;
  // }

  // float const gyroSens  = (250.0f / 32768.0f * static_cast<float>(M_PI) / 180.0f);

  // int32_t xOffset = std::lround(a_offset.at(0) / gyroSens);
  // int32_t yOffset = std::lround(a_offset.at(1) / gyroSens);
  // int32_t zOffset = std::lround(a_offset.at(2) / gyroSens);

  // uint8_t xh = (-xOffset/4 >> 8);
  // uint8_t xl = ((-xOffset/4) & 0xFF);
  // uint8_t yh = (-yOffset/4 >> 8);
  // uint8_t yl = ((-yOffset/4) & 0xFF);
  // uint8_t zh = (-zOffset/4 >> 8);
  // uint8_t zl = ((-zOffset/4) & 0xFF);

  // i2cAccessDevice(MPU9250_ADDRESS);
  // i2cWriteRegister(std::vector<uint8_t>{MPU9250::XG_OFFSET_H, xh, xl, yh, yl, zh, zl});
  
}

opendlv::proxy::AccelerationReading MPU9250Device::readAccelerometer()
{
  // uint8_t addr = MPU9250_ADDRESS;
  // i2cAccessDevice(addr);
  // uint8_t reg = MPU9250::ACCEL_XOUT_H;
  // uint8_t rawData[6];
  // i2cReadRegister(reg, &rawData[0], 6);

  // float const c = getAscale();

  // int16_t x = (((int16_t)rawData[0] << 8) | rawData[1] );
  // int16_t y = (((int16_t)rawData[2] << 8) | rawData[3] );
  // int16_t z = (((int16_t)rawData[4] << 8) | rawData[5] );
  // opendlv::proxy::AccelerationReading accelerometerReading(x*c,y*c,z*c);
  // std::cout << "magneto x: " << x;
  // std::cout << "magneto y: " << y;
  // std::cout << "magneto z: " << z;
  // opendlv::proxy::AccelerationReading accelerometerReading(0,0,0);
  return opendlv::proxy::AccelerationReading();
}

opendlv::proxy::MagneticFieldReading MPU9250Device::readMagnetometer()
{
  // uint8_t addr = MPU9250_ADDRESS;
  // i2cAccessDevice(addr);
  // uint8_t reg = MPU9250::AK8963_XOUT_L;
  // uint8_t rawData[6];
  // i2cReadRegister(reg, &rawData[0], 6);

  // int16_t x = (((int16_t)rawData[0] << 8) | rawData[1] );
  // int16_t y = (((int16_t)rawData[2] << 8) | rawData[3] );
  // int16_t z = (((int16_t)rawData[4] << 8) | rawData[5] );

  // opendlv::proxy::MagneticFieldReading magnetometerReading(x,y,z);
  return opendlv::proxy::MagneticFieldReading();
}


opendlv::proxy::AngularVelocityReading MPU9250Device::readGyroscope()
{
  // uint8_t reg = MPU9250::GYRO_XOUT_H;
  // uint8_t rawData[6];
  // i2cReadRegister(reg, &rawData[0], 6);

  // float const c = getGscale(true);
  
  // int16_t x = (((int16_t)rawData[0] << 8) | rawData[1] );
  // int16_t y = (((int16_t)rawData[2] << 8) | rawData[3] );
  // int16_t z = (((int16_t)rawData[4] << 8) | rawData[5] );
  
  // opendlv::proxy::AngularVelocityReading gyroscopeReading(x*c,y*c,z*c);
  // opendlv::proxy::AngularVelocityReading gyroscopeReading(0,0,0);
  return opendlv::proxy::AngularVelocityReading();
}
opendlv::proxy::PressureReading MPU9250Device::readAltimeter()
{
  // opendlv::proxy::PressureReading altimeterReading(0);
  return opendlv::proxy::PressureReading();
}

opendlv::proxy::TemperatureReading MPU9250Device::readThermometer()
{
  // uint8_t addr = MPU9250_ADDRESS;
  // i2cAccessDevice(addr);
  // uint8_t reg = MPU9250::TEMP_OUT_H;
  // uint8_t rawData[2];
  // i2cReadRegister(reg, &rawData[0], 2);

  // int16_t temp = (((int16_t)rawData[0] << 8) | rawData[1]) / 1.0f;

  // opendlv::proxy::TemperatureReading temperatureReading(21.0f + temp / 333.87f);
  // opendlv::proxy::TemperatureReading temperatureReading(0);
  return opendlv::proxy::TemperatureReading();
}

void MPU9250Device::setAscale(A_SCALE a_scale)
{
  m_ascale = a_scale;
}

float MPU9250Device::getAscale()
{
  switch (m_ascale) {
    case AFS_2G:
      return (9.82f * 2.0f / 32768.0f);
    case AFS_4G:
      return (9.82f * 4.0f / 32768.0f);
    case AFS_8G:
      return (9.82f * 8.0f / 32768.0f);
    case AFS_16G:
      return (9.82f * 16.0f / 32768.0f);
    default:
      return 0.0f;
  }
}

void MPU9250Device::setGscale(G_SCALE a_scale)
{
  m_gscale = a_scale;
}

float MPU9250Device::getGscale(bool a_radFlag)
{
    float conversion = 1;
    if (a_radFlag) {
      conversion = static_cast<float>(M_PI) / 180.0f;
    }
    switch (m_ascale) {
    case GFS_250DPS:
      return (250.0f / 32768.0f) * conversion;
    case GFS_500DPS:
      return (500.0f / 32768.0f) * conversion;
    case GFS_1000DPS:
      return (1000.0f / 32768.0f) * conversion;
    case GFS_2000DPS:
      return (2000.0f / 32768.0f) * conversion;
    default:
      return 0.0f;   
  }
}


