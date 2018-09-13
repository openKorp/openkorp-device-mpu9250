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
// #include <fstream>
#include <unistd.h>

#include <eigen3/Eigen/SVD>

#include "MPU9250Device.hpp"

/**
 * Constructor for MPU9250 device interfacing through I2C.
 */
MPU9250Device::MPU9250Device(std::string const &a_deviceName, bool const &a_calibrate)
    : m_deviceFile()
    , m_addressType()
    , m_instrumentAdress()
    , m_accCal()
    , m_gyroCal()
    , m_magCal()
    , m_accCalFile("acc.cal")
    , m_gyroCalFile("gyro.cal")
    , m_magCalFile("mag.cal")
    , m_afsr(MPU9250Device::A_SCALE::AFS_2G)
    , m_gfsr(MPU9250Device::G_SCALE::GFS_250DPS)
    , m_accConversion(0.0f)
    , m_gyroConversion(0.0f)
    , m_adlpf(MPU9250Device::A_DLPF::ADLPF_184)
    , m_gdlpf(MPU9250Device::G_DLPF::GDLPF_184)
    , m_mfsr(MPU9250Device::M_SCALE::MFS_14BITS)
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
  initMpu();
  (void) a_calibrate;
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
  i2cAccessDevice(MPU9250_ADDRESS);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::PWR_MGMT_1, 0x01<<7});
  usleep(10000); // Wait for all registers to reset

  if (i2cReadRegister(std::vector<uint8_t>{MPU9250::WHO_AM_I_MPU9250}, 1).at(0) != 0x71) {
    std::cerr << "[MPU9250] Wrong who am I code returned. " << std::endl;
  }
}

void MPU9250Device::initMpu()
{
  resetMpu();
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG2, MPU9250::BIT_FIFO_SIZE_1024 | 0x8});
  m_accCal = Calibration(m_accCalFile);
  m_gyroCal = Calibration(m_gyroCalFile);
  setAccCalibration();
  setGyroCalibration();
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::SMPLRT_DIV, 0x00});
  setAccFullScaleRange(m_afsr);
  setGyroFullScaleRange(m_gfsr);
  setAccDigitalLowPassFilter(m_adlpf);
  setGyroDigitalLowPassFilter(m_gdlpf);
}


std::vector<float> MPU9250Device::getGyroCalibration()
{
  i2cAccessDevice(MPU9250_ADDRESS);
  std::cout << "[MPU9250] Starting calibration gyroscope...\n";
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

  float xOffset = 0;
  float yOffset = 0;
  float zOffset = 0;

  while(calibrate){
    float const DEVIATION_THRESHOLD = 50;
    float const OFFSET_THRESHOLD = 500;

    i2cWriteRegister(std::vector<uint8_t>{MPU9250::USER_CTRL, 0x40});
    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, MPU9250::FIFO_GYRO_X_EN | MPU9250::FIFO_GYRO_Y_EN | MPU9250::FIFO_GYRO_Z_EN});
    usleep(1000000);

    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, 0x00});
    
    rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_COUNTH}, 2);

    int16_t fifoCount = ((uint16_t) rawData.at(0) <<  8) | rawData.at(1);

    std::cout << "[MPU9250] FIFO Count: " << fifoCount << std::endl;
    int32_t sampleCount = fifoCount/6;
    std::cout << "[MPU9250] Sample Count: " << sampleCount << std::endl;

    Eigen::VectorXf x = Eigen::VectorXf::Zero(sampleCount);
    Eigen::VectorXf y = Eigen::VectorXf::Zero(sampleCount);
    Eigen::VectorXf z = Eigen::VectorXf::Zero(sampleCount);

    // // std::vector<float> gyroOffset;  
    // int32_t gyroOffset[3] = {0,0,0};
    for (uint8_t i = 0; i < sampleCount; i++) {
      // int16_t gyroSampl[3] = {0,0,0};
      rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_R_W}, 6);

      x(i) = (int16_t) (((int16_t)rawData.at(0) << 8) | rawData.at(1));
      y(i) = (int16_t) (((int16_t)rawData.at(2) << 8) | rawData.at(3));
      z(i) = (int16_t) (((int16_t)rawData.at(4) << 8) | rawData.at(5));

    }
    xOffset = x.mean();
    float xDeviation = std::sqrt(((x.array()-xOffset).pow(2).sum()/(sampleCount-1)));
    yOffset = y.mean();
    float yDeviation = std::sqrt(((y.array()-yOffset).pow(2).sum()/(sampleCount-1)));
    zOffset = z.mean();
    float zDeviation = std::sqrt(((z.array()-zOffset).pow(2).sum()/(sampleCount-1)));
    
    calibrate = false;

    if (xDeviation > DEVIATION_THRESHOLD || yDeviation > DEVIATION_THRESHOLD || zDeviation > DEVIATION_THRESHOLD) {
      std::cout << "[MPU9250] Deviation too high: " << xDeviation << ", " 
          << yDeviation << ", " << zDeviation << ".\n" 
          << "Recalibrating..." << std::endl;
      calibrate = true;
    }
    if(std::abs(xOffset) > OFFSET_THRESHOLD ||  std::abs(yOffset) > OFFSET_THRESHOLD ||  std::abs(zOffset) > OFFSET_THRESHOLD) {
      std::cout << "[MPU9250] Offset too high: " << xOffset << ", " 
          << yOffset << ", " << zOffset << ".\n" 
          << "Recalibrating..." << std::endl;
      calibrate = true;
    }
        
  }
  std::cout << "[MPU9250] Gyro calibration successful, found bias: " << xOffset << ", " 
      << yOffset << ", " << zOffset << "." << std::endl;
  return std::vector<float>{xOffset, yOffset, zOffset};
}

std::vector<float> MPU9250Device::getAccCalibration()
{
  std::vector<std::string> const axis{"x","y","z"};
  std::vector<std::string> const orientation{"upwards","downwards"};
  i2cAccessDevice(MPU9250_ADDRESS);
  std::cout << "[MPU9250] Starting calibration accelerometer...\n";
  resetMpu();

  // Ellipsoid approx
  // First row: center point, Second row: length axis
  Eigen::MatrixXf calibration(2,3);

  Eigen::MatrixXf measurementAverage(6,3);

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

  // collect data
  uint8_t j{0};
  for (const std::string& ax : axis) {
    for (const std::string& orient : orientation) {
      std::cout << "Orient the " << ax << "-axis pointing " << orient << " and keep still..."
          << "\nCalibrating in\n";
      for (uint8_t i = 10; i > 0; i--) {
        usleep(1000000);
        std::cout << i << std::endl;
      }
      measurementAverage.row(j) = sampleAccelerometer();
      j++;
    }
  }
  measurementAverage = measurementAverage/16384.0f;




  return std::vector<float>{};
}

Eigen::Vector3f MPU9250Device::sampleAccelerometer()
{
  float const DEVIATION_THRESHOLD = 100;
  Eigen::Vector3f mean;
  Eigen::Vector3f deviation;
  std::vector<uint8_t> rawData;
  std::cout << "Calibrating the accelerometer..." << std::endl;
  
  while ((deviation.array().abs() > DEVIATION_THRESHOLD).sum() > 0) {
    std::cout << "Keep the sensor still..." << std::endl;
  
    i2cWriteRegister(std::vector<uint8_t>{MPU9250::USER_CTRL, 0x40});
    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, MPU9250::FIFO_ACCEL_EN});
    usleep(1000000);
    i2cWriteRegister(std::vector<uint8_t>{MPU9250::FIFO_EN, 0x00});
    
    rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_COUNTH}, 2);

    int16_t fifoCount = ((uint16_t) rawData.at(0) <<  8) | rawData.at(1);

    std::cout << "[MPU9250] FIFO Count: " << fifoCount << std::endl;
    int32_t sampleCount = fifoCount/6;
    std::cout << "[MPU9250] Sample Count: " << sampleCount << std::endl;

    Eigen::MatrixXf samples(sampleCount, 3);

    for (uint8_t i = 0; i < sampleCount; i++) {
      // int16_t gyroSampl[3] = {0,0,0};
      rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::FIFO_R_W}, 6);

      samples(i, 0) = (int16_t) (((int16_t)rawData.at(0) << 8) | rawData.at(1));
      samples(i, 1) = (int16_t) (((int16_t)rawData.at(2) << 8) | rawData.at(3));
      samples(i, 2) = (int16_t) (((int16_t)rawData.at(4) << 8) | rawData.at(5));

    }
    for (uint8_t i = 0; i < 3; i++) {
      mean(i) = samples.col(i).mean();
      deviation(i) = std::sqrt(((samples.col(i).array()-mean(i)).pow(2).sum()/(sampleCount-1)));
    }
    
  }

  return mean;
}

int8_t MPU9250Device::setGyroCalibration()
{
  Eigen::Vector3f offset = m_gyroCal.getCenter();

  i2cAccessDevice(MPU9250_ADDRESS);

  int16_t xOffset = std::lround(offset(0));
  int16_t yOffset = std::lround(offset(1));
  int16_t zOffset = std::lround(offset(2));

  uint8_t xh = (-xOffset/4 >> 8);
  uint8_t xl = ((-xOffset/4) & 0xFF);
  uint8_t yh = (-yOffset/4 >> 8);
  uint8_t yl = ((-yOffset/4) & 0xFF);
  uint8_t zh = (-zOffset/4 >> 8);
  uint8_t zl = ((-zOffset/4) & 0xFF);

  i2cWriteRegister(std::vector<uint8_t>{MPU9250::XG_OFFSET_H, xh, xl, yh, yl, zh, zl});

  return 0;
  
}

int8_t MPU9250Device::setAccCalibration()
{
  Eigen::Vector3f offset = m_accCal.getCenter();

  i2cAccessDevice(MPU9250_ADDRESS);

  int16_t xOffset = std::lround(offset(0));
  int16_t yOffset = std::lround(offset(1));
  int16_t zOffset = std::lround(offset(2));

  uint8_t xh = (xOffset >> 7) & 0xFF;
  uint8_t xl = (xOffset << 1) & 0xFF;
  uint8_t yh = (yOffset >> 7) & 0xFF;
  uint8_t yl = (yOffset << 1) & 0xFF;
  uint8_t zh = (zOffset >> 7) & 0xFF;
  uint8_t zl = (zOffset << 1) & 0xFF;

  i2cWriteRegister(std::vector<uint8_t>{MPU9250::XA_OFFSET_H, xh, xl, yh, yl, zh, zl});
  return 0;
}

opendlv::proxy::AccelerationReading MPU9250Device::readAccelerometer()
{
  i2cAccessDevice(MPU9250_ADDRESS);
  std::vector<uint8_t> rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::ACCEL_XOUT_H}, 6);

  float const c = m_accConversion;

  int16_t x = (((int16_t)rawData[0] << 8) | rawData[1] );
  int16_t y = (((int16_t)rawData[2] << 8) | rawData[3] );
  int16_t z = (((int16_t)rawData[4] << 8) | rawData[5] );

  opendlv::proxy::AccelerationReading reading;
  reading.accelerationX(x*c);
  reading.accelerationY(y*c);
  reading.accelerationZ(z*c);
  return reading;
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

opendlv::proxy::PressureReading MPU9250Device::readAltimeter()
{
  return opendlv::proxy::PressureReading();
}


opendlv::proxy::AngularVelocityReading MPU9250Device::readGyroscope()
{
  i2cAccessDevice(MPU9250_ADDRESS);
  std::vector<uint8_t> rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::GYRO_XOUT_H}, 6);

  float const c = m_gyroConversion;

  int16_t x = (((int16_t)rawData[0] << 8) | rawData[1] );
  int16_t y = (((int16_t)rawData[2] << 8) | rawData[3] );
  int16_t z = (((int16_t)rawData[4] << 8) | rawData[5] );
  
  opendlv::proxy::AngularVelocityReading reading;
  reading.angularVelocityX(x*c);
  reading.angularVelocityY(y*c);
  reading.angularVelocityZ(z*c);
  return reading;
}

opendlv::proxy::TemperatureReading MPU9250Device::readThermometer()
{
  i2cAccessDevice(MPU9250_ADDRESS);
  std::vector<uint8_t> rawData = i2cReadRegister(std::vector<uint8_t>{MPU9250::TEMP_OUT_H}, 2);

  float temp = (((int16_t)rawData[0] << 8) | rawData[1]) / 1.0f;


  opendlv::proxy::TemperatureReading reading;
  reading.temperature(21.0f + temp / 333.87f);
  return reading;
}

void MPU9250Device::setAccFullScaleRange(A_SCALE const &a_fsr)
{
  uint8_t val = 0;
  switch (a_fsr) {
    case AFS_2G:
      val = AFS_2G;
      m_accConversion = (9.80665f * 2.0f / 32768.0f);
      break;
    case AFS_4G:
      val = AFS_4G;
      m_accConversion = (9.80665f * 4.0f / 32768.0f);
      break;
    case AFS_8G:
      val = AFS_8G;
      m_accConversion = (9.80665f * 8.0f / 32768.0f);
      break;
    case AFS_16G:
      val = AFS_16G;
      m_accConversion = (9.80665f * 16.0f / 32768.0f);
      break;
    default:
      m_accConversion = 0.0f;
      return;
      break;
  }
  i2cAccessDevice(MPU9250_ADDRESS);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG, val});
}

void MPU9250Device::setGyroFullScaleRange(G_SCALE const &a_fsr)
{
  uint8_t val = 0;
  float const conversion =  static_cast<float>(M_PI) / 180.0f;
  
  switch (a_fsr) {
    case GFS_250DPS:
      val = MPU9250::GYRO_FSR_CFG_250 | MPU9250::FCHOICE_B_DLPF_EN;
      m_gyroConversion = (250.0f / 32768.0f) * conversion;
      break;
    case GFS_500DPS:
      val = MPU9250::GYRO_FSR_CFG_500 | MPU9250::FCHOICE_B_DLPF_EN;
      m_gyroConversion = (500.0f / 32768.0f) * conversion;
      break;
    case GFS_1000DPS:
      val = MPU9250::GYRO_FSR_CFG_1000 | MPU9250::FCHOICE_B_DLPF_EN;
      m_gyroConversion = (1000.0f / 32768.0f) * conversion;
      break;
    case GFS_2000DPS:
      val = MPU9250::GYRO_FSR_CFG_2000 | MPU9250::FCHOICE_B_DLPF_EN;
      m_gyroConversion = (2000.0f / 32768.0f) * conversion;
      break;
    default:
      m_gyroConversion = 0.0f;
      return;
      break;
  }
  i2cAccessDevice(MPU9250_ADDRESS);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::GYRO_CONFIG, val});
}
void MPU9250Device::setAccDigitalLowPassFilter(A_DLPF const &a_dlpf)
{
  uint8_t buf = MPU9250::ACCEL_FCHOICE_1KHZ | MPU9250::BIT_FIFO_SIZE_1024;
  switch (a_dlpf) {
    case ADLPF_OFF:
      buf = MPU9250::ACCEL_FCHOICE_4KHZ | MPU9250::BIT_FIFO_SIZE_1024;
      break;
    case ADLPF_460:
      buf |= 0; 
      break;
    case ADLPF_184:
      buf |= 1;
      break;
    case ADLPF_92:
      buf |= 2;
      break;
    case ADLPF_41:
      buf |= 3;
      break;
    case ADLPF_20:
      buf |= 4;
      break;
    case ADLPF_10:
      buf |= 5;
      break;
    case ADLPF_5:
      buf |= 6;
      break;
    default:
      return;
      break;
  }
  i2cAccessDevice(MPU9250_ADDRESS);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::ACCEL_CONFIG2, buf});
} 
void MPU9250Device::setGyroDigitalLowPassFilter(G_DLPF const &a_dlpf)
{
  uint8_t buf = 0;
  switch (a_dlpf) { 
    case GDLPF_OFF:
      buf |= 7;
      break;
    case GDLPF_250:
      buf |= 0;
      break;
    case GDLPF_184:
      buf |= 1;
      break;
    case GDLPF_92:
      buf |= 2;
      break;
    case GDLPF_41:
      buf |= 3;
      break;
    case GDLPF_20:
      buf |= 4;
      break;
    case GDLPF_10:
      buf |= 5;
      break;
    case GDLPF_5:
      buf |= 6;
      break;
    default:
      return;
      break;
  }
  i2cAccessDevice(MPU9250_ADDRESS);
  i2cWriteRegister(std::vector<uint8_t>{MPU9250::CONFIG, buf});
} 