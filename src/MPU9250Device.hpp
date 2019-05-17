/**
 * Copyright (C) 2018 openKorp
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
#pragma once
#ifndef mpudevice_h
#define mpudevice_h


#include <string>
#include <vector>



#include "opendlv-standard-message-set.hpp"
#include "calibration.hpp"


class MPU9250Device{
 public:
  MPU9250Device(std::string const &, bool const &);
  MPU9250Device(MPU9250Device const &) = delete;
  MPU9250Device &operator=(MPU9250Device const &) = delete;
  virtual ~MPU9250Device();
  opendlv::proxy::AccelerationReading readAccelerometer();
  opendlv::proxy::AngularVelocityReading readGyroscope();
  opendlv::proxy::MagneticFieldReading readMagnetometer();
  opendlv::proxy::PressureReading readAltimeter();
  opendlv::proxy::TemperatureReading readThermometer();

  
 private:
  enum A_SCALE {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
  };

  enum G_SCALE {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
  };
  enum A_DLPF {
    ADLPF_OFF = 0,
    ADLPF_460,
    ADLPF_184,
    ADLPF_92,
    ADLPF_41,
    ADLPF_20,
    ADLPF_10,
    ADLPF_5 
  };
  enum G_DLPF {
    GDLPF_OFF = 0,
    GDLPF_250,
    GDLPF_184,
    GDLPF_92,
    GDLPF_41,
    GDLPF_20,
    GDLPF_10,
    GDLPF_5
  };
  enum M_SCALE {
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
  };

  enum M_MODE {
    M_8HZ = 0x02,  // 8 Hz update
    M_100HZ = 0x06 // 100 Hz continuous magnetometer
  };

  void initMpu();
  void resetMpu();
  void terminateMpu();
  void initMagnetometer();
  void terminateMagnetometer();

  // Specify sensor full scale
  A_SCALE m_afsr;
  G_SCALE m_gfsr;
  // float m_accConversion;
  // float m_gyroConversion;
  // float m_magConversion;
  A_DLPF m_adlpf;
  G_DLPF m_gdlpf;
  // Choose either 14-bit or 16-bit magnetometer resolution
  M_SCALE m_mfsr;
  // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
  M_MODE m_mmode;
  // float m_magSens[3];

  float const GRAVITY_CONST = 9.80665f;

  // float const ACCEL_SENSITIVITY = 16384; // = 16384 LSB/g
  // float const GYRO_SENSITIVITY  = 131;   // = 131 LSB/degrees/sec

};
#endif