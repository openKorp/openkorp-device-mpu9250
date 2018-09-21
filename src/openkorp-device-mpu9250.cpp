/*
 * Copyright (C) 2018 Björnborg Ngúyen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ncurses.h>
#include <sstream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "openkorp-message-set.hpp"

#include "MPU9250Device.hpp"

int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("freq") ||
      0 == commandlineArguments.count("id")){
    std::cerr << argv[0] << " interfaces to the MPU9250 IMU on I2C bus." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --freq=<runtime frequency> --dev=<I2C bus> --id=<id> [--verbose] [--calibrate]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --freq=10 --dev=/dev/i2c-0 --id=0 --verbose=1"  << std::endl;
    return 1;
  } 

  // Setup
  int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
  if (VERBOSE) {
    VERBOSE = std::stoi(commandlineArguments["verbose"]);
  }
  bool CALIBRATE{commandlineArguments.count("calibrate") != 0};
  int32_t const FREQ{std::stoi(commandlineArguments["freq"])};
  std::string const devNode{commandlineArguments["dev"]};
  uint16_t const CID = std::stoi(commandlineArguments["cid"]);
  uint16_t const ID = std::stoi(commandlineArguments["id"]);
  cluon::OD4Session od4{CID};
  std::unique_ptr<MPU9250Device> imu(new MPU9250Device(devNode, CALIBRATE));

  if (VERBOSE == 2) {
    initscr();
  }
  auto atFrequency{[&imu, &ID, &VERBOSE, &od4]() -> bool
    {
      opendlv::proxy::AccelerationReading accreading = imu->readAccelerometer();
      opendlv::proxy::AngularVelocityReading gyroreading = imu->readGyroscope();
      opendlv::proxy::TemperatureReading tempreading = imu->readThermometer();
      opendlv::proxy::MagneticFieldReading magreading = imu->readMagnetometer();
      opendlv::proxy::PressureReading altreading = imu->readAltimeter();

      std::stringstream ss;
      ss << "Acceleration: " << accreading.accelerationX() << ", \t" << accreading.accelerationY() << ", \t" << accreading.accelerationZ() << "\n";
      ss << "Magnetfield: " << magreading.magneticFieldX() << ", \t" << magreading.magneticFieldY() << ", \t" << magreading.magneticFieldZ() << "\n";
      ss << "Gyroscope: " << gyroreading.angularVelocityX() << ", \t" << gyroreading.angularVelocityY() << ", \t" << gyroreading.angularVelocityZ() << "\n";
      ss << "TemperatureReading: " << tempreading.temperature() << "\n";
      ss << "Pressure: " << altreading.pressure() << "\n";
      if (VERBOSE == 1) {
        std::cout << ss.str() << std::endl;
      } else if (VERBOSE == 2) {
        mvprintw(1,1, ss.str().c_str());
        refresh();
      }
      return true;
    }};

  od4.timeTrigger(FREQ, atFrequency);
  if (VERBOSE == 2) {
    endwin();     /* End curses mode      */
  }
  return 0;
}
