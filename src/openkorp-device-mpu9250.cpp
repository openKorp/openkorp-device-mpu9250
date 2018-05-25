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
#include <memory>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "openkorp-message-set.hpp"

#include "MPU9250Device.cpp"

int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("freq") ||
      0 == commandlineArguments.count("id")
    ){
    std::cerr << argv[0] << " interfaces to the MPU9250 IMU on I2C bus." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --freq=<runtime frequency> --dev=<I2C bus> --id=<id> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --freq=10 --dev=/dev/i2c-0 --id=0"  << std::endl;
    return 1;
  } 

  // Setup
  int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
  if (VERBOSE) {
    VERBOSE = std::stoi(commandlineArguments["verbose"]);
  }
  int32_t const FREQ{std::stoi(commandlineArguments["freq"])};
  std::string const devNode{commandlineArguments["dev"]};
  uint16_t const CID = std::stoi(commandlineArguments["cid"]);
  uint16_t const ID = std::stoi(commandlineArguments["id"]);
  cluon::OD4Session od4{CID};
  std::unique_ptr<MPU9250Device> imu(new MPU9250Device(devNode));

  if (VERBOSE == 2) {
    initscr();
  }
  auto atFrequency{[&imu, &ID, &VERBOSE, &od4]() -> bool
    {
      return true;
    }};

  od4.timeTrigger(FREQ, atFrequency);
  if (VERBOSE == 2) {
    endwin();     /* End curses mode      */
  }
  return 0;
}
