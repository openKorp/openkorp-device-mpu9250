/*
 * Copyright (C) 2019 Björnborg
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

// #include <stdio.h>
// #include <getopt.h>
// #include <signal.h>
// #include <stdlib.h> // for atoi() and exit()
#include <rc/mpu.h>
#include <rc/time.h>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "openkorp-message-set.hpp"

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21

static rc_mpu_data_t data;

int32_t main(int32_t argc, char** argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")
      // 0 == commandlineArguments.count("input") ||
      // 0 == commandlineArguments.count("freq")
  ) {
    std::cerr << argv[0] << " interfaces to the mpu9250 imu sensor."
              << std::endl;
    std::cerr << "Usage:   " << argv[0]
              << " --cid=<conference id> [--verbose=<[0-9]>]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --verbose=2"
              << std::endl;
    return 1;
  }

  // start with default config and modify based on options
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;

  // parse arguments
  // opterr = 0;
  // sample_rate = atoi(optarg);
  // if(sample_rate>200 || sample_rate<4){
  // 	printf("sample_rate must be between 4 & 200");
  // 	return -1;
  // }
  conf.dmp_sample_rate = 200;
  // priority = atoi(optarg);
  // conf.dmp_interrupt_priority = priority;
  // conf.dmp_interrupt_sched_policy = SCHED_FIFO;
  // enable_mag = 1;
  conf.enable_magnetometer = 1;
  // show_compass = 1;
  // show_accel = 1;
  // show_gyro = 1;
  conf.dmp_fetch_accel_gyro = 1;
  // show_quat = 1;
  // show_tb = 1;
  // show_temp = 1;
  // set signal handler so the loop can exit cleanly
  // signal(SIGINT, __signal_handler);
  // running = 1;

  // now set up the imu for dmp interrupt operation
  if (rc_mpu_initialize_dmp(&data, conf)) {
    printf("rc_mpu_initialize_failed\n");
    return -1;
  }
  // write labels for what data will be printed and associate the interrupt
  // function to print data immediately after the header.
  // __print_header();
  uint16_t const CID = std::stoi(commandlineArguments["cid"]);
  cluon::OD4Session od4{CID};

  openkorp::logic::Quaternion quaternionMsg;
  auto SendQuaternionMsg{[&]() mutable -> void {
    quaternionMsg.w(static_cast<float>(data.fused_quat[QUAT_W]))
        .x(static_cast<float>(data.fused_quat[QUAT_X]))
        .y(static_cast<float>(data.fused_quat[QUAT_Y]))
        .z(static_cast<float>(data.fused_quat[QUAT_Z]));
    od4.send(quaternionMsg);
  }};
  // Work around for lambda functions in legacy C programs
  // https://bannalia.blogspot.com/2016/07/passing-capturing-c-lambda-functions-as.html
  auto HelperCbFunc = [](void* arg) {  // note thunk is captureless
    (*static_cast<decltype(SendQuaternionMsg)*>(arg))();
  };

  rc_mpu_set_dmp_callback(HelperCbFunc, &SendQuaternionMsg);
  // now just wait, print_data() will be called by the interrupt

  while (1) {
    rc_usleep(100000);
  }

  // shut things down
  rc_mpu_power_off();
  // printf("\n");
  // fflush(stdout);
  return 0;
}
