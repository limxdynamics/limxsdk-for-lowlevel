/**
 * @file wheel legged template
 *
 * @brief This file contains the wheel_legged_template of robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <iostream>
#include "limxsdk/wheel_legged.h"

class WheelLeggedTemplate {
public:
  WheelLeggedTemplate() {
    // Get an instance of the WheelLegged class
    wl_ = limxsdk::WheelLegged::getInstance();

    // Subscribe to robot state updates and bind the callback function
    wl_->subscribeRobotState(std::bind(&WheelLeggedTemplate::robotStateCallback, this, std::placeholders::_1));
  }

  void publish(const limxsdk::RobotCmd& msg) {
    // Publish the robot command message
    wl_->publishRobotCmd(msg);
  }
    
private:
  void robotStateCallback(const limxsdk::RobotStateConstPtr& msg) {
    // Print received imu.aac[0] value from the robot state message
    std::cout << "Receive imu.aac[0]: " << msg->imu_acc[0] << std::endl;
  }

  limxsdk::WheelLegged* wl_;
};

int main() {
  // Get an instance of the WheelLegged class
  limxsdk::WheelLegged* wl = limxsdk::WheelLegged::getInstance();

  // Initialize the WheelLegged instance with a robot IP address
  // * For simulation, it is typically set   to "127.0.0.1",
  // * while for a real robot, it may be set to "192.168.1.2"
  if (!wl->init("192.168.1.2")) {
    exit(1);
  }

  // Create an instance of the WheelLeggedTemplate class
  WheelLeggedTemplate temp;

  while (true) {
    // Sleep for 1 second
#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
  }
}
