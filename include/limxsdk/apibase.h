/**
 * @file apibase.h
 *
 * @brief This file contains the declarations of interface related to the control of robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_INTERFACE_H_
#define _LIMX_SDK_INTERFACE_H_

#include <string>
#include <functional>
#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"

namespace limxsdk {
  /**
   * @brief Base class for the LIMX SDK API.
   *        Provides interface methods for controlling and accessing robot functionality.
   */
  class LIMX_SDK_API ApiBase {
    public:
      /**
       * @brief Pure virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set to "127.0.0.1",
       *                         while for a real robot, it may be set to "192.168.1.2".
       * @return True if init successfully, otherwise false.
       */
      virtual bool init(const std::string& robot_ip_address) { return false; }

      /**
       * @brief Pure virtual method to get the number of motors in the robot.
       * @return The total number of motors.
       */
      virtual uint32_t getMotorNumber() { return 0; };

      /**
       * @brief Pure virtual method to subscribe to robot state updates.
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      virtual void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb) { }

      /**
       * @brief Pure virtual method to publish a command to control the robot's actions.
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      virtual bool publishRobotCmd(const RobotCmd& cmd) { return false; }

      /**
       * @brief Pure virtual method to subscribe to robot commands for simulation.
       * @param cb The callback function to be invoked when a robot command is received in simulation mode.
       */
      virtual void subscribeRobotCmdForSim(std::function<void(const RobotCmdConstPtr&)> cb) { }

      /**
       * @brief Pure virtual method to publish the robot state for simulation.
       * @param state The RobotState object representing the current state of the robot in simulation.
       * @return True if the state was successfully published, otherwise false.
       */
      virtual bool publishRobotStateForSim(const RobotState& state) { return false; }

    protected:
      std::function<void(const RobotStateConstPtr&)> robot_state_callback_{nullptr};  // Callback function for handling robot state updates.
      std::function<void(const RobotCmdConstPtr&)> robot_cmd_callback_{nullptr};  // Callback function for handling robot commands in simulation mode.

  };
};

#endif