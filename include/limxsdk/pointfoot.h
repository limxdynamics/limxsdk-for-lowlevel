/**
 * @file pointfoot.h
 *
 * @brief This file contains the declarations of classes related to the control of pointfoot robots.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_POINTFOOT_H_
#define _LIMX_SDK_POINTFOOT_H_

#include "limxsdk/macros.h"
#include "limxsdk/datatypes.h"
#include "limxsdk/apibase.h"

namespace limxsdk {
  /**
   * @brief Class for controlling a pointfoot robot using the LIMX SDK API.
   */
  class LIMX_SDK_API PointFoot : public ApiBase {
    public:
      /**
       * @brief Get an instance of the PointFoot class.
       * @return A pointer to a PointFoot instance (Singleton pattern).
       */
      static PointFoot* getInstance();

      /**
       * @brief Pure virtual initialization method.
       *        This method should specify the operations to be performed before using the object in the main function.
       * @param robot_ip_address The IP address of the robot.
       *                         For simulation, it is typically set to "127.0.0.1",
       *                         while for a real robot, it may be set to "192.168.1.2".
       * @return True if init successfully, otherwise false.
       */
      bool init(const std::string& robot_ip_address = "127.0.0.1") override;

      /**
       * @brief Get the number of motors in the robot.
       * @return The total number of motors.
       */
      uint32_t getMotorNumber() override;

      /**
       * @brief Subscribe to receive updates about the robot state.
       * The motor order for the state data is as follows:
       *        0: abad_L_joint,  1: hip_L_joint,  2: knee_L_joint
       *        3: abad_R_joint,  4: hip_R_joint,  5: knee_R_joint
       * 
       * @param cb The callback function to be invoked when a robot state update is received.
       */
      void subscribeRobotState(std::function<void(const RobotStateConstPtr&)> cb) override;

      /**
       * @brief Publish a command to control the robot's actions.
       * The motor order for the commnd data is as follows:
       *        0: abad_L_joint,  1: hip_L_joint,  2: knee_L_joint
       *        3: abad_R_joint,  4: hip_R_joint,  5: knee_R_joint
       * 
       * @param cmd The RobotCmd object representing the desired robot command.
       * @return True if the command was successfully published, otherwise false.
       */
      bool publishRobotCmd(const RobotCmd& cmd) override;

      /**
       * @brief Destructor for the PointFoot class.
       *        Cleans up any resources used by the object.
       */
      virtual ~PointFoot();

    private:
      /**
       * @brief Private constructor to prevent external instantiation of the PointFoot class.
       */
      PointFoot();
  };
};

#endif