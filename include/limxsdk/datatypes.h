/**
 * @file datatypes.h
 *
 * @brief This file contains the declarations of classes and structures related to robotics.
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#ifndef _LIMX_SDK_DATATYPES_H_
#define _LIMX_SDK_DATATYPES_H_

#include <stdint.h>
#include <vector>
#include <memory>

namespace limxsdk {
  /**
   * @struct RobotState
   *
   * @brief Structure representing the state of a robot based on sensor feedback.
   *
   * This structure encapsulates various data points that could be used to monitor and control a robot, including IMU data (accelerometer, gyroscope, quaternion), output torque, current angles, and velocities.
   */
  struct RobotState {
    RobotState() { }
    RobotState(int motor_num)
    : tau(motor_num, 0.0)
    , q(motor_num, 0.0)
    , dq(motor_num, 0.0) { }
    uint64_t stamp;             // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    float imu_acc[3];           // Array to store IMU (Inertial Measurement Unit) accelerometer data for tracking linear acceleration along three axes
    float imu_gyro[3];          // Array to store IMU gyroscope data for tracking angular velocity or rotational speed
    float imu_quat[4];          // Array to store IMU quaternion data, represents orientation in 3D space
    std::vector<float> tau;     // Vector to store the current estimated output torque (in Newton meters)
    std::vector<float> q;       // Vector to store the current angles (in radians)
    std::vector<float> dq;      // Vector to store the current velocities (in radians per second)
  };
  typedef std::shared_ptr<RobotState> RobotStatePtr;
  typedef std::shared_ptr<RobotState const> RobotStateConstPtr;

  /**
   * @struct RobotCmd
   *
   * @brief Structure representing the command for controlling a robot.
   *
   * This structure holds various commands that can be used to control a robot, including the desired working mode, desired angles, desired velocities, desired output torque, desired position stiffness, and desired velocity stiffness.
   */
  struct RobotCmd {
    RobotCmd() { }
    RobotCmd(int motor_num)
    : mode(motor_num, 0)
    , q(motor_num, 0.0)
    , dq(motor_num, 0.0)
    , tau(motor_num, 0.0)
    , Kp(motor_num, 0.0)
    , Kd(motor_num, 0.0) { }
    uint64_t stamp;             // Timestamp in nanoseconds, typically represents the time when this data was recorded or generated.
    std::vector<uint8_t> mode;  // The desired working mode of the robot.
    std::vector<float> q;       // Vector storing the desired angles (in radians).
    std::vector<float> dq;      // Vector storing the desired velocities (in radians per second).
    std::vector<float> tau;     // Vector storing the desired output torque (in Newton meters).
    std::vector<float> Kp;      // Vector storing the desired position stiffness (in Newton meters per radian).
    std::vector<float> Kd;      // Vector storing the desired velocity stiffness (in Newton meters per radian per second).
  };     
  typedef std::shared_ptr<RobotCmd> RobotCmdPtr;
  typedef std::shared_ptr<RobotCmd const> RobotCmdConstPtr;                
};

#endif