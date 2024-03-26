# Limx SDK Python 接口介绍



## 1、PYTHONPATH 环境设置

- Linux环境

  确保将 `/path/to/directory_containing_limxsdk` 替换为 `limxsdk` 包所在的实际路径

  ```
  export PYTHONPATH="/path/to/directory_containing_limxsdk:$PYTHONPATH"
  ```

- Windwos环境

  确保将 `D:\path\to\directory_containing_limxsdk` 替换为 `limxsdk` 包所在的实际路径

  ```
  set PYTHONPATH=D:\path\to\directory_containing_limxsdk;%PYTHONPATH%
  ```

  

## 2、Robot.py 接口介绍

```
"""
@brief This file contains the declarations of classes related to the control of robots.

@file Robot.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
from typing import Callable, Any
import limxsdk.datatypes as datatypes
import limxsdk.robot as robot

class Robot(object):
    """
    @class Robot
    @brief Represents a robot with various functionalities.

    This class provides an interface to interact with different types of robots.
    """

    def __init__(self, robot_type: robot.RobotType):
        """
        @brief Initializes a Robot object with a specified type.

        @param robot_type: Type of robot, either RobotType.PointFoot or RobotType.WheelLegged.
        """
        __slots__ = ['robot', 'robot_ip']

        # Default robot IP address
        self.robot_ip = '127.0.0.1'

        # Create a native robot instance based on the specified type
        if robot_type == robot.RobotType.PointFoot:
            self.robot = robot.RobotNative("PointFoot")
        elif robot_type == robot.RobotType.WheelLegged:
            self.robot = robot.RobotNative("WheelLegged")

    def init(self, robot_ip: str):
        """
        @brief Initializes the robot with a specified IP address.

        @param robot_ip: IP address of the robot.
        @return: True if initialization is successful, False otherwise.
        """
        self.robot_ip = robot_ip
        return self.robot.init(self.robot_ip)

    def getMotorNumber(self):
        """
        @brief Gets the number of motors of the robot.

        @return: Number of motors.
        """
        return self.robot.getMotorNumber()

    def subscribeRobotState(self, callback: Callable[[datatypes.RobotState], Any]):
        """
        @brief Subscribes to receive updates on the robot's state.

        @param callback: Callable[[datatypes.RobotState], Any]: 
                  Callback function to handle robot state updates.
        @return: Subscription status.
        """
        return self.robot.subscribeRobotState(callback)

    def publishRobotCmd(self, cmd: datatypes.RobotCmd):
        """
        @brief Publishes a robot command.

        @param cmd: Robot command to be published.
        @return: Status of the command publication.
        """
        return self.robot.publishRobotCmd(cmd)

    def getJointOffset(self, timeout: float = -1.0):
        """
        @brief Gets the joint offset of the robot.

        @param timeout: Timeout for getting joint offset (-1 for infinite waiting time).
        @return: Joint offset.
        """
        return self.robot.getJointOffset(timeout)

    def getJointLimit(self, timeout: float = -1.0):
        """
        @brief Gets the joint limit of the robot.

        @param timeout: Timeout for getting joint limit (-1 for infinite waiting time).
        @return: Joint limit.
        """
        return self.robot.getJointLimit(timeout)

    def subscribeSensorJoy(self, callback: Callable[[datatypes.SensorJoy], Any]):
        """
        @brief Subscribes to receive sensor joy updates.

        @param callback: Callable[[datatypes.SensorJoy], Any]: 
                  Callback function to handle sensor joy updates.
        @return: Subscription status.
        """
        return self.robot.subscribeSensorJoy(callback)

    def subscribeDiagnosticValue(self, callback: Callable[[datatypes.DiagnosticValue], Any]):
        """
        @brief Subscribes to receive diagnostic value updates.

        @param callback (Callable[[datatypes.DiagnosticValue], Any]): 
                  Callback function to handle diagnostic value updates.
        @return: Subscription status.
        """
        return self.robot.subscribeDiagnosticValue(callback)

```

## 3、示例

```
"""
@file example.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import sys
import time
from functools import partial
import limxsdk.robot.Rate as Rate
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

class RobotReceiver:
    # Callback function for receiving robot state
    def robotStateCallback(self, robot_state: datatypes.RobotState):
        print("\n------\nrobot_state:" + \
              "\n  stamp: " + str(robot_state.stamp) + \
              "\n  imu_acc: " + str(robot_state.imu_acc) + \
              "\n  imu_gyro: " + str(robot_state.imu_gyro) + \
              "\n  imu_quat: " + str(robot_state.imu_quat) + \
              "\n  tau: " + str(robot_state.tau) + \
              "\n  q: " + str(robot_state.q) + \
              "\n  dq: " + str(robot_state.dq))

    # Callback function for receiving sensor joy data
    def sensorJoyCallback(self, sensor_joy: datatypes.SensorJoy):
        print("\n------\nsensor_joy:" + \
              "\n  stamp: " + str(sensor_joy.stamp) + \
              "\n  axes: " + str(sensor_joy.axes) + \
              "\n  buttons: " + str(sensor_joy.buttons))

    # Callback function for receiving diagnostic value
    def diagnosticValueCallback(self, diagnostic_value: datatypes.DiagnosticValue):
        print("\n------\ndiagnostic_value:" + \
              "\n  stamp: " + str(diagnostic_value.stamp) + \
              "\n  name: " + diagnostic_value.name + \
              "\n  level: " + str(diagnostic_value.level) + \
              "\n  code: " + str(diagnostic_value.code) + \
              "\n  message: " + diagnostic_value.message)

if __name__ == '__main__':
    # Create a Robot instance of type PointFoot
    robot = Robot(RobotType.PointFoot)
    # Initialize the robot with IP address "127.0.0.1"
    robot.init("127.0.0.1")

    # Get joint offset, joint limit, and motor number information
    joint_offset = robot.getJointOffset()
    joint_limit = robot.getJointLimit()
    motor_number = robot.getMotorNumber()

    # Create an instance of RobotReceiver to handle callbacks
    receiver = RobotReceiver()

    # Create partial functions for callbacks
    robotStateCallback = partial(receiver.robotStateCallback)
    sensorJoyCallback = partial(receiver.sensorJoyCallback)
    diagnosticValueCallback = partial(receiver.diagnosticValueCallback)

    # Subscribe to robot state, sensor joy, and diagnostic value topics
    robot.subscribeRobotState(robotStateCallback)
    robot.subscribeSensorJoy(sensorJoyCallback)
    robot.subscribeDiagnosticValue(diagnosticValueCallback)
    
    # Main loop to continuously publish robot commands
    rate = Rate(1000) # 1000 Hz
    while True:
        cmd_msg = datatypes.RobotCmd()
        cmd_msg.stamp = time.time_ns()  # Set the timestamp
        # Set default values for control mode, joint positions, velocities, torques, Kp, and Kd
        cmd_msg.mode = [1.0 for _ in range(motor_number)]
        cmd_msg.q = [1.0 for _ in range(motor_number)]
        cmd_msg.dq = [1.0 for _ in range(motor_number)]
        cmd_msg.tau = [1.0 for _ in range(motor_number)]
        cmd_msg.Kp = [1.0 for _ in range(motor_number)]
        cmd_msg.Kd = [1.0 for _ in range(motor_number)]
        robot.publishRobotCmd(cmd_msg)  # Publish the robot command
        rate.sleep()  # Control loop frequency

```

