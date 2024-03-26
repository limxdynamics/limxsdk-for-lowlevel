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
