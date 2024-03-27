import sys

class RobotState(object):
    __slots__ = ['stamp','imu_acc','imu_gyro','imu_quat','tau','q','dq']
    def __init__(self):
        self.stamp = 0
        self.imu_acc = [0. for x in range(0, 3)]
        self.imu_gyro = [0. for x in range(0, 3)]
        self.imu_quat = [0. for x in range(0, 4)]
        self.tau = []
        self.q = []
        self.dq = []