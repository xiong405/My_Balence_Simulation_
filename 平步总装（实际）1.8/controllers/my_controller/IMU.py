import math


class IMU_yaw(object):
    def __init__(self):
        self.yawRound = 0
        self.lastYaw = 0
    def round_yaw(self,yaw):
        if (yaw - self.lastYaw)>math.pi:
            self.yawRound=self.yawRound-1
        elif (yaw - self.lastYaw) < -math.pi:
            self.yawRound = self.yawRound + 1
        self.lastYaw = yaw
        yaw = yaw + self.yawRound * 2 * math.pi
        return yaw