import os, sys

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import time
import numpy
import threading
import math

from Driver.DriverSerial import DriversSerial
from Driver.DriverOdometry import Odometry


class DriverAgent(object):
    def __init__(self, disable_mode: bool = False):
        super(DriverAgent, self).__init__()
        """
        Driver Version 2.1  修复版：还原原始轮速算法 + 无卡顿 + 方向正确
        """
        # hardware parameters
        self.version = 2.1
        self.driver_serial = DriversSerial(port_key='/dev/ttyS6')
        self.wheel_radius = 0.09  # wheel radius (m)
        self.wheel_dis = 0.65  # wheel distance (m)

        # control parameters
        self.speed = 0  # linear speed (m/s)
        self.omega = 0  # angular speed (rad/s)
        self.radius = 0  # turning radius (m)
        self._left_spd = 0  # left wheel speed (m/s)
        self._right_spd = 0  # right wheel speed (m/s)

        # ===================== 防卡顿核心：缓存上一次转速 =====================
        self.last_left_rpm = 0  # 上一次左轮转速
        self.last_right_rpm = 0  # 上一次右轮转速
        self.RPM_THRESHOLD = 3  # 转速变化≥3rpm才更新
        # ====================================================================

        # initialize the odometry
        self.odo = Odometry()

        # initialize the motor drivers
        self.enable_driver(enable=True)

        self.disable_mode = disable_mode

        # main thread for control
        self.thread_control = threading.Thread(target=self.main_control, args=())
        self.thread_control.daemon = True  # 守护线程，程序退出自动关闭
        self.thread_control.start()

    def __version__(self):
        print("Driver version:", self.version)

    def update_control_params(self, speed, omega, radius):
        """
        update the control parameters
        """
        self.speed = speed
        self.omega = omega
        self.radius = radius

    # ===================== 【完全还原】你原始的轮速计算函数，无任何修改 =====================
    def _get_wheels_speed(self):
        """
        combine the linear speed and angular speed to the motor speed
        based on the wheel radius
        :return:
        """
        # first put the linear speed to both wheels
        self._left_spd = self.speed
        self._right_spd = self.speed

        # then put the angular speed to the wheels, based on the wheel distance
        if self.omega >= 0:
            # turning right
            left_angular_spd = self.omega * (self.radius + self.wheel_dis / 2)
            right_angular_spd = self.omega * (self.radius - self.wheel_dis / 2)
        else:
            # turning left
            left_angular_spd = self.omega * (self.radius - self.wheel_dis / 2)
            right_angular_spd = self.omega * (self.radius + self.wheel_dis / 2)

        self._left_spd += left_angular_spd
        self._right_spd += right_angular_spd

        return self._left_spd, self._right_spd

    # ====================================================================================

    def _set_wheel_rpm(self):
        """
        【对标旧版本代码】完全复刻旧代码符号规则
        1. 直行：正常
        2. omega>0：右转
        3. omega<0：左转（和旧代码一致）
        """
        wheel_circumference = 2 * math.pi * self.wheel_radius

        # 1. 基础转速计算
        left_rpm = int(self._left_spd / wheel_circumference * 60)
        right_rpm = int(self._right_spd / wheel_circumference * 60)

        # ===================== 【复刻旧代码核心】omega<0 轮速取反 =====================
        if self.omega <= 0:
            left_rpm = -left_rpm
            right_rpm = -right_rpm
        # ============================================================================

        # ===================== 【旧代码硬件适配】右轮取反 =====================
        right_rpm = -right_rpm
        # ====================================================================

        # 防卡顿写入（不变）
        if abs(left_rpm - self.last_left_rpm) >= self.RPM_THRESHOLD:
            self.driver_serial.set_single_driver_speed(rpm=left_rpm, motor='left')
            self.last_left_rpm = left_rpm
        if abs(right_rpm - self.last_right_rpm) >= self.RPM_THRESHOLD:
            self.driver_serial.set_single_driver_speed(rpm=right_rpm, motor='right')
            self.last_right_rpm = right_rpm

    def enable_driver(self, enable: bool = False):
        """
        disable both drivers
        :return:
        """
        self.driver_serial.set_motor_enable(enable=enable, motor='left')
        self.driver_serial.set_motor_enable(enable=enable, motor='right')

    def main_control(self):
        """
        Loop control the driver
        :return:
        """
        while True:
            try:
                if self.disable_mode:
                    # in record mode, deactivate the driver
                    self.enable_driver(False)
                    time.sleep(0.2)
                    continue

                # update the wheel speed
                self._get_wheels_speed()
                # set the wheel rpm（防卡顿写入）
                self._set_wheel_rpm()

                # update the odometry
                left_pos, right_pos = self.driver_serial.get_driver_position()
                self.odo.update_pose(left_pos, right_pos)
                # time delay for control loop
                time.sleep(0.2)
            except Exception as e:
                print(f"控制循环异常: {e}")
                time.sleep(0.2)


if __name__ == "__main__":
    driver_ins = DriverAgent(disable_mode=False)
    driver_ins.__version__()
    driver_ins.update_control_params(speed=0.2, omega=0, radius=0)
    time.sleep(2)
    # driver_ins.update_control_params(speed=-0.2, omega=0, radius=0)
    # time.sleep(2)
    # driver_ins.update_control_params(speed=0, omega=0.3, radius=0.8)
    # time.sleep(5)
    # driver_ins.update_control_params(speed=0, omega=-0.3, radius=0.8)
    # time.sleep(5)
    # driver_ins.update_control_params(speed=0, omega=0.0, radius=0)
    # time.sleep(2)
    driver_ins.enable_driver(False)
