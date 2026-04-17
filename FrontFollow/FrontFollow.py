import numpy as np
import torch
import time
import threading
import sklearn
import os
import sys
import signal  # 新增：捕获程序退出信号

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

from Driver.DriverAgent import DriverAgent
from Sensors import Cameras, Softskin, LiDAR_YDLIDAR, Button


class FFL(object):
    def __init__(self):
        """
        Front Following
        """
        super(FFL, self).__init__()

        # modules(sensors, drivers)
        self.driver = DriverAgent()
        # self.camera = Cameras.DualCamera()
        self.LiDAR = LiDAR_YDLIDAR.LiDAR_YDLIDAR()
        self.softskin = Softskin.SoftSkin()
        self.button = Button.Button()

        # speed parameters
        self.f_spd = 0.3  # forward speed(m/s)
        self.b_spd = -0.3  # backward speed(m/s)
        self.t_spd = 0.3  # turning speed(m/s) # maximum turning speed for counting the omega

        self.spd_change_ratio = 0.8  # speed change ratio

        # leg data
        self.left_leg = np.zeros((2,))
        self.right_leg = np.zeros((2,))
        self.human_center = np.zeros((2,))

        # conditioning parameters
        self.forward_boundary = 150  # if the legs are further than this, then should go forward
        self.backward_boundary = -50  # likewise, go backward
        self.center_left_boundary = 20  # like wise, turn left while moving forward, the center means this is for the center point of user
        self.center_right_boundary = -20  # like wise, turn right
        self.left_boundary = 90
        self.right_boundary = -70
        self.left_max_boundary = 140  # left max value
        self.right_max_boundary = -140  # right max value

        # thread event
        self.FFLevent = threading.Event()
        self.FFLevent.clear()

        self.FFLthread = threading.Thread(target=self.main, args=())
        self.FFLthread.daemon = True
        self.FFLthread.start()

    # 新增：安全停止函数，电机失能
    def stop_safely(self):
        print("\n程序退出，电机失能...")
        self.update_driver(0, 0, 0)
        self.driver.enable_driver(False)

    def update_driver(self, speed: float = 0, omega: float = 0, radius: float = 0):

        current_speed, current_radius, current_omega = self.driver.speed, self.driver.radius, self.driver.omega
        target_speed, target_radius, target_omega = speed, radius, omega

        actual_speed = current_speed + self.spd_change_ratio * (target_speed - current_speed)
        actual_radius = current_radius + self.spd_change_ratio * (target_radius - current_radius)
        actual_omega = current_omega + self.spd_change_ratio * (target_omega - current_omega)

        self.driver.speed = actual_speed
        self.driver.radius = actual_radius
        self.driver.omega = actual_omega

    def main(self):
        while True:
            # if self.softskin.is_abnormal:
            #     print("emergency stop due to abnormal softskin force")
            #     self.FFLevent.clear()
            #     self.update_driver(speed=0, omega=0, radius=0)
            self.FFLevent.wait()
            leg_data = self.LiDAR.get_leg_data()
            if leg_data is not None:
                self.left_leg = leg_data[0]
                self.right_leg = leg_data[1]

                # 优化：先判断无腿，再计算中心，避免无效数据运算
                if self.left_leg[0] < -1500 or self.right_leg[0] < -1500:
                    print("no leg detected, stop")
                    self.update_driver(speed=0, omega=0, radius=0)
                    time.sleep(0.1)
                    continue

                self.human_center = (self.left_leg + self.right_leg) / 2
                print("left leg:", self.left_leg, "\tright leg:", self.right_leg, "\thuman center:", self.human_center)

                # conditioning
                if self.human_center[0] > self.forward_boundary:
                    if self.human_center[1] > self.center_left_boundary:
                        # turn left
                        if self.LiDAR.ob_front > 0 or self.LiDAR.ob_front_left > 0:
                            # obstacle
                            print("go left but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            print("go left")
                            radius = max(0.5, 0.3 + abs(0.5 * (self.left_max_boundary - self.left_leg[0]) / (
                                    self.left_max_boundary - self.left_boundary)))
                            omega = -self.t_spd / radius
                            print(f"radius:{radius:.3f}, omega:{omega:.3f}")
                            self.update_driver(speed=0, omega=omega, radius=radius)
                    elif self.human_center[1] < self.center_right_boundary:
                        # turn right
                        if self.LiDAR.ob_front > 0 or self.LiDAR.ob_front_right > 0:
                            # obstacle
                            print("go right but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            print("go right")
                            radius = max(0.5, 0.3 + abs(0.5 * (self.right_leg[0] - self.right_max_boundary) / (
                                    self.right_boundary - self.right_max_boundary)))
                            omega = self.t_spd / radius
                            print(f"radius:{radius:.3f}, omega:{omega:.3f}")
                            self.update_driver(speed=0, omega=omega, radius=radius)
                    else:
                        if self.LiDAR.ob_front > 0:
                            # obstacle
                            print("go front but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            # go straight
                            print("go forward")
                            self.update_driver(speed=self.f_spd, omega=0, radius=0)
                elif self.human_center[0] < self.backward_boundary:
                    if self.LiDAR.ob_back > 0:
                        # obstacle
                        print("go back but obstacle")
                        self.update_driver(speed=0, omega=0, radius=0)
                    else:
                        # go backward
                        print("go backward")
                        self.update_driver(speed=self.b_spd, omega=0, radius=0)
                else:
                    # stop
                    print("stop")
                    self.update_driver(speed=0, omega=0, radius=0)

            time.sleep(0.1)


if __name__ == "__main__":
    ffl = FFL()


    # 新增：捕获Ctrl+C退出信号，自动电机失能
    def exit_handler(signum, frame):
        ffl.stop_safely()
        sys.exit(0)


    signal.signal(signal.SIGINT, exit_handler)

    time.sleep(1)
    ffl.FFLevent.set()

    # 保持主程序运行
    while True:
        time.sleep(1)
