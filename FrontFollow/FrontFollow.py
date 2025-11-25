import numpy as np
import torch
import time
import threading
import sklearn
import os,sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

from Driver.Driver_v2 import Driver
from Sensors import Cameras, Arduino, Softskin, LiDAR_YDLIDAR

class FFL(object):
    def __init__(self):
        """
        Front Following
        """
        super(FFL, self).__init__()
        self.driver = Driver()
        # self.camera = Cameras.DualCamera()
        self.LiDAR = LiDAR_YDLIDAR.LiDAR_YDLIDAR()
        self.softskin = Softskin.SoftSkin()

        # control parameters
        self.f_spd = 0.1  # forward speed(m/s)
        self.b_spd = -0.1  # backward speed(m/s)
        self.t_spd = 0.2  # turning speed(m/s)

        self.omega_l = -0.4  # left omega (rad/s) turning on the spot
        self.omega_r = 0.4  # right omega (rad/s) turning on the spot

        self.spd_change_ratio = 0.8  # speed change ratio

        # leg data
        self.left_leg = np.zeros((2,))
        self.right_leg = np.zeros((2,))
        self.human_center = np.zeros((2,))

        # conditioning parameters
        self.forward_boundary = 15  # if the legs are further than this, then should go forward
        self.backward_boundary = -5  # likewise, go backward
        self.center_left_boundary = 2  # like wise, turn left while moving forward, the center means this is for the center point of user
        self.center_right_boundary = 1  # like wise, turn right
        self.left_boundary = 9
        self.right_boundary = -7
        self.left_max_boundary = 14  # left max value
        self.right_max_boundary = -14  # right max value

        # thread event
        self.FFLevent = threading.Event()
        self.FFLevent.clear()

        self.FFLthread = threading.Thread(target=self.main, args=())
        self.FFLthread.start()

    def update_driver(self, speed: float = 0, omega: float = 0, radius: int = 0):

        current_speed, current_radius, current_omega = self.driver.speed, self.driver.radius, self.driver.omega
        target_speed, target_radius, target_omega = speed, radius, omega

        actual_speed = target_speed + self.spd_change_ratio * (current_speed - target_speed)
        actual_radius = target_radius + self.spd_change_ratio * (current_radius - target_radius)
        actual_omega = target_omega + self.spd_change_ratio * (current_omega - target_omega)

        self.driver.speed = actual_speed
        self.driver.radius = actual_radius
        self.driver.omega = actual_omega

    def main(self):
        while True:
            leg_data = self.LiDAR.get_leg_data()
            if leg_data is not None:
                self.left_leg = leg_data[0]
                self.right_leg = leg_data[1]
                self.human_center = (self.left_leg + self.right_leg)/2

                # conditioning
                if self.human_center[1] > self.forward_boundary:
                    if self.human_center[0] > self.center_left_boundary:
                        # turn left
                        print("go left")
                        # self.update_driver(speed=self.f_spd, omega=self.omega_l, radius=0)
                    elif self.human_center[0] < self.center_right_boundary:
                        # turn right
                        print("go right")
                        # self.update_driver(speed=self.f_spd, omega=self.omega_r, radius=0)
                    else:
                        # go straight
                        print("go forward")
                        # self.update_driver(speed=self.f_spd, omega=0, radius=0)
                elif self.human_center[1] < self.backward_boundary:
                    # go backward
                    print("go backward")
                    # self.update_driver(speed=self.b_spd, omega=0, radius=0)
                else:
                    # stop
                    print("stop")
                    # self.update_driver(speed=0, omega=0, radius=0)
            else:
                # no leg detected, stop
                print("no leg detected, stop")
                # self.update_driver(speed=0, omega=0, radius=0)

            time.sleep(0.1)



if __name__ == "__main__":
    ffl = FFL()
    time.sleep(1)
    ffl.FFLevent.set()
