import os,sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import time
import numpy
import threading

from Driver.DriverSerial import DriversSerial
from Driver.DriverOdometry import Odometry


class DriverAgent(object):
    def __init__(self, disable_mode: bool = False):
        super(DriverAgent, self).__init__()
        """
        Driver Version 2.1. Using pymodbus (version > 3.0)
        Two motors connected to the same serial port with different device address.
        """
        # hardware parameters
        self.version = 2.1  # driver version
        self.driver_serial = DriversSerial(port_key='/dev/ttyS6')   # serial port for both drivers
        self.wheel_radius = 0.18 # wheel radius (m)
        self.wheel_dis = 0.65 # wheel distance (m)

        # control parameters
        # driver parameters
        self.speed = 0  # linear speed (m/s)
        self.omega = 0  # angular speed (rad/s)
        self.radius = 0  # turning radius (m)
        # wheel parameters
        self._left_spd = 0  # left wheel speed (m/s)
        self._right_spd = 0  # right wheel speed (m/s)

        # initialize the odometry
        self.odo = Odometry()

        # initialize the motor drivers
        self.enable_driver(enable=True)

        # record mode

        self.disable_mode = disable_mode

        # main thread for control
        self.thread_control = threading.Thread(target=self.main_control, args=())
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
            left_angular_spd = self.omega * (self.radius + self.wheel_dis/2)
            right_angular_spd = self.omega * (self.radius - self.wheel_dis/2)
        else:
            # turning left
            left_angular_spd = self.omega * (self.radius - self.wheel_dis/2)
            right_angular_spd = self.omega * (self.radius + self.wheel_dis/2)

        self._left_spd += left_angular_spd
        self._right_spd += right_angular_spd

        return self._left_spd, self._right_spd

    def _set_wheel_rpm(self):
        """
        set the wheel speed to the driver
        :return:
        """
        left_rpm = int(self._left_spd / (2 * 3.14 * self.wheel_radius) * 60)
        self.driver_serial.set_single_driver_speed(rpm=left_rpm, motor='left')
        right_rpm = int(self._right_spd / (2 * 3.14 * self.wheel_radius) * 60)
        self.driver_serial.set_single_driver_speed(rpm=right_rpm, motor='right')
        print(left_rpm, right_rpm)

    def enable_driver(self, enable: bool = True):
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
            if self.disable_mode:
                # in record mode, deactivate the driver
                self.enable_driver(False)
            else:
                # update the wheel speed
                self._get_wheels_speed()
                # set the wheel rpm
                self._set_wheel_rpm()
            # update the odometry
            left_pos, right_pos = self.driver_serial.get_driver_position()
            self.odo.update_pose(left_pos, right_pos)
            # time delay for control loop
            time.sleep(0.1)


if __name__ == "__main__":
    driver_ins = DriverAgent(disable_mode=False)
    driver_ins.__version__()
    driver_ins.update_control_params(speed=0.5, omega=0,radius=0)
    time.sleep(5)
    driver_ins.update_control_params(speed=0, omega=0.0, radius=0)
    time.sleep(2)
    driver_ins.enable_driver()