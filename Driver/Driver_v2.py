import serial
import time
from DriverSerial import SingleDriverSerial
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *
from DriverOdometry import Odometry

class Driver(object):
    def __init__(self):
        """
        A class to handle serial communication with a motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(Driver, self).__init__()
        # speed part
        self.speed = 0 # linear speed (m/s)
        self.omega = 0 # angular speed (rad/s)
        self.radius = 0 # angle speed radius (m)
        self.wheel_radius = 0.18 # wheel radius (m)
        self.wheel_dis = 0.65 # wheel distance (m)

        # motor information
        self.left_spd = 0 # left motor speed (m/s)
        self.right_spd = 0 # right motor speed (m/s)

        # serial part
        self.left_motor = SingleDriverSerial(port_key=DRIVER_LEFT_LOCATION)
        self.right_motor = SingleDriverSerial(port_key=DRIVER_RIGHT_LOCATION)

        # get the initial motor position, then initial the Odometry
        self.left_motor.get_driver_position()
        self.right_motor.get_driver_position()
        self.odo = Odometry(Odo_l=self.left_motor.pos, Odo_r=self.right_motor.pos)


    def _get_wheels_speed(self):
        """
        combine the linear speed and angular speed to the motor speed
        based on the wheel radius
        :return:
        """
        # first put the linear speed to both wheels
        self.left_spd = self.speed
        self.right_spd = self.speed


        # then put the angular speed to the wheels, based on the wheel distance
        if self.omega >= 0:
            # turning right
            left_angular_spd = self.omega * (self.radius + self.wheel_dis/2)
            right_angular_spd = self.omega * (self.radius - self.wheel_dis/2)
        else:
            # turning left
            left_angular_spd = self.omega * (self.radius - self.wheel_dis/2)
            right_angular_spd = self.omega * (self.radius + self.wheel_dis/2)

        self.left_spd += left_angular_spd
        self.right_spd += right_angular_spd

        return self.left_spd, self.right_spd

    def _set_wheel_rpm(self):
        """
        set the wheel speed to the driver
        :return:
        """
        left_rpm = self.left_spd / (2 * 3.14 * self.wheel_radius) * 60
        right_rpm = self.right_spd / (2 * 3.14 * self.wheel_radius) * 60
        self.left_motor.set_driver_speed(rpm=left_rpm)
        self.right_motor.set_driver_speed(rpm=right_rpm)

    def set_speed(self, speed:float, omega:float):
        """
        set the speed and omega to the driver
        :param speed: linear speed (m/s)
        :param omega: angular speed (rad/s)
        :return:
        """
        self.speed = speed
        self.omega = omega
        self._get_wheels_speed()
        self._set_wheel_rpm()

    def control(self,):
        """
        A loop constantly update the speed to the drivers
        Also, update the odometry

        :return:
        """
        self.set_speed(speed, omega)



if __name__ == "__main__":
    driver = Driver()

