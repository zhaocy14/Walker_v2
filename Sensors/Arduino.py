import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
from Sensors.SensorFunctions import *
from Sensors.SensorConfig import *
import threading
import time
import numpy as np
import serial


class ArduinoModule(object):
    def __init__(self, dis_avg: bool = True, print_out: bool = False):
        """
        This module is for distance detection.
        Arduino board reads the inf data and controls the brake.
        The arduino continuously reads the distance data, which will be 10 units
        :arg
        dis_avg: whether to use moving average to mitigate noises
        print_out: to print the infrared data each iteration during reading
        """

        port_name, _ = detect_serials(port_key=ARDUINO_LOCATION, sensor_name="Arduino")
        self.serial = serial.Serial(port_name, ARDUINO_BAUDRATE, timeout=None)
        # distance data
        self.dis_dim = INFRARED_SENSOR_NUM # number of distance sensors
        self.dis_mean_width = 10  # sliding window width(frame number)
        self.dis_data = np.zeros((self.dis_dim,))   # sliding window
        self.dis_buffer = np.zeros((self.dis_mean_width,self.dis_dim))  # distance data
        self.dis_avg = dis_avg

        # threading
        self.event = threading.Event()
        self.thread = threading.Thread(target=self.read_dis_data, args=())

        # log print
        self.print_out = print_out

        # start the threading
        if self.serial.is_open:
            self.event.set()
            self.thread.start()


    def read_dis_data(self,):
        """
        infrared data reading loop.
        """
        while True:
            try:
                self.event.wait()
                one_line_data = self.serial.readline().decode()
                one_line_data = one_line_data.strip('\r\n').split(',')
                # sliding window update
                self.dis_buffer[0:-1, :] = self.dis_buffer[1:self.dis_dim+1, :]
                self.dis_buffer[-1, :] = np.array(one_line_data).reshape(self.dis_dim)
                if self.dis_avg: # if avg across the window
                    self.dis_data = np.mean(self.dis_buffer, axis=0)
                else:   # just the latest data
                    self.dis_data = self.dis_buffer[-1, :]
                if self.print_out:
                    print("Infrared data: ", self.dis_data)
            except Exception as e:
                print("arduino restarting because", e)
                time.sleep(0.5)



if __name__ == "__main__":
    arduino = ArduinoModule(print_out=True)



