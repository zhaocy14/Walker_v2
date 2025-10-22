import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
from Sensors.SensorFunctions import *
from Sensors.SensorConfig import *
import numpy as np
import threading
import time
import serial


class SoftSkin(object):

    def __init__(self, device:str = "OrangePi"):
        # serial
        if device == "OrangePi":
            port_name = '/dev/ttyS3'
        else:
            port_name, _ = detect_serials(port_key=SOFTSKIN_LOCATION, sensor_name="Softskin")  # Arduino Mega 2560 ttyACM0
        self.serial = serial.Serial(port_name, SOFTSKIN_BAUDRATE, timeout=None)
        print("Softskin serial port:", port_name)

        # sensor number
        self.sensor_num = SKIN_SENSOR_NUM

        # data list
        self.data_list = []
        self.voltage_data = np.zeros((self.sensor_num))
        self.pressure_data = np.zeros((self.sensor_num))

        # detect abnormal signal
        self.max_pressure = 0
        self.is_abnormal = False
        # some threshold
        self.pressed_threshold_low = 1000  # to test a normal gentle grab
        self.pressed_threshold_high = 12900
        self.emergency_threshold = 13800  # huge force

        self.convert_table = np.zeros((2, 14))
        self.initialize_table()

        self.is_pressed = False
        # self.build_base_line_data()

        # threading
        self.reading_thread = threading.Thread(target=self.Softskin_main_thread, args=())
        self.reading_thread.start()

    def initialize_table(self):
        self.convert_table[0, :] = np.array(SKIN_TABLE_AC)
        self.convert_table[1, :] = np.array(SKIN_TABLE_PRESSURE)

    def data_process(self):
        """process the voltage data"""
        """first convert the voltage data to force data"""
        """detect whether there is an abnormal max pressure"""
        self.pressure_data = self.voltage_data
        self.pressure_data[0] = self.voltage_data[0]*2
        # if self.pressure_data[0] > 4700 or self.pressure_data.max() > 7000:
        #     if self.pressure_data.max() == self.pressure_data[2]:
        #         if self.pressure_data[2] > 9800:
        #             self.is_abnormal = True
        #     else:
        #         self.is_abnormal = True
        if self.pressure_data.max() > 9000:
            self.is_abnormal = True
        else:
            self.is_abnormal = False
        # self.max_pressure = self.pressure_data.max()
        # # if self.pressure_data.max() > self.max_pressure:
        # #     self.max_pressure = self.pressure_data.max()
        # #     print(self.max_pressure)
        # if self.max_pressure > self.pressed_threshold_low:
        #     self.is_pressed = True
        # else:
        #     self.is_pressed = False
        # if self.max_pressure > self.emergency_threshold:
        #     self.is_abnormal = True
        # else:
        #     self.is_abnormal = False

    def Softskin_main_thread(self):
        # try:
        self.serial.flush()
        while True:
            # the data would have 20 bytes starting with ff 00 00
            while True:
                # to detect the head data and command data
                head_data = self.serial.read(1).hex()
                if head_data == "ff":
                    command_data = self.serial.read(2).hex()
                    if command_data == "0000":
                        break
            self.data_list = []
            data = self.serial.read(17)
            print(data)
            for i in range(0, self.sensor_num * 2, 2):
                self.data_list.append(int.from_bytes(data[i:i + 2], byteorder='big', signed=False))
            self.voltage_data = np.array(self.data_list)
            print(self.voltage_data)
            self.data_process()
            # print(self.pressure_data, self.pressure_data[0]-self.pressure_data[2])

    # except BaseException as be:
    #     print("Data Error:", be)

    def unlock(self, unlock_time: int = 1):
        record_time = 0  # to record how long does the sensor are
        while True:
            # print(self.pressure_data[0], self.pressure_data[2])
            if 4000 > self.pressure_data[0] > 100 and \
                    6000 > self.pressure_data[2] > 600:
                # print("hlod!")
                time.sleep(0.1)
                record_time += 0.1
            else:
                record_time = 0
            if record_time >= unlock_time:
                break


if __name__ == '__main__':

    skin = SoftSkin()
    # while True:
    #     if skin.is_abnormal:
    #         time.sleep(0.01)
    #         print("yes")
    #         skin.unlock()
    # test
    # se = serial.Serial('/dev/ttyS3', 115200, timeout=None)
    # time.sleep(2)
    #
    # while True:
    #     print(se.read(1))
    #     time.sleep(0.2)