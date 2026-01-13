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

    def __init__(self, device:str = "OrangePi", is_show: bool = False):
        # serial
        if device == "OrangePi":
            port_name = '/dev/ttyS3'
        else:
            port_name, _ = detect_serials(port_key=SOFTSKIN_LOCATION, sensor_name="Softskin")  # Arduino Mega 2560 ttyACM0
        self.serial = serial.Serial(port_name, SOFTSKIN_BAUDRATE, timeout=None)
        # 转换设置读取速度的十六进制指令为字节类型
        set_speed_cmd = bytes.fromhex("FF820000640000000000001A")
        # 向串口写入指令
        self.serial.write(set_speed_cmd)
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

        self.emergency_threshold = [10000, 9000, 10000]  # huge force

        self.convert_table = np.zeros((2, 14))
        self.initialize_table()

        self.is_pressed = False
        # self.build_base_line_data()

        self.is_show = is_show

        # threading
        self.reading_thread = threading.Thread(target=self.softskin_main_thread, args=())
        self.reading_thread.start()

    def initialize_table(self):
        self.convert_table[0, :] = np.array(SKIN_TABLE_AC)
        self.convert_table[1, :] = np.array(SKIN_TABLE_PRESSURE)

    def data_process(self):
        """convert the voltage data to pressure data"""
        # TODO: to be done in the future
        self.pressure_data = self.voltage_data
        # to detect whether one sensor is abnormal among the three sensors
        bool_list = []
        for i in range(self.sensor_num):
            if self.pressure_data[i] > self.emergency_threshold[i]:
                bool_list.append(True)
            else:
                bool_list.append(False)
        if True in bool_list:
            self.is_abnormal = True
        else:
            self.is_abnormal = False

    def softskin_main_thread(self):
        self.serial.flush()
        while True:
            # the data would have 20 bytes starting with ff 00 00
            while True:
                # to detect the head data and command data
                # total 3 bytes
                head_data = self.serial.read(1).hex()
                if head_data == "ff":
                    command_data = self.serial.read(2).hex()
                    if command_data == "0000":
                        break
            self.data_list = []
            # read the remaining 17 bytes
            data = self.serial.read(17)
            # orangepi version only has 3 sensors
            # Sequence: left, middle, right
            for i in range(0, self.sensor_num * 2, 2):
                self.data_list.append(int.from_bytes(data[i:i + 2], byteorder='big', signed=False))
            self.voltage_data = np.array(self.data_list)
            if self.is_show:
                print(self.voltage_data)
            self.data_process()
            if self.is_abnormal:
                print("Softskin abnormal detected!", self.pressure_data)
                time.sleep(3)

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

    skin = SoftSkin(is_show=True)