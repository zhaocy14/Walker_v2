import serial
import threading
import time
import rplidar
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class LiDAR(object):
    def __init__(self, is_zmq: bool = True):
        """
        Search the LiDAR for scanning. It includes two version: python and C.
        Usually use the python version. While the C version is to use the ZMQ to communicate between C and python code.
        :param is_zmq: a bool value to decide whether you use python version(False) or C version(True)
        """
        super().__init__()
        if not is_zmq:
            self.port_name, _ = detect_serials(port_key=LIDAR_LOCATION_LOW, sensor_name="LiDAR Python")
            self.python_lidar = rplidar.RPLidar(self.port_name)
        else:
            self.port_name, _ = detect_serials(port_key=LIDAR_LOCATION_HIGH, sensor_name="LiDAR")
        # store the data
        self.scan_data_list = []

    def python_scan(self, is_show: bool = False, retry = 10):
        """ just for checking LiDAR serial port"""
        try:
            info = self.python_lidar.get_info()
            health = self.python_lidar.get_health()
            print(info)
            print(health)
            for i, scan in enumerate(self.python_lidar.iter_scans()):
                self.scan_data_list = scan
                if is_show:
                    print(self.scan_data_list)
        except BaseException as be:
            if retry > 0:
                print(f"{be}, retrying")
                self.python_lidar.stop()
                self.python_lidar.stop_motor()
                self.python_lidar.reset()
                self.python_lidar.start_motor()
                return self.python_scan(is_show=is_show, retry=retry-1)
            else:
                print("Failed to start the lidar.")


if __name__ == "__main__":
    # just for checking the LiDAR
    lidar_instance = LiDAR(is_zmq=False)
    lidar_instance.python_scan(is_show=True)
    # print(lidar_instance.port_name)
