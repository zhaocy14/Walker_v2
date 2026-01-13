import os, sys
import numpy as np
#   DATA PATH
PWD = os.path.abspath(os.path.abspath(__file__))
FATHER_PATH = os.path.abspath(os.path.dirname(PWD) + os.path.sep + "..")
sys.path.append(FATHER_PATH)
DATA_PATH = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".." + os.path.sep + "Data")

#   Serial Port Configuration
LIDAR_LOCATION_HIGH = "1-3"
LIDAR_LOCATION_LOW = "1-3"

SOFTSKIN_LOCATION = "1-4.4:1.1"
SOFTSKIN_BAUDRATE = 115200

INFRARED_LOCATION = "3-2.3"
INFRARED_BAUDRATE = 115200

POWER_LOCATION = "3-2.1:1.1"
POWER_BAUDRATE = 9600

ARDUINO_LOCATION = "1-4.3"
ARDUINO_BAUDRATE = 9600

DRIVER_LEFT_LOCATION = "1-4.2.1"
DRIVER_RIGHT_LOCATION = "1-4.2.2"
DRIVER_BAUDRATE = 115200
# =======================================================================

#   Softskin Configuration
SKIN_SENSOR_NUM = 3
SKIN_TABLE_PRESSURE = [0, 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]  # for AC to pressure converting
SKIN_TABLE_AC = [51, 1912, 2724, 3011, 3163, 3340, 3455, 3522, 3572, 3608, 3633, 3656, 3680, 3697]
SKIN_MAX_THRESHOLD = 8  # Abnormal maximum pressure
SKIN_SAFE_CHANGE_RATE = 10  # Safe pressure change rate for unlocking the walker
SKIN_EMERGENCY_CHANGE_RATE = 50     # Abnormal pressure change rate for locking the walker

# Infrared Sensor
INFRARED_SENSOR_NUM = 8

#   LiDAR Configuration
SCAN_UNIT = 1000   # 1m = SCAN_UNIT * unit, 100 for cm, 1000 for mm; at least 100
# scanning configurations
SCAN_SIZE = int(SCAN_UNIT * 3)
HALF_SIZE = int(SCAN_SIZE / 2)
# old version of filtering useless data
COLUMN_BOUNDARY = int(HALF_SIZE - 0.2*SCAN_UNIT)
BOTTOM_BOUNDARY = int(HALF_SIZE - 1*SCAN_UNIT)
FILTER_THETA = 150
# new version of filtering useless data
# all numerical represent *.* m
WALKER_TOP_BOUNDARY = int(0.1039 * SCAN_UNIT) # 0.10386m
WALKER_BOTTOM_BOUNDARY = int(0.4231 * SCAN_UNIT * 2) # 0.42314m x2, considering the backward area
WALKER_LEFT_BOUNDARY = int(0.354 * SCAN_UNIT)  # 0.354m
WALKER_RIGHT_BOUNDARY = int(0.354 * SCAN_UNIT)  # 0.354m
WALKER_BOX_BOUNDARY_VERTICAL = int(0.018 * SCAN_UNIT) # approximately the wheel diameter
WALKER_REAR_WHEEL_ROW_IDX = int(0.48 * SCAN_UNIT)
WALKER_REAR_WHEEL_COL_IDX = int(1 * SCAN_UNIT)
WALKER_REAR_WHEEL_DIAMETER = int(0.03 * SCAN_UNIT)
WALKER_REAR_WHEEL_WIDTH = int(0.08 * SCAN_UNIT)


# LiDAR center point
HUMAN_TO_LIDAR = int(0.5 * SCAN_UNIT) # human operating height to LiDAR center

# obstacle part
OBSTACLE_DISTANCE = int(0.015 * SCAN_UNIT)  # 15 cm detection