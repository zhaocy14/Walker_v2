import pymodbus
from pymodbus.client import ModbusSerialClient
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class DriverSerial(object):
    def __init__(self):
        """
        A class to handle serial communication with a motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(DriverSerial, self).__init__()

        # motor status & speed
        self.spd_L = 0
        self.pos_L = 0

        self.spd_R = 0
        self.pos_R = 0

        # serial part
        left_port_name, _ = detect_serials(port_key=DRIVER_LEFT_LOCATION, sensor_name="Driver")
        self.client_left = ModbusSerialClient(
            framer=pymodbus.framer.FramerType.RTU,
            port=left_port_name,
            baudrate=DRIVER_BAUDRATE,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8
        )
        # right_port_name, _ = detect_serials(port_key=DRIVER_RIGHT_LOCATION, sensor_name="Driver")
        # self.client_right = ModbusSerialClient(
        #     framer=pymodbus.framer.FramerType.RTU,
        #     port=right_port_name,
        #     baudrate=DRIVER_BAUDRATE,
        #     timeout=1,
        #     parity=serial.PARITY_NONE,
        #     stopbits=1,
        #     bytesize=8
        # )
        self.slave_id = 0x01  # Modbus slave ID


    def get_driver_position(self):
        """
        Get the status of the driver.
        :return: status
        """
        try:
            self.client_left.connect()
            result = self.client_left.read_holding_registers(
                address=0x0004,
                count=2,
                slave=self.slave_id
            )
            if not result.isError():
                position = (result.registers[0] << 16) + result.registers[1]
                print(f"电机当前绝对位置: {position}")
                return position
            else:
                print(f"读取电机位置时发生错误: {result}")
        except Exception as e:
            print(f"读取电机位置时发生异常: {e}")
        return None

    def set_driver_speed(self, speed):
        """
        Set the speed of the driver.
        :param speed: speed value
        :return: None
        """
        try:
            self.client_left.connect()
            self.client_left.write_registers(
                address=0x0019,
                values=[speed],
                slave=self.slave_id
            )
            print(f"电机速度设置为: {speed}")
        except Exception as e:
            print(f"设置电机速度时发生异常: {e}")


    def set_motor_enable(self, enable, lcurrent=False):
        """
        Enable or disable the motor.
        :param enable: True to enable, False to disable
        :return: None
        """
        try:
            self.client_left.connect()
            self.client_left.write_registers(
                address=0x0008,
                values=[1 if enable else 0],
                slave=self.slave_id
            )
            print(f"电机{'启用' if enable else '禁用'}")
            if not enable and lcurrent: # set low current for better disable
                self.set_motor_low_current()
        except Exception as e:
            print(f"设置电机状态时发生异常: {e}")


    def set_motor_low_current(self):
        """
        Set the low current of the motor.
        :return: None
        """
        try:
            self.client_left.connect()
            self.client_left.write_registers(
                address=0x000A,
                values=0x0000,
                slave=self.slave_id
            )
            print(f"电机低电流设置为低")
        except Exception as e:
            print(f"设置电机低电流时发生异常: {e}")


    def get_motor_speed(self):
        """
        Get the speed of the motor.
        :return: speed
        """
        try:
            self.client_left.connect()
            result = self.client_left.read_holding_registers(
                address=0x000C,
                count=1,
                slave=self.slave_id
            )
            if not result.isError():
                speed = result.registers[0]
                print(f"电机当前速度: {speed}")
                return speed
            else:
                print(f"读取电机速度时发生错误: {result}")
        except Exception as e:
            print(f"读取电机速度时发生异常: {e}")
        return None

if __name__ == "__main__":
    import time
    driver = DriverSerial()
    # a loop for reading and logging driver's position, speed
    driver.set_motor_enable(False,True)
    while True:
        position = driver.get_driver_position()
        speed = driver.get_motor_speed()
        print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
        time.sleep(0.1)

