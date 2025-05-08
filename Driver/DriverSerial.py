import pymodbus
from pymodbus.client import ModbusSerialClient
from sympy.strategies.branch import condition

from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class SingleDriverSerial(object):
    def __init__(self, port_key:str):
        """
        A class to handle serial communication with a single motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(SingleDriverSerial, self).__init__()

        # serial part
        port_name, _ = detect_serials(port_key=port_key, sensor_name="Driver")
        self.client = ModbusSerialClient(
            framer=pymodbus.framer.FramerType.RTU,
            port=port_name,
            baudrate=DRIVER_BAUDRATE,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8
        )
        self.slave_id = 0x01  # Modbus slave ID
        self.client.connect()


    def get_driver_position(self):
        """
        Get the status of the driver.
        :return: status
        """
        try:
            result = self.client.read_holding_registers(
                address=0x0004,
                count=2,
                slave=self.slave_id
            )
            if not result.isError():
                position = (result.registers[0] << 16) + result.registers[1]
                return position
            else:
                print(f"读取电机位置时发生错误: {result}")
        except Exception as e:
            print(f"读取电机位置时发生异常: {e}")
        return None

    def get_motor_speed(self):
        """
        Get the speed of the motor.
        :return: speed
        """
        try:
            result = self.client.read_holding_registers(
                address=0x0019,
                count=1,
                slave=self.slave_id
            )
            if not result.isError():
                speed = result.registers[0]
                return speed
            else:
                print(f"读取电机速度时发生错误: {result}")
        except Exception as e:
            print(f"读取电机速度时发生异常: {e}")
        return None

    def set_driver_speed(self, speed):
        """
        Set the speed of the driver.
        :param speed: speed value
        :return: None
        """
        try:
            if speed > 0:
                cond = 1
            elif speed == 0:
                cond = 0
            elif speed < 0:
                cond = 257
            else:
                cond = 256
            speed = abs(speed)
            self.set_motor_cond(cond)
            self.client.write_register(
                address=0x009a,
                value=speed,
                slave=self.slave_id
            )
        except Exception as e:
            print(f"设置电机速度时发生异常: {e}")


    def set_motor_enable(self, enable, low_cur=False):
        """
        Enable or disable the motor.
        :param enable: True to enable, False to disable
        :return: None
        """
        try:
            self.client.write_register(
                address=0x0008,
                value=1 if enable else 0,
                slave=self.slave_id
            )
            print(f"电机{'启用' if enable else '禁用'}")
            if not enable and low_cur: # set low current for better disable
                self.set_motor_low_current()
        except Exception as e:
            print(f"设置电机状态时发生异常: {e}")


    def set_motor_low_current(self):
        """
        Set the low current of the motor.
        :return: None
        """
        try:
            self.client.write_register(
                address=0x000A,
                value=0x0000,
                slave=self.slave_id
            )
        except Exception as e:
            print(f"设置电机低电流时发生异常: {e}")


    def set_motor_cond(self, cond:int = 0):
        """
        Set the condition of the motor.
        :param cond: 0 for slow down; 1 for turn forward; 257 for turn backward; 256 for shart stop
        :return: None
        """
        try:
            self.client.write_register(
                address=0x00c8,
                value=cond,
                slave=self.slave_id
            )
        except Exception as e:
            print(f"设置电机状态时发生异常: {e}")



if __name__ == "__main__":
    import time
    time.sleep(3)
    driver = SingleDriverSerial(port_key=DRIVER_RIGHT_LOCATION)
    # a loop for reading and logging driver's position, speed
    # driver.set_motor_enable(False, True)
    # while True:
    #     position = driver.get_driver_position()
    #     speed = driver.get_motor_speed()
    #     print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    #     time.sleep(0.1)

    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")

    driver.set_driver_speed(10)
    time.sleep(2)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")

    driver.set_driver_speed(0)
    time.sleep(2)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")

    driver.set_driver_speed(-10)
    time.sleep(2)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")

    driver.set_driver_speed(0)
    time.sleep(2)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")