import pymodbus
from pymodbus.client import ModbusSerialClient
from sympy.strategies.branch import condition
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class SingleDriverSerial(object):
    def __init__(self, port_key:str, slave_id:int = 0x01):
        """
        A class to handle serial communication with a single motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(SingleDriverSerial, self).__init__()

        # motor information
        self.pos = 0 # motor position
        self.rpm = 0 # motor speed(rpm)
        self.enable = False # motor enable status

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
        self.slave_id = slave_id  # Modbus slave ID, default is 0x01; Left:0x01, Right:0x02
        self.client.connect()

    def _write_register(self, address:int, value:int, action:str):
        """
        Write a value to a register. Single register.
        Function code is embedded in the pymodbus library.
        :param address: register address
        :param value: value to write
        :param action: action name for logging
        :return: None
        """
        try:
            self.client.write_register(
                address=address,
                value=value,
                slave=self.slave_id
            )
        except Exception as e:
            print(f"{action}时发生异常: {e}")

    def _read_register(self, address:int, count:int, action:str):
        """
        Read a value from a register. Single register.
        Function code is embedded in the pymodbus library.
        :param address: register address
        :param count: number of registers to read
        :param action: action name for logging
        :return: value read
        """
        try:
            result = self.client.read_holding_registers(
                address=address,
                count=count,
                slave=self.slave_id
            )
            if not result.isError():
                return result.registers[0]
            else:
                print(f"{action}时发生错误: {result}")
        except Exception as e:
            print(f"{action}时发生异常: {e}")
        return None

    def get_driver_position(self):
        """
        Get the status of the driver.
        :return: status
        """
        self.pos = self._read_register(address=0x0004, count=2, action="读取电机位置")
        return self.pos

    def get_motor_speed(self):
        """
        Get the speed of the motor.
        :return: speed
        """
        self.rpm = self._read_register(address=0x0019, count=1, action="读取电机速度")
        return self.rpm

    def set_driver_speed(self, rpm):
        """
        Set the speed of the driver.
        :param rpm: speed value
        :return: None
        """
        if rpm*self.rpm < 0:
            # if the target speed and the current speed have different signs
            # set the condition to stop first
            self.set_motor_cond(0)
            time.sleep(0.1)

        # first set turn forward/slow down/backward/sharp stop
        if rpm > 0:
            cond = 1  # forward
        elif rpm == 0:
            cond = 0  # slow down
        elif rpm < 0:
            cond = 257 # backward
        else:
            cond = 256 # sharp stop
        self.set_motor_cond(cond)
        # speed must be positive value
        rpm = abs(rpm)
        self._write_register(address=0x009a, value=rpm, action="设置电机速度")

    def set_motor_cond(self, cond:int = 0):
        """
        Set the condition of the motor.
        :param cond: 0 for slow down; 1 for turn forward; 257 for turn backward; 256 for shart stop
        :return: None
        """
        # note that you cannot set the motor turn forward and then backward immediately.
        # you need to set the motor to stop first(cond = 0).
        if cond not in [0, 1, 256, 257]:
            print(f"电机状态设置错误: {cond}, 只能设置为0, 1, 256, 257")
            # For safety, set the motor to stop first.
            self._write_register(address=0x00c8, value=0, action="设置电机状态")
        else:
            self._write_register(address=0x00c8, value=cond, action="设置电机状态")


    def set_motor_enable(self, enable:bool):
        """
        Enable or disable the motor.
        :param enable: True or False.
        :return: None
        """
        self.enable = enable
        self._write_register(address=0x00d4, value=0 if enable else 1, action="设置电机使能状态")


    def restart_motor(self):
        """
        Restart the motor.
        :return: None
        """
        self._write_register(address=0x00d4, value=0x0100, action="重启电机驱动")


    def set_motor_low_current(self):
        """
        Set the low current of the motor.
        :return: None
        """
        self._write_register(address=0x000A, value=1, action="设置电机低电流")

    def get_motor_alarm(self):
        """
        Get the alarm status of the motor.
        :return: alarm status
        """
        alarm = self._read_register(address=0x00a3, count=1, action="读取电机报警状态")
        # alarm is a hexadecimal value
        # 15~13 is the third alarm
        # 11~8 is the second alarm
        # 7~4 is the first alarm
        # 3~0 is the current alarm
        third_alarm = int((alarm[0] >> 13) & 0x07)
        second_alarm = int((alarm[0] >> 8) & 0x0F)
        first_alarm = int((alarm[0] >> 4) & 0x0F)
        current_alarm = int(alarm[0] & 0x0F)
        alarm_dict = {
            0: "正常",
            1: "电机相位过流",
            2: "供电电压过高",
            3: "供电电压过低",
            4: "电机A相开路",
            5: "电机B相开路",
            6: "其他报警或位置超差",
            7: "内部 24V 电压偏移",
            8: "AI电压错误",
            9: "BI电压错误",
            10: "编码器错误",
        }
        print(f"电机报警状态: {alarm[0]:#04x}, 第三报警: {alarm_dict[third_alarm]}, "
              f"第二报警: {alarm_dict[second_alarm]}, "
              f"第一报警: {alarm_dict[first_alarm]}, "
              f"当前报警: {alarm_dict[current_alarm]}")
        return alarm_dict[current_alarm]

    def clear_alarm(self):
        """
        Clear the alarm status of the motor. Recommend restarting the motor after clearing the alarm.
        :return: None
        """
        self._write_register(address=0x00a4, value=0, action="清除电机报警状态")




if __name__ == "__main__":
    import time
    time.sleep(3)
    driver = SingleDriverSerial(port_key='\dev\ttyS6', slave_id=0x01)
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

    # driver.set_driver_speed(10)
    # time.sleep(5)
    # position = driver.get_driver_position()
    # speed = driver.get_motor_speed()
    # print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    # #
    # # driver.set_driver_speed(20)
    # # position = driver.get_driver_position()
    # # speed = driver.get_motor_speed()
    # # print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    # # time.sleep(3)
    # #
    # driver.set_driver_speed(-10)
    # time.sleep(5)
    # position = driver.get_driver_position()
    # speed = driver.get_motor_speed()
    # print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    #
    # #
    # driver.set_driver_speed(0)
    # time.sleep(2)
    # position = driver.get_driver_position()
    # speed = driver.get_motor_speed()
    # print(f"电机当前绝对位置: {position}, 当前速度: {speed}")