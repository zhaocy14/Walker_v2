import os,sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import pymodbus
from pymodbus.client import ModbusSerialClient
import serial


class DriversSerial(object):
    def __init__(self, port_key:str):
        """
        A class to handle serial communication with a single motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(DriversSerial, self).__init__()

        # motor information
        self.l_pos = 0 # left motor position
        self.l_rpm = 0 # left motor speed(rpm)
        self.l_enable = False # left motor enable status

        self.r_pos = 0 # right motor position
        self.r_rpm = 0 # right motor speed(rpm)
        self.r_enable = False # right motor enable status

        self.left_device_id = 0x01
        self.right_device_id = 0x02

        # serial part
        print("Driver serial port:", '/dev/ttyS6')
        self.client = ModbusSerialClient(
            framer=pymodbus.framer.FramerType.RTU,
            port='/dev/ttyS6',
            baudrate=115200,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8
        )
        self.client.connect()
        print(self.client.connected)

    def _write_register(self, address:int, value:int, action:str, motor:str):
        """
        Write a value to a register. Single register.
        Function code is embedded in the pymodbus library.
        :param address: register address
        :param value: value to write
        :param action: action name for logging
        :param motor: str, left or right
        :return: None
        """
        try:
            if motor == 'left':
                device_id = self.left_device_id
            elif motor == 'right':
                device_id = self.right_device_id
            else:
                device_id = 0
                print("电机选择错误，只能选择'left'或'right'，已设置为0")
            self.client.write_register(
                address=address,
                value=value,
                device_id=device_id
            )
        except Exception as e:
            print(f"{action}时发生异常: {e}")

    def _read_register(self, address:int, count:int, action:str, motor:str):
        """
        Read a value from a register. Single register.
        Function code is embedded in the pymodbus library.
        :param address: register address
        :param count: number of registers to read
        :param action: action name for logging
        :return: value read
        """
        try:
            if motor == 'left':
                device_id = self.left_device_id
            elif motor == 'right':
                device_id = self.right_device_id
            else:
                device_id = 0
                print("电机选择错误，只能选择'left'或'right'，已设置为0")
            result = self.client.read_holding_registers(
                address=address,
                count=count,
                device_id=device_id
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
        self.l_pos = self._read_register(address=0x0004, count=2, action="读取左电机位置", motor='left')
        self.r_pos = self._read_register(address=0x0004, count=2, action="读取右电机位置", motor='right')
        return self.l_pos, self.r_pos

    def get_motor_speed(self):
        """
        Get the speed of the motor.
        :return: speed
        """
        self.l_rpm = self._read_register(address=0x0019, count=1, action="读取左电机速度", motor='left')
        self.r_rpm = self._read_register(address=0x0019, count=1, action="读取右电机速度", motor='right')
        return self.l_rpm, self.r_rpm

    def set_single_driver_speed(self, rpm, motor:str):
        """
        Set the speed of the driver.
        :param rpm: speed value
        :param motor: str, left or right
        :return: None
        """
        if motor == 'left':
            cur_rpm = self.l_rpm
        elif motor == 'right':
            cur_rpm = self.r_rpm
        else:
            print("电机选择错误，只能选择'left'或'right'，已退出设置速度")
            return

        if rpm*cur_rpm < 0:
            # if the target speed and the current speed have different signs
            # set the condition to stop first
            self.set_motor_cond(motor=motor, cond=0)
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
        self.set_motor_cond(motor=motor, cond=cond)
        # speed must be positive value
        rpm = abs(rpm)
        self._write_register(address=0x009a, value=rpm, action="设置电机速度", motor=motor)

    def set_motor_cond(self, motor:str, cond:int = 0):
        """
        Set the condition of the motor.
        :param motor: str, left or right
        :param cond: 0 for slow down; 1 for turn forward; 257 for turn backward; 256 for shart stop
        :return: None
        """
        # note that you cannot set the motor turn forward and then backward immediately.
        # you need to set the motor to stop first(cond = 0).
        if motor == 'left':
            device_id = self.left_device_id
        elif motor == 'right':
            device_id = self.right_device_id
        else:
            print("电机选择错误，只能选择'left'或'right'，已退出设置电机状态")
            return

        if not device_id:
            return # invalid motor selection
        else:
            if cond not in [0, 1, 256, 257]:
                print(f"{motor}电机状态设置错误: {cond}, 只能设置为0, 1, 256, 257")
                # For safety, set the motor to stop first.
                self._write_register(address=0x00c8, value=0, action="设置电机状态", motor=motor)
            else:
                self._write_register(address=0x00c8, value=cond, action="设置电机状态", motor=motor)


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
    driver = DriversSerial(port_key='/dev/ttyS6')
    # a loop for reading and logging driver's position, speed
    # driver.set_motor_enable(False, True)
    # while True:
    #     position = driver.get_driver_position()
    #     speed = driver.get_motor_speed()
    #     print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    #     time.sleep(0.1)
    #
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")

    driver.set_single_driver_speed(rpm=10, motor='left')
    driver.set_single_driver_speed(rpm=10, motor='right')
    time.sleep(5)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    #
    driver.set_single_driver_speed(rpm=20, motor='left')
    driver.set_single_driver_speed(rpm=20, motor='right')
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    time.sleep(3)
    #
    driver.set_single_driver_speed(rpm=-10, motor='left')
    driver.set_single_driver_speed(rpm=-10, motor='right')
    time.sleep(5)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")
    #
    # #
    driver.set_single_driver_speed(rpm=-0, motor='left')
    driver.set_single_driver_speed(rpm=-0, motor='right')
    time.sleep(2)
    position = driver.get_driver_position()
    speed = driver.get_motor_speed()
    print(f"电机当前绝对位置: {position}, 当前速度: {speed}")