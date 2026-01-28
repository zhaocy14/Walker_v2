import os,sys
import time
from PIL.SpiderImagePlugin import iforms

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import pymodbus
from pymodbus.client import ModbusSerialClient
import serial


class DriversSerial(object):
    def __init__(self, port_key:str = '/dev/ttyS6'):
        """
        A class to handle serial communication with a single motor driver.
        Using Modbus RTU protocol for communication.
        """
        super(DriversSerial, self).__init__()

        # motor information
        self.l_pos = 0 # left motor position
        self.l_rpm = 0 # left motor speed(rpm), for setting
        self.l_record_rpm = 0 # recorded rpm by reading
        self.l_enable = False # left motor enable status

        self.r_pos = 0 # right motor position
        self.r_rpm = 0 # right motor speed(rpm)
        self.r_record_rpm = 0 # recorded rpm by reading
        self.r_enable = False # right motor enable status

        self.left_device_id = 0x01
        self.right_device_id = 0x02

        # serial part
        self.client = ModbusSerialClient(
            framer=pymodbus.framer.FramerType.RTU,
            port=port_key,
            baudrate=115200,
            timeout=1,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8
        )
        self.client.connect()

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
                return result.registers
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
        l_regs = self._read_register(address=0x0004, count=2, action="读取左电机位置位置", motor='left')
        r_regs = self._read_register(address=0x0004, count=2, action="读取右电机位置位置", motor='right')
        self.l_pos = (l_regs[0] << 16) | l_regs[1] if l_regs is not None else 0
        self.r_pos = (r_regs[0] << 16) | r_regs[1] if r_regs is not None else 0
        return self.l_pos, self.r_pos

    def get_motor_speed(self):
        """
        Get the speed of the motor.
        :return: speed
        """
        l_regs = self._read_register(address=0x0019, count=1, action="读取左电机速度", motor='left')
        r_regs = self._read_register(address=0x0019, count=1, action="读取右电机速度", motor='right')
        self.l_record_rpm = l_regs[0] if l_regs is not None else 0
        self.r_record_rpm = r_regs[0] if r_regs is not None else 0
        return self.l_record_rpm, self.r_record_rpm

    def set_single_driver_speed(self, rpm, motor: str):
        """
        适配对称电机的速度设置：内部自动处理方向反转，外部无需手动调整正负
        - 左电机（left）：沿用输入rpm的原方向（原正负号）
        - 右电机（right）：自动取rpm的反方向（原正负号反转）
        - 其他逻辑完全遵循英鹏飞手册协议
        """
        # 1. 前置校验：电机选择+使能状态+速度范围
        if motor not in ['left', 'right']:
            print("电机选择错误，仅支持'left'/'right'")
            return

        # 2. 对称电机方向自动处理：右电机速度取反（核心适配逻辑）
        if motor == 'right':
            adjusted_rpm = -rpm  # 右电机反转，适配对称放置
        else:
            adjusted_rpm = rpm  # 左电机沿用原方向

        # 3. 速度值域校验（手册0x009A支持0~10000转/分）
        abs_adjusted_rpm = abs(adjusted_rpm)

        # 4. 方向与启停指令映射（基于调整后的速度）
        if adjusted_rpm > 0:
            direction = 0  # 正转（对应set_motor_direction的0）
            start_cond = 1  # 正转启动（对应set_motor_cond的1）
        elif adjusted_rpm < 0:
            direction = 1  # 反转（对应set_motor_direction的1）
            start_cond = 257  # 反转启动（对应set_motor_cond的257）
        else:
            direction = 0  # 速度为0时，方向默认正转
            start_cond = 0  # 减速停止（对应set_motor_cond的0）

        # 5. 高频切换安全逻辑：非0速切换方向时，先停止电机
        pre_rpm = self.l_rpm if motor == 'left' else self.r_rpm
        if pre_rpm != 0 and (adjusted_rpm * pre_rpm < 0):
            self.set_motor_cond(motor=motor, cond=0)  # 调用指定函数停止
            time.sleep(0.1)  # 确保电机停稳，避免堵转

        # 6. 按协议顺序执行：设方向 → 设速度 → 设启停
        # 6.3 调用指定函数启动/停止（0x00C8）
        self.set_motor_cond(motor=motor, cond=start_cond)
        # 6.1 调用指定函数设置方向（0x006B）
        self.set_motor_direction(direction=direction, motor=motor)
        # 6.2 写入速度（0x009A，手册要求仅支持非负值，传入调整后的绝对值）
        self._write_register(
            address=0x009A,
            value=abs_adjusted_rpm,
            action=f"{motor}电机速度设置（{abs_adjusted_rpm}转/分）",
            motor=motor
        )
        # print(start_cond, direction)
        # 6.3 调用指定函数启动/停止（0x00C8）
        self.set_motor_cond(motor=motor, cond=start_cond)
        # 6.1 调用指定函数设置方向（0x006B）
        self.set_motor_direction(direction=direction, motor=motor)

        # 7. 更新本地状态缓存（缓存调整后的速度，便于后续切换判断）
        if motor == 'left':
            self.l_rpm = adjusted_rpm
        else:
            self.r_rpm = adjusted_rpm

    def set_motor_direction(self, direction:int, motor:str):
        """
        Set the direction of the motor.
        :param direction: 0 for forward; 1 for backward
        :param motor: str, left or right
        :return: None
        """
        if motor not in ['left', 'right']:
            print("电机选择错误，只能选择'left'或'right'，已退出设置电机方向")
            return
        self._write_register(address=0x006b, value=direction, action="设置电机方向", motor=motor)

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


    def set_motor_enable(self, enable:bool, motor:str):
        """
        Enable or disable the motor.
        :param enable: True or False.
        :param motor: str, left or right
        :return: None
        """
        if motor == 'left':
            self.l_enable = enable
            self._write_register(address=0x00d4, value=0 if enable else 1, action="设置左电机使能状态", motor='left')
        elif motor == 'right':
            self.r_enable = enable
            self._write_register(address=0x00d4, value=0 if enable else 1, action="设置右电机使能状态", motor='right')


    def restart_motor(self):
        """
        Restart the motor.
        :return: None
        """
        self._write_register(address=0x00d4, value=0x0100, action="重启左电机驱动", motor='left')
        self._write_register(address=0x00d4, value=0x0100, action="重启右电机驱动", motor='right')


    def set_torque_limit(self, torque:int, motor:str):
        """
        Set the torque limit of the motor.
        :param torque: torque limit value
        :param motor: str, left or right
        :return: None
        """
        self._write_register(address=0x009E, value=torque, action="设置左电机力矩限制", motor='left')
        self._write_register(address=0x009E, value=torque, action="设置右电机力矩限制", motor='right')

    def get_motor_alarm(self):
        """
        Get the alarm status of the motor.
        :return: alarm status
        """
        all_alarm = {}
        for motor_id in ['left', 'right']:
            alarm = self._read_register(address=0x00a3, count=1, action="读取电机报警状态", motor=motor_id)
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
            all_alarm[motor_id] = alarm_dict[current_alarm]
        return all_alarm

    def clear_alarm(self):
        """
        Clear the alarm status of the motor. Recommend restarting the motor after clearing the alarm.
        :return: None
        """
        self._write_register(address=0x00a4, value=0, action="清除电机报警状态")




if __name__ == "__main__":
    driver = DriversSerial(port_key='/dev/ttyS6')
    # a loop for reading and logging driver's position, speed

    driver.set_motor_enable(enable=True, motor='left')
    driver.set_motor_enable(enable=True, motor='right')
    driver.set_single_driver_speed(rpm=-30, motor='left')
    driver.set_single_driver_speed(rpm=-30, motor='right')
    time.sleep(1)
    driver.set_single_driver_speed(rpm=0, motor='left')
    driver.set_single_driver_speed(rpm=0, motor='right')
    time.sleep(2)
    #
    driver.set_single_driver_speed(rpm=-30, motor='left')
    driver.set_single_driver_speed(rpm=-30, motor='right')
    time.sleep(2)
    #
    # # # #
    driver.set_single_driver_speed(rpm=30, motor='left')
    driver.set_single_driver_speed(rpm=30, motor='right')
    time.sleep(2)

    driver.set_single_driver_speed(rpm=0, motor='left')
    driver.set_single_driver_speed(rpm=0, motor='right')
    time.sleep(2)

    driver.set_single_driver_speed(rpm=30, motor='left')
    driver.set_single_driver_speed(rpm=30, motor='right')
    time.sleep(2)
    #
    # # # #
    driver.set_single_driver_speed(rpm=30, motor='left')
    driver.set_single_driver_speed(rpm=30, motor='right')
    time.sleep(1)
    #
    driver.set_motor_enable(enable=False, motor='left')
    driver.set_motor_enable(enable=False, motor='right')

    # print(f"电机当前绝对位置: {driver.get_driver_position()}, 当前速度: {driver.get_motor_speed()}")
