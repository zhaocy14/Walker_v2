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

        self.max_rpm = 80 # approximately 0.75m/s

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
        val: 0~4294967295
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
        英鹏飞485驱动器 最终版 + 转速过载保护
        核心：删除冲突的方向寄存器，严格遵循协议，自动限制最大转速
        功能：rpm正负直接控制方向，右电机自动反向适配对称安装，超转速自动保护
        """
        # 1. 基础校验
        if motor not in ['left', 'right']:
            print("电机选择错误，仅支持'left'/'right'")
            return

        # # 2. 对称电机核心逻辑：右电机自动反向（适配双电机对称安装）
        # if motor == "right":
        #     target_rpm = -rpm  # 右电机取反，实现左右同步前进/后退
        # else:
        #     target_rpm = rpm  # 左电机保持原方向
        # 无需在此对称修改，因为会影响上层的转弯和直行逻辑控制，直接交给上层完成对称相关的修正
        target_rpm = rpm

        # ===================== 核心：转速过载保护机制 =====================
        abs_target = abs(target_rpm)
        if abs_target > self.max_rpm:
            # 超限时：自动钳位到最大转速，保持原方向，打印警告
            target_rpm = self.max_rpm if target_rpm > 0 else -self.max_rpm
        # =================================================================

        # 3. 映射启停指令（驱动器唯一方向控制源）
        abs_rpm = abs(target_rpm)
        if target_rpm > 0:
            cond = 1  # 正转
        elif target_rpm < 0:
            cond = 257  # 反转
        else:
            cond = 0  # 停止

        # 4. 安全换向：方向改变时，先停止电机
        pre_rpm = self.l_rpm if motor == 'left' else self.r_rpm
        if pre_rpm != 0 and (target_rpm * pre_rpm < 0):
            self.set_motor_cond(motor=motor, cond=0)
            time.sleep(0.1)

        # # 5. 驱动器严格协议顺序：停止 → 写速度 → 启动
        # self.set_motor_cond(motor=motor, cond=0)
        # time.sleep(0.01)

        # 写入速度（仅支持绝对值）
        self._write_register(
            address=0x009A,
            value=abs_rpm,
            action=f"{motor}电机速度: {abs_rpm}rpm",
            motor=motor
        )
        time.sleep(0.01)

        # 发送启停+方向指令
        self.set_motor_cond(motor=motor, cond=cond)

        # 6. 更新本地缓存
        if motor == 'left':
            self.l_rpm = target_rpm
        else:
            self.r_rpm = target_rpm

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


    def delta_encoder(start, end):
        """处理 32 位无符号编码器回绕（范围 0~4294967295）"""
        diff = end - start
        if diff > 2147483647:  # 超过 INT32_MAX，说明反向回绕
            diff -= 4294967296
        elif diff < -2147483648:  # 小于 INT32_MIN，说明正向回绕
            diff += 4294967296
        return diff


    # 先使能
    driver.set_motor_enable(enable=True, motor='left')
    driver.set_motor_enable(enable=True, motor='right')
    time.sleep(0.2)


    def test_motor(motor: str, test_rpm: int = 15, duration: float = 1.0):
        """
        测试单个电机：编码器方向 + 实际转速反馈验证
        """
        print(f"\n========== 测试 {motor} 电机 ==========")

        # 1. 读初始状态
        l0, r0 = driver.get_driver_position()
        start_pos = l0 if motor == 'left' else r0
        l_spd0, r_spd0 = driver.get_motor_speed()
        start_spd = l_spd0 if motor == 'left' else r_spd0
        print(f"[{motor}] 初始编码器: {start_pos}, 初始转速反馈: {start_spd} rpm")

        # 2. 发送正转指令
        print(f"[{motor}] 发送设定 RPM = +{test_rpm}")
        driver.set_single_driver_speed(rpm=test_rpm, motor=motor)

        # 3. 等待稳定后读取实际转速
        time.sleep(0.3)
        l_spd1, r_spd1 = driver.get_motor_speed()
        mid_spd = l_spd1 if motor == 'left' else r_spd1
        print(f"[{motor}] 0.3秒后实际转速反馈: {mid_spd} rpm")

        time.sleep(duration - 0.3)
        l_spd2, r_spd2 = driver.get_motor_speed()
        end_spd = l_spd2 if motor == 'left' else r_spd2
        print(f"[{motor}] {duration}秒后实际转速反馈: {end_spd} rpm")

        # 4. 读最终编码器位置并停止
        l1, r1 = driver.get_driver_position()
        end_pos = l1 if motor == 'left' else r1
        driver.set_single_driver_speed(rpm=0, motor=motor)
        time.sleep(0.3)

        # 5. 计算带回绕保护的差值
        delta = delta_encoder(start_pos, end_pos)
        print(f"[{motor}] 最终编码器: {end_pos}, 变化量 Δ = {delta}")

        # 6. 转速一致性判断
        print(f"[{motor}] 设定值 vs 实际值: {test_rpm} vs {end_spd}")
        if abs(end_spd - test_rpm) <= 3:
            print(f"[{motor}] 转速反馈 ✓ 基本一致")
        elif abs(end_spd) < 2 and test_rpm != 0:
            print(f"[{motor}] 转速反馈 ✗ 电机未转或反馈异常")
        else:
            print(f"[{motor}] 转速反馈 △ 有偏差，注意检查")

        # 7. 方向判断
        if delta > 0:
            print(f"[{motor}] 方向: 发送 +RPM 时编码器增加")
        elif delta < 0:
            print(f"[{motor}] 方向: 发送 +RPM 时编码器减少")
        else:
            print(f"[{motor}] 方向: 编码器无变化")

        return delta, end_spd


    # 分别测试左右电机
    delta_left, spd_left = test_motor('left', test_rpm=20, duration=1.0)
    delta_right, spd_right = test_motor('right', test_rpm=20, duration=1.0)

    # 综合判断对称性
    print("\n========== 综合判断 ==========")
    print(f"left:  Δpos={delta_left}, 实际转速={spd_left}")
    print(f"right: Δpos={delta_right}, 实际转速={spd_right}")

    same_sign = (delta_left > 0 and delta_right > 0) or (delta_left < 0 and delta_right < 0)
    if same_sign:
        print("结论: 两编码器同向变化 → 需要给其中一个软件取反")
        if abs(delta_left) <= abs(delta_right):
            print("建议: flip_left = True")
        else:
            print("建议: flip_right = True")
    else:
        print("结论: 两编码器反向变化 → 已自然对称，无需软件取反")

    # 失能
    driver.set_motor_enable(enable=False, motor='left')
    driver.set_motor_enable(enable=False, motor='right')
