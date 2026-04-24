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
from collections import deque


class SoftSkin(object):

    def __init__(self, device: str = "OrangePi", is_show: bool = False):
        # serial
        if device == "OrangePi":
            port_name = '/dev/ttyS3'
        else:
            port_name, _ = detect_serials(port_key=SOFTSKIN_LOCATION,
                                          sensor_name="Softskin")  # Arduino Mega 2560 ttyACM0
        self.serial = serial.Serial(port_name, SOFTSKIN_BAUDRATE, timeout=None)

        # ========== 新增：等待硬件稳定并清空启动前的脏数据 ==========
        time.sleep(0.3)
        self.serial.reset_input_buffer()

        # ========== 新增：强制设置主动上传模式（防止模块处于问答模式） ==========
        upload_cmd = bytes.fromhex("FF7840000000000000000048")
        self.serial.write(upload_cmd)
        self.serial.flush()
        time.sleep(0.1)

        # 转换设置读取速度的十六进制指令为字节类型
        set_speed_cmd = bytes.fromhex("FF820000210000000000005D") # 30Hz
        # set_speed_cmd = bytes.fromhex("FF820000640000000000001A") # 10Hz
        # 向串口写入指令
        self.serial.write(set_speed_cmd)
        self.serial.flush()
        time.sleep(0.1)

        # 再次清空缓冲区，丢弃命令响应或杂散字节
        self.serial.reset_input_buffer()
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

        # ========== 新增：解锁监测相关变量 ==========
        self.unlock_mode = False  # 是否处于解锁监听状态
        self.peak_count = 0  # 已检测到的波峰数量
        self.can_unlock = False  # 是否可以解锁（达到3个波峰）
        self.peak_timeout = 2.0  # 超时时间：2秒内必须有新波峰

        # 波峰检测参数
        self.peak_threshold_low = 800  # 拍压波峰最小值（排除噪声）
        self.peak_threshold_high = 7000  # 拍压波峰最大值（正常拍压范围）
        self.peak_min_interval = 0.3  # 两个波峰间最小间隔（秒）

        # 波峰检测状态机
        self.pressure_history = deque(maxlen=5)  # 保存最近5个时刻的压力值用于判断趋势
        self.last_peak_time = 0
        self.trend_state = "STABLE"  # STABLE, RISING, FALLING

        # 退出标志与线程
        self.running = True
        self.reading_thread = threading.Thread(target=self.softskin_main_thread, args=())
        self.reading_thread.daemon = True
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

    def detect_peaks(self):
        """
        波峰检测算法（独立外部调用版本）：检测连续3个有效的拍压波峰
        规则：检测到波峰后，必须在2秒内检测到下一个波峰，否则计数器归零

        调用时机：应在解锁监听模式下由外部循环定期调用（如每50ms一次）
        """
        if not self.unlock_mode or self.can_unlock:
            return  # 只有在监听模式且未解锁时才进行检测

        current_time = time.time()

        # 超时检查：如果已有波峰但超过2秒没有新波峰，重置计数
        if self.peak_count > 0 and (current_time - self.last_peak_time) > self.peak_timeout:
            print(f"[Unlock Monitor] ⏰ Timeout! No peak within 2s. Reset from {self.peak_count} to 0")
            self.peak_count = 0

        # 使用3个传感器的平均值作为拍压检测值
        current_pressure = np.mean(self.pressure_data)

        # 维护历史记录
        self.pressure_history.append(current_pressure)
        if len(self.pressure_history) < 3:
            return

        # 计算趋势（最后几个点的斜率）
        recent_data = list(self.pressure_history)[-3:]
        prev_val, mid_val, curr_val = recent_data[0], recent_data[1], recent_data[2]

        # 状态机转换
        if self.trend_state == "STABLE":
            if curr_val > prev_val * 1.05:  # 上升5%认为开始RISING
                self.trend_state = "RISING"

        elif self.trend_state == "RISING":
            if curr_val < mid_val * 0.95:  # 从峰值下降，说明mid_val是峰值
                # 检测到波峰在 mid_val
                if self.peak_threshold_low < mid_val < self.peak_threshold_high:
                    # 检查时间间隔（避免一次拍压被多次计数）
                    if current_time - self.last_peak_time > self.peak_min_interval:
                        self.peak_count += 1
                        self.last_peak_time = current_time  # 更新最后波峰时间，重置2秒倒计时
                        print(f"[Unlock Monitor] Peak {self.peak_count}/3 detected: {mid_val:.1f}")

                        if self.peak_count >= 3:
                            self.can_unlock = True
                            print("[Unlock Monitor] ✓ Unlock condition met (3 peaks detected)")
                            return

                self.trend_state = "FALLING"
            elif curr_val < prev_val * 0.95:  # 未达峰值就下降，重置
                self.trend_state = "STABLE"

        elif self.trend_state == "FALLING":
            # 等待压力回落到基线附近
            if curr_val < self.peak_threshold_low * 0.5:  # 回落到阈值一半以下
                self.trend_state = "STABLE"
            elif curr_val > mid_val * 1.05:  # 又开始上升，可能是下一个波峰
                self.trend_state = "RISING"

    def start_unlock_monitoring(self):
        """
        外部调用：启动解锁监听模式
        重置计数器，准备检测3个波峰
        """
        self.unlock_mode = True
        self.peak_count = 0
        self.can_unlock = False
        self.last_peak_time = 0
        self.pressure_history.clear()
        self.trend_state = "STABLE"
        print("[SoftSkin] Unlock monitoring started. Waiting for 3 taps (timeout: 2s between taps)...")

    def stop_unlock_monitoring(self):
        """外部调用：停止监听"""
        self.unlock_mode = False

    def check_can_unlock(self):
        """外部调用：检查是否可以解锁"""
        return self.can_unlock

    def reset_after_unlock(self):
        """解锁成功后重置所有状态"""
        self.unlock_mode = False
        self.peak_count = 0
        self.can_unlock = False
        self.is_abnormal = False  # 重置异常标志，使外部可以继续正常检测
        self.pressure_history.clear()
        self.trend_state = "STABLE"

    def softskin_main_thread(self):
        self.serial.flush()
        # ========== 新增：线程启动时再次清空缓冲区，丢弃残留数据 ==========
        self.serial.reset_input_buffer()
        try:
            while self.running:
                # the data would have 20 bytes starting with ff 00 00
                while self.running:
                    # to detect the head data and command data
                    # total 3 bytes
                    head_data = self.serial.read(1).hex()
                    if head_data == "ff":
                        command_data = self.serial.read(2).hex()
                        if command_data == "0000":
                            break
                if not self.running:
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
        except Exception:
            # 串口被外部关闭（stop()）或异常断开，线程正常退出
            pass

    def stop(self):
        """外部调用：请求线程退出并关闭串口"""
        self.running = False
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()


if __name__ == '__main__':

    skin = SoftSkin(is_show=False)

    print("=" * 50)
    print("系统启动 - 软皮肤压力监控与异常保护系统")
    print("正常运行时打印压力值，异常时自动锁定并要求拍压解锁")
    print("=" * 50)

    while True:
        # ========================================
        # 阶段 1: 正常运行阶段
        # ========================================
        if not skin.is_abnormal:
            # 这里可以放置正常的控制逻辑
            print(f"[正常运行] 压力值: {skin.pressure_data} | 最大压力: {np.max(skin.pressure_data):.0f}")
            time.sleep(0.1)  # 正常轮询间隔
            continue

        # ========================================
        # 阶段 2: 检测到异常，打断循环并进入锁定状态
        # ========================================
        print("\n" + "!" * 50)
        print(f"⚠️  检测到压力异常！当前压力: {skin.pressure_data}")
        print("🔒 系统锁定！进入安全监听模式")
        print("👉 请连续拍压3次以解锁（每次间隔不超过2秒）")
        print("!" * 50 + "\n")

        # 启动解锁监听模式
        skin.start_unlock_monitoring()

        # ========================================
        # 阶段 3: 锁定监听阶段 - 持续检测波峰直到解锁
        # ========================================
        locked = True
        while locked:
            # 显式调用波峰检测（独立方法，非回调式）
            skin.detect_peaks()

            # 检查是否满足解锁条件（3个波峰且未超时）
            if skin.check_can_unlock():
                locked = False
                print("\n" + "=" * 50)
                print("✅ 解锁成功！检测到连续3次有效拍压")
                print("=" * 50 + "\n")
            else:
                # 可选：在这里添加额外逻辑，如超时退出整个系统、报警等
                # 当前使用内部的2秒超时重置机制
                time.sleep(0.05)  # 50ms检测间隔，避免CPU占用过高

        # ========================================
        # 阶段 4: 解锁成功，重置状态并恢复运行
        # ========================================
        skin.reset_after_unlock()
        print("🔄 系统恢复正常运行...")
        time.sleep(1)  # 解锁后短暂暂停，避免立即再次触发异常