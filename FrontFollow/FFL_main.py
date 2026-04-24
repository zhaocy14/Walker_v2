import numpy as np
import torch
import time
import threading
import sklearn
import os
import sys
import signal

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

from Driver.DriverAgent import DriverAgent
from Sensors import Cameras, Softskin, LiDAR_YDLIDAR, Button
from SpeedBuffer import EMABuffer, SCurvePlanner, MinJerkPlanner


class FFL(object):
    def __init__(self):
        """
        Front Following
        """
        super(FFL, self).__init__()

        # modules(sensors, drivers)
        self.driver = DriverAgent()
        # self.camera = Cameras.DualCamera()
        self.LiDAR = LiDAR_YDLIDAR.LiDAR_YDLIDAR()
        self.softskin = Softskin.SoftSkin()
        self.button = Button.Button()

        # speed parameters
        self.f_spd = 0.6  # forward speed(m/s)
        self.b_spd = -0.6  # backward speed(m/s)
        self.t_spd = 0.6  # turning speed(m/s) # maximum turning speed for counting the omega

        # 新增：速度缓冲层配置（参数内聚在SpeedBuffer类中，FFL仅选择模式）
        # 可选: 'ema' | 'scurve' | 'minjerk'
        self.buffer_mode = 'minjerk'
        self._init_planners()

        # leg data
        self.left_leg = np.zeros((2,))
        self.right_leg = np.zeros((2,))
        self.human_center = np.zeros((2,))

        # conditioning parameters
        self.forward_boundary = 150  # if the legs are further than this, then should go forward
        self.backward_boundary = -50  # likewise, go backward
        self.center_left_boundary = 20  # like wise, turn left while moving forward, the center means this is for the center point of user
        self.center_right_boundary = -20  # like wise, turn right
        self.left_boundary = 90
        self.right_boundary = -70
        self.left_max_boundary = 140  # left max value
        self.right_max_boundary = -140  # right max value

        # 全局退出标志
        self.running = True

        self.FFLthread = threading.Thread(target=self.main, args=())
        self.FFLthread.daemon = True
        self.FFLthread.start()

    # 新增：安全停止函数，电机失能
    def stop_safely(self):
        print("\n程序退出，电机失能...")
        self.update_driver(0, 0, 0)  # 速度参数归零
        self.driver.stop()  # 先失能电机，再停控制循环

    def _init_planners(self):
        """
        根据 buffer_mode 直接实例化各通道规划器，无需外部传参。
        所有参数内聚在 SpeedBuffer 类内部。
        """
        if self.buffer_mode == 'ema':
            self.planner_speed = EMABuffer()
            self.planner_omega = EMABuffer()
            self.planner_radius = EMABuffer()
        elif self.buffer_mode == 'scurve':
            self.planner_speed = SCurvePlanner(channel='speed')
            self.planner_omega = SCurvePlanner(channel='omega')
            self.planner_radius = SCurvePlanner(channel='radius')
        elif self.buffer_mode == 'minjerk':
            self.planner_speed = MinJerkPlanner(channel='speed')
            self.planner_omega = MinJerkPlanner(channel='omega')
            self.planner_radius = MinJerkPlanner(channel='radius')
        else:
            raise ValueError(f"Unknown buffer_mode: {self.buffer_mode}")

    def update_driver(self, speed: float = 0, omega: float = 0, radius: float = 0):
        # ============================================
        # 方案C：状态感知——转弯→直行快速过渡
        # ============================================
        # 判断条件：目标要求直行前进(speed>0)，且目标omega已归零，
        # 但当前底盘仍有残余角速度（说明刚从转弯切过来）
        is_turn_to_straight = (
            speed > 0.05
            and abs(omega) < 1e-3
            and abs(self.driver.omega) > 0.05
        )

        if is_turn_to_straight:
            # 对 omega 和 radius 强制立即归零，消除"转过了"的尾巴
            # speed 仍走正常缓冲，保证起步平滑
            actual_omega = self.planner_omega.quick_settle(0.0)
            actual_radius = self.planner_radius.quick_settle(0.0)
            actual_speed = self.planner_speed.update(speed)
        else:
            # 其他所有情况（含转弯→停止）维持原有缓冲逻辑
            actual_speed = self.planner_speed.update(speed)
            actual_omega = self.planner_omega.update(omega)
            actual_radius = self.planner_radius.update(radius)
        # ============================================

        # 保险：防止 DriverAgent 中 if self.omega < 0 在临界区触发
        if abs(actual_omega) < 1e-4:
            actual_omega = 0.0
        if abs(actual_speed) < 1e-4:
            actual_speed = 0.0

        self.driver.speed = actual_speed
        self.driver.radius = actual_radius
        self.driver.omega = actual_omega

    def main(self):
        first_run = True  # 标记是否为首次进入循环
        while self.running:
            time.sleep(0.1)
            # ============================================
            # 新增：首次启动强制解锁环节（仅第一次执行）
            # ============================================
            if first_run:
                self._wait_for_startup_unlock()
                if not self.running:
                    break
                first_run = False

            # ============================================
            # 原有：SoftSkin 异常检测与波峰解锁机制
            # ============================================
            if self.softskin.is_abnormal:
                self._handle_softskin_emergency()
                continue  # 回到循环开头，继续下一次循环

            # ============================================
            # 原有正常跟随逻辑（完全不变）
            # ============================================
            leg_data = self.LiDAR.get_leg_data()
            if leg_data is not None:
                self.left_leg = leg_data[0]
                self.right_leg = leg_data[1]

                # 优化：先判断无腿，再计算中心，避免无效数据运算
                if self.left_leg[0] < -1500 or self.right_leg[0] < -1500:
                    print("no leg detected, stop")
                    self.driver.brake()  # ← 直接写入 0 RPM，绕过缓冲层
                    # 同步清空缓冲层内部状态，避免恢复时突跳
                    self.planner_speed.quick_settle(0.0)
                    self.planner_omega.quick_settle(0.0)
                    self.planner_radius.quick_settle(0.0)
                    continue

                self.human_center = (self.left_leg + self.right_leg) / 2
                print("left leg:", self.left_leg, "\tright leg:", self.right_leg, "\thuman center:", self.human_center)

                # conditioning
                if self.human_center[0] > self.forward_boundary:
                    if self.human_center[1] > self.center_left_boundary:
                        # turn left
                        if self.LiDAR.ob_front > 0 or self.LiDAR.ob_front_left > 0:
                            # obstacle
                            print("go left but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            print("go left")
                            # 左转半径（正常，保持不变）
                            radius = max(0.5, 0.3 + abs(0.4 * (self.left_max_boundary - self.left_leg[1]) / (
                                    self.left_max_boundary - self.left_boundary)))
                            omega = -self.t_spd / radius
                            print(f"radius:{radius:.3f}, omega:{omega:.3f}")
                            self.update_driver(speed=0, omega=omega, radius=radius)
                    elif self.human_center[1] < self.center_right_boundary:
                        # turn right
                        if self.LiDAR.ob_front > 0 or self.LiDAR.ob_front_right > 0:
                            # obstacle
                            print("go right but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            print("go right")
                            # ✅ 核心修复：右转半径公式与左转完全对称，分母参数修正
                            radius = max(0.5, 0.3 + abs(0.4 * (self.right_leg[1] - self.right_max_boundary) / (
                                    self.right_max_boundary - self.right_boundary)))
                            omega = self.t_spd / radius
                            print(f"radius:{radius:.3f}, omega:{omega:.3f}")
                            self.update_driver(speed=0, omega=omega, radius=radius)
                    else:
                        if self.LiDAR.ob_front > 0:
                            # obstacle
                            print("go front but obstacle")
                            self.update_driver(speed=0, omega=0, radius=0)
                        else:
                            # go straight
                            print("go forward")
                            self.update_driver(speed=self.f_spd, omega=0, radius=0)
                elif self.human_center[0] < self.backward_boundary:
                    if self.LiDAR.ob_back > 0:
                        # obstacle
                        print("go back but obstacle")
                        self.update_driver(speed=0, omega=0, radius=0)
                    else:
                        # go backward
                        print("go backward")
                        self.update_driver(speed=self.b_spd, omega=0, radius=0)
                else:
                    # stop
                    print("stop")
                    self.update_driver(speed=0, omega=0, radius=0)

    # ============================================
    # 新增：解锁逻辑封装
    # ============================================
    def _wait_for_startup_unlock(self):
        """
        首次启动时的强制解锁等待。
        阻塞直到检测到连续3个波峰解锁信号。
        """
        self.driver.enable_driver(False)  # ← 新增：首次启动先失能，安全等待解锁
        self.softskin.start_unlock_monitoring()
        print("🔒 System locked on startup. Waiting for unlock to begin following...")

        # 进入监听循环，等待连续3个波峰（带2秒超时重置）
        while self.running and not self.softskin.check_can_unlock():
            self.softskin.detect_peaks()  # 检测波峰
            time.sleep(0.05)  # 50ms 检查间隔

        if not self.running:
            return  # 被外部终止，保持失能状态退出

        # 解锁成功，重置状态并恢复电机
        print("✅ Startup unlocked. Beginning front following...")
        self.softskin.reset_after_unlock()
        self.driver.enable_driver(True)  # ← 新增：解锁成功，恢复电机使能

    def _handle_softskin_emergency(self):
        """
        SoftSkin 异常检测后的紧急停止与解锁恢复。
        直接制动（绕过缓冲层），保持电机使能作刹车，不滑动。
        """
        print("emergency stop due to abnormal softskin force")

        # 直接制动：绕过缓冲层，直接写入0 RPM，保持电机使能（刹车）
        self.driver.brake()

        # 启动解锁监听模式
        self.softskin.start_unlock_monitoring()
        print("🔒 System locked. Waiting for 3 taps to unlock...")

        # 进入监听循环，等待连续3个波峰（带2秒超时重置）
        while self.running and not self.softskin.check_can_unlock():
            self.softskin.detect_peaks()  # 检测波峰
            time.sleep(0.05)  # 50ms 检查间隔

        if not self.running:
            return

        # 解锁成功，重置状态并恢复
        print("✅ unlocked, resuming front following...")
        self.softskin.reset_after_unlock()

        # 清空缓冲层残余，从0平滑起步，避免恢复时突跳
        self.planner_speed.quick_settle(0.0)
        self.planner_omega.quick_settle(0.0)
        self.planner_radius.quick_settle(0.0)


if __name__ == "__main__":
    ffl = FFL()


    # __main__ 中的信号处理：按顺序停止所有模块
    def exit_handler(signum, frame):
        print(f"\n[info] 收到信号 {signum}，准备退出...")
        # 1. 通知 FFL 主循环退出
        ffl.running = False
        ffl.FFLthread.join(timeout=2)

        # 2. 先失能电机（最高优先级）
        ffl.stop_safely()

        # 3. 再停其他传感器（顺序无关，但建议按资源占用从重到轻）
        ffl.LiDAR.stop()
        ffl.softskin.stop()
        ffl.button.stop()
        if hasattr(ffl, 'camera') and ffl.camera is not None:
            ffl.camera.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_handler)
    signal.signal(signal.SIGTERM, exit_handler)

    time.sleep(1)

    # 保持主程序运行
    while ffl.running:
        time.sleep(0.5)