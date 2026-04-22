import datetime
import time
import cv2
import numpy
import threading
import glob

import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)


class DualCamera:
    def __init__(self, show_fps=False, root_path='./'):
        # 退出标志与同步
        self.running = True
        self.camera_lock = threading.Lock()  # 保护重连时的句柄替换

        # 扫描可用摄像头（验证帧）
        cameras = self._find_camera_devices()
        if len(cameras) < 2:
            raise RuntimeError(f"只找到 {len(cameras)} 个有效摄像头，需要 2 个")

        (idx0, self.camera1), (idx1, self.camera2) = cameras[0], cameras[1]
        self.camera_ports = [idx0, idx1]  # 记录端口，用于热插拔后优先重试
        print(f"Camera Ports: {self.camera_ports}")

        self.camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.set_frame_rate()

        # 图像数据方便外部调用
        self.image1 = None
        self.image2 = None

        # 数据保存路径
        self.root_path = root_path
        self.set_root_path(root_path)

        # 显示帧率，默认不显示，主要测试实际是否能达到60fps
        self.show_fps = show_fps

        # 为每个摄像头创建线程（daemon=True，内部启动）
        self.cam_thread1 = threading.Thread(target=self.run, args=(0,), daemon=True)
        self.cam_thread2 = threading.Thread(target=self.run, args=(1,), daemon=True)
        self.cam_thread1.start()
        self.cam_thread2.start()

    def _open_camera(self, index):
        """尝试打开指定 index，并验证能读到真实帧"""
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            # 有的设备 isOpened() 为 True 但实际已断开，必须读一帧验证
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                return cap
            cap.release()
        return None

    def _find_camera_devices(self):
        """
        扫描 /dev/video*，过滤 metadata/假设备，验证可读帧。
        返回 [(index, cap), ...] 列表，最多找 2 个真实摄像头。
        """
        found = []
        # 方法1：通过 /dev/video* 枚举（Linux 标准）
        video_nodes = sorted(glob.glob('/dev/video*'))

        # 方法2：如果 glob 失败，兜底扫描 0~19
        if not video_nodes:
            video_nodes = [f'/dev/video{i}' for i in range(20)]

        for node in video_nodes:
            # 过滤非标准命名（如 /dev/video-wrapped）
            if not node.startswith('/dev/video'):
                continue
            # 提取数字索引
            try:
                idx = int(node.replace('/dev/video', ''))
            except ValueError:
                continue

            cap = self._open_camera(idx)
            if cap is not None:
                found.append((idx, cap))
                if len(found) >= 2:
                    break

        return found

    def set_frame_rate(self, frame_rate=60):
        # 设置摄像头的帧率
        # 1280x720 会无法双摄像头60fps读取

        # width, height = 1280, 720
        # width, height = 800, 600
        # width, height = 640, 480
        width, height = 352, 280

        self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera1.set(cv2.CAP_PROP_FPS, frame_rate)

        self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera2.set(cv2.CAP_PROP_FPS, frame_rate)

        # print(int(self.camera1.get((cv2.CAP_PROP_FPS))))
        # print(int(self.camera2.get((cv2.CAP_PROP_FPS))))

    def set_root_path(self, root_path):
        # 创建保存路径
        os.makedirs(self.root_path, exist_ok=True)
        self.root_path = root_path

    def end(self):
        # 兼容旧接口
        self.stop()

    def stop(self):
        """请求线程退出并释放摄像头资源"""
        self.running = False
        with self.camera_lock:
            self.camera1.release()
            self.camera2.release()
        self.cam_thread1.join(timeout=1)
        self.cam_thread2.join(timeout=1)

    def run(self, cam_id):
        frame_count = 0
        start_time = time.time()

        while self.running:
            # 每次循环重新获取最新句柄（支持热插拔重连后更新）
            with self.camera_lock:
                camera = self.camera1 if cam_id == 0 else self.camera2

            if not camera.isOpened():
                print(f'USB camera {cam_id} is not opened')
                if self.running:
                    self.find_camera_port(reconfig=True)
                time.sleep(0.5)
                continue

            ret, frame = camera.read()
            if not ret:
                print(f'Can not read data from camera {cam_id}')
                if not self.running:
                    break

                # 读取失败，重复尝试10次（每次重新获取句柄）
                retry_success = False
                for try_i in range(10):
                    with self.camera_lock:
                        camera = self.camera1 if cam_id == 0 else self.camera2
                    ret, frame = camera.read()
                    if ret:
                        retry_success = True
                        break
                    time.sleep(0.05)

                if not retry_success:
                    print(f'Failed to read data from camera {cam_id} after 10 attempts')
                    if self.running:
                        self.find_camera_port(reconfig=True)
                    continue

            # # 旋转图像180°
            # frame = cv2.rotate(frame, cv2.ROTATE_180)

            if cam_id == 0:
                self.image1 = frame
            else:
                self.image2 = frame

            # # 实时显示图像
            # # 双摄像头会无法同时显示而卡死
            # # 想同时看必须降低频率，用单独一个线程来显示，详见示例
            # cv2.imshow(f'USB Camera {cam_id} Stream', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     print(f'USB camera {cam_id} quit')
            #     break
            # 计算帧率
            frame_count += 1
            elapsed_time = time.time() - start_time
            if self.show_fps and elapsed_time > 1:
                fps = frame_count / elapsed_time
                print(f'USB camera {cam_id} FPS: {fps:.2f}')
                frame_count = 0
                start_time = time.time()

        # 释放摄像头资源并关闭窗口
        with self.camera_lock:
            camera = self.camera1 if cam_id == 0 else self.camera2
            camera.release()
        # cv2.destroyAllWindows()
        print(f"USB camera {cam_id} thread ends")

    def get_images(self):
        # 获取当前图像
        return self.image1, self.image2

    def save_images(self, file_name1, file_name2, log=True):
        """
        保存图像到 ~/Walker_v2/log/camera/YYYY-MM-DD/ 下
        文件名格式: {file_name}_{HHMMSS}.jpg
        """
        # 基础路径
        base_path = os.path.expanduser('~/Walker_v2/log/camera')
        # 按日期分子文件夹
        today = datetime.datetime.now().strftime('%Y-%m-%d')
        save_dir = os.path.join(base_path, today)
        os.makedirs(save_dir, exist_ok=True)

        # 时间戳后缀
        timestamp = datetime.datetime.now().strftime('%H%M%S')

        frame1 = self.image1
        frame2 = self.image2

        # 组装完整文件名
        name1, ext1 = os.path.splitext(file_name1)
        name2, ext2 = os.path.splitext(file_name2)
        if not ext1:
            ext1 = '.jpg'
        if not ext2:
            ext2 = '.jpg'

        save_path1 = os.path.join(save_dir, f"{name1}_{timestamp}{ext1}")
        save_path2 = os.path.join(save_dir, f"{name2}_{timestamp}{ext2}")

        try:
            cv2.imwrite(save_path1, frame1)
            cv2.imwrite(save_path2, frame2)
        except Exception as e:
            print(f"Error occurred while saving the files: {e}")
            return

        if log:
            print(f"Save USB camera 1 frame to {save_path1} successfully.")
            print(f"Save USB camera 2 frame to {save_path2} successfully.")

    def find_camera_port(self, reconfig=False):
        """
        热插拔/掉线后重新寻找摄像头。
        优先尝试之前记住的端口，失败再扫描 /dev/video*。
        """
        with self.camera_lock:
            if reconfig:
                self.camera1.release()
                self.camera2.release()
                time.sleep(0.5)

            # 策略1：先尝试之前成功的端口
            if hasattr(self, 'camera_ports') and all(p != -1 for p in self.camera_ports):
                c1 = self._open_camera(self.camera_ports[0])
                c2 = self._open_camera(self.camera_ports[1])
                if c1 is not None and c2 is not None:
                    self.camera1 = c1
                    self.camera2 = c2
                    self._configure_cameras()
                    print("Reconfigured cameras using previous ports:", self.camera_ports)
                    return

            # 策略2：全量扫描
            cameras = self._find_camera_devices()
            if len(cameras) < 2:
                print("警告：未能找到两个可用的摄像头，保留旧句柄")
                return

            self.camera_ports = [cameras[0][0], cameras[1][0]]
            self.camera1 = cameras[0][1]
            self.camera2 = cameras[1][1]
            self._configure_cameras()
            print("Reconfigured cameras. New ports:", self.camera_ports)

    def _configure_cameras(self):
        """重连后统一设置编码和帧率"""
        self.camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.set_frame_rate()


if __name__ == "__main__":
    # def count_cameras():
    #     camera_count = 0
    #     for i in range(10):  # 尝试0到9的摄像头索引，可根据实际情况调整范围
    #         cap = cv2.VideoCapture(i)
    #         if cap.isOpened():
    #             print(i)
    #             camera_count += 1
    #             cap.release()
    #     return camera_count
    # count_cameras()
    # 创建 DualCamera 实例，可通过 show_fps 参数选择是否显示帧率
    dual_camera = DualCamera(show_fps=False)
    # 启动线程（已改到内部启动，无需外部调用）
    # dual_camera.cam_thread1.start()
    # dual_camera.cam_thread2.start()
    dual_camera.save_images('img1', 'img2')
    time.sleep(1)
    while True:
        time.sleep(0.2)
        im1, im2 = dual_camera.get_images()
        print(im1.shape, im2.shape)
        print(im1)
        cv2.imshow('im1', im1)
        # cv2.imshow('im2', im2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
    dual_camera.stop()  # 退出前释放资源