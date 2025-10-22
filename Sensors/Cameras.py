import datetime
import queue
import time
import cv2
import os
import numpy
import threading


class DualCamera:
    def __init__(self, show_fps=False, root_path='./'):
        # 初始化两个摄像头

        # 先确认相机ID
        camera_port = [-1, -1]
        for i in range(10):  # 尝试0到9的摄像头索引，可根据实际情况调整范围
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                if camera_port[0] == -1:
                    camera_port[0] = i
                    cap.release()
                elif camera_port[1] == -1:
                    camera_port[1] = i
                    cap.release()
                    break
        print("Camera Ports:", camera_port)
        self.camera1 = cv2.VideoCapture(camera_port[0])
        self.camera2 = cv2.VideoCapture(camera_port[1])
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

        # 为每个摄像头创建线程
        self.cam_thread1 = threading.Thread(target=self.run, args=(self.camera1, 0))
        self.cam_thread2 = threading.Thread(target=self.run, args=(self.camera2, 1))
        self.cam_thread_end = False

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
        # 结束线程标志
        self.cam_thread_end = True

    def run(self, camera, cam_id):
        # 检查摄像头是否打开
        if not camera.isOpened():
            print(f'USB camera {cam_id} is not opened')
            raise RuntimeError(f"USB camera {cam_id} is not opened")

        frame_count = 0
        start_time = time.time()
        while True and not self.cam_thread_end:
            ret, frame = camera.read()
            if not ret:
                print(f'Can not read data from camera {cam_id}')
                # 如果读取失败，重复尝试10次
                for try_i in range(10):
                    ret, frame = camera.read()
                    if ret:
                        break
                    else:
                        if try_i == 9:
                            # 如果10次都失败，尝试重新配置摄像头
                            print(f'Failed to read data from camera {cam_id} after 10 attempts')
                            self.reconfig_cameras()
                            # 重新读取数据
                            ret, frame = camera.read()


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
        camera.release()
        # cv2.destroyAllWindows()
        print(f"USB camera {cam_id} thread ends")

    def get_images(self):
        # 获取当前图像
        return self.image1, self.image2

    def save_images(self, file_path1, file_path2, log=True):
        frame1 = self.image1
        frame2 = self.image2
        save_path1 = os.path.join(self.root_path, file_path1)
        save_path2 = os.path.join(self.root_path, file_path2)
        print(save_path1, save_path2)

        # 确保保存为 JPG 格式
        if not save_path1.endswith('.jpg'):
            save_path1 = f'{save_path1}.jpg'
        if not save_path2.endswith('.jpg'):
            save_path2 = f'{save_path2}.jpg'

        try:
            # 保存图像
            cv2.imwrite(save_path1, frame1)
            cv2.imwrite(save_path2, frame2)
        except Exception as e:
            print(f"Error occurred while saving the files: {e}")

        if log:
            print(f"Save USB camera 1 frame to {save_path1} successfully.")
            print(f"Save USB camera 2 frame to {save_path2} successfully.")

    def find_camera_port(self, reconfig=False):
        """
        When the cameras lost connection, try to release camera and scan the camera port again.
        :param reconfig: whether to reconfigure the camera
        :return:
        """
        if reconfig:
            self.camera1.release()
            self.camera2.release()
        time.sleep(0.5)
        # try other ids
        camera_port = [-1, -1]
        # 动态搜寻多个id
        for i in range(100):  # 尝试0到9的摄像头索引，可根据实际情况调整范围
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                if camera_port[0] == -1:
                    camera_port[0] = i
                    cap.release()
                elif camera_port[1] == -1:
                    camera_port[1] = i
                    cap.release()
                    break
        self.camera1 = cv2.VideoCapture(camera_port[0])
        self.camera2 = cv2.VideoCapture(camera_port[1])
        self.set_frame_rate()
        print("Reconfigured cameras.")


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
    # 启动线程
    dual_camera.cam_thread1.start()
    dual_camera.cam_thread2.start()
    dual_camera.save_images('img1.jpg', 'img2.jpg')
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


