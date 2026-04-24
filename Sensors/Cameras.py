import datetime
import time
import cv2
import os
import glob
import threading


class DualCamera:
    def __init__(self, show_fps=False, root_path='./'):
        self.running = True

        ports = self._find_ports()
        if len(ports) < 2:
            raise RuntimeError(f"只找到 {len(ports)} 个有效摄像头，需要 2 个")
        print(f"Found camera ports: {ports}")

        self.camera1 = cv2.VideoCapture(ports[0])
        self.camera2 = cv2.VideoCapture(ports[1])
        self.camera1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.camera2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.set_frame_rate()

        self.image1 = None
        self.image2 = None
        self.root_path = root_path
        self.set_root_path(root_path)
        self.show_fps = show_fps

        self.cam_thread1 = threading.Thread(target=self.run, args=(self.camera1, 0), daemon=True)
        self.cam_thread2 = threading.Thread(target=self.run, args=(self.camera2, 1), daemon=True)

    def _find_ports(self):
        ports = []
        nodes = sorted(glob.glob('/dev/video*'))
        if not nodes:
            nodes = [f'/dev/video{i}' for i in range(10)]
        for node in nodes:
            if not node.startswith('/dev/video'):
                continue
            try:
                idx = int(node.replace('/dev/video', ''))
            except ValueError:
                continue
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                for _ in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        ports.append(idx)
                        break
                    time.sleep(0.05)
            cap.release()
            if len(ports) >= 2:
                break
        return ports

    def set_frame_rate(self, frame_rate=60):
        # 分辨率提升为 640x480
        # 若双路 60fps 出现卡顿/丢帧，可改为 frame_rate=30
        width, height = 640, 480

        self.camera1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera1.set(cv2.CAP_PROP_FPS, frame_rate)

        self.camera2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.camera2.set(cv2.CAP_PROP_FPS, frame_rate)

        self.camera1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        self.camera2.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    def set_root_path(self, root_path):
        os.makedirs(root_path, exist_ok=True)
        self.root_path = root_path

    def end(self):
        self.running = False

    def stop(self):
        self.running = False
        self.camera1.release()
        self.camera2.release()
        cv2.destroyAllWindows()

    def run(self, camera, cam_id):
        if not camera.isOpened():
            print(f'USB camera {cam_id} is not opened')
            return

        frame_count = 0
        start_time = time.time()
        while self.running:
            ret, frame = camera.read()
            if not ret:
                print(f'Can not read data from camera {cam_id}')
                if not self.running:
                    break
                for _ in range(10):
                    time.sleep(0.05)
                    ret, frame = camera.read()
                    if ret:
                        break
                else:
                    print(f'Failed to read data from camera {cam_id} after 10 attempts')
                    break

            # frame = cv2.rotate(frame, cv2.ROTATE_180)

            if cam_id == 0:
                self.image1 = frame
            else:
                self.image2 = frame

            frame_count += 1
            elapsed_time = time.time() - start_time
            if self.show_fps and elapsed_time > 1:
                fps = frame_count / elapsed_time
                print(f'USB camera {cam_id} FPS: {fps:.2f}')
                frame_count = 0
                start_time = time.time()

        camera.release()
        print(f"USB camera {cam_id} thread ends")

    def get_images(self):
        return self.image1, self.image2

    def save_images(self, file_name1, file_name2, log=True):
        if self.image1 is None or self.image2 is None:
            print("Warning: image1 or image2 is None, skip saving.")
            return

        base_path = os.path.expanduser('~/Walker_v2/log/camera')
        today = datetime.datetime.now().strftime('%Y-%m-%d')
        save_dir = os.path.join(base_path, today)
        os.makedirs(save_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime('%H%M')

        frame1 = self.image1
        frame2 = self.image2

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


if __name__ == "__main__":
    dual_camera = DualCamera(show_fps=True)
    dual_camera.cam_thread1.start()
    dual_camera.cam_thread2.start()
    time.sleep(4)

    try:
        while True:
            time.sleep(5)
            dual_camera.save_images('img1', 'img2')
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        dual_camera.stop()