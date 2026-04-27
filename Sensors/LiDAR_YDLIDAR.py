import os,sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import ydlidar
import time
import matplotlib
from sklearn.cluster import KMeans
from Sensors.SensorConfig import *
import threading
import math
from PIL import Image
import cv2
import numpy as np

class LiDAR_YDLIDAR:
    def __init__(self, text_show: bool = False, cv_show: bool = False):
        # 初始化雷达
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ttyUSB0"
        for key, value in ports.items():
            port = value
            print(f"使用端口: {port}")
        self.lidar = ydlidar.CYdLidar()
        self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.lidar_settings()
        self.ret = self.lidar.initialize()
        if self.ret:
            self.ret = self.lidar.turnOn()
            print("YDLIDAR初始化成功并开始扫描")
        else:
            print("YDLIDAR启动失败")


        # for leg position storage
        self.scan_raw_data = np.zeros((1, 1))
        self.kmeans = KMeans(n_clusters=2)
        self.left_leg = np.zeros((2))
        self.right_leg = np.zeros((2))
        self.scope = 2

        # for 2D scan images sizes
        self.size = SCAN_SIZE
        self.scan_img = np.zeros((self.size,self.size))

        # new version of filtering useless data
        self.walker_box = WALKER_BOX_BOUNDARY_VERTICAL

        self.leg_img = np.zeros((WALKER_TOP_BOUNDARY + WALKER_BOTTOM_BOUNDARY,
                                 WALKER_LEFT_BOUNDARY + WALKER_RIGHT_BOUNDARY))
        # center point is the geometry center of the walker
        self.center_point = np.array([WALKER_TOP_BOUNDARY+HUMAN_TO_LIDAR,WALKER_LEFT_BOUNDARY])

        # obstacle part
        # five regions to detect the obstacle
        # 0 means no obstacle, else means yes
        self.ob_front_left = 0
        self.ob_front = 0
        self.ob_front_right = 0
        self.ob_left = 0
        self.ob_right = 0
        self.ob_back = 0

        # show & save
        self.text_show = text_show
        self.cv_show = cv_show
        self.save_freq = 30  # save frequency in how many scans

        # 退出标志与线程
        self.running = True
        self.reading_thread = threading.Thread(target=self.scan, args=())
        self.reading_thread.daemon = True
        self.reading_thread.start()

    def lidar_settings(self,):
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 30.0)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 20.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.01)
        self.lidar.setlidaropt(ydlidar.LidarPropIntenstiy, False)  # 若需强度信息，设为True
        print("YDLIDAR参数配置完成")

    def turn_to_img(self, original_list: list, is_save:bool=False) -> None:
        """
        turn the scan list to an image
        :param is_save: whether to save the image
        :param original_list: a list of the [angle, distance, quality]
        """
        self.scan_img[:] = 0
        for i in range(len(original_list)):
            theta = original_list[i][1]
            distance = original_list[i][2] * SCAN_UNIT # unit: mm
            # turn distance*theta -> x-y axis in the scan image
            index_x = int(distance * math.sin(theta) + HALF_SIZE)
            index_y = int(distance * math.cos(theta) + HALF_SIZE)
            index_x = min(max(index_x, 0), self.size - 1)
            index_y = min(max(index_y, 0), self.size - 1)
            self.scan_img[index_y, index_x] = 1
        self.scan_img = np.flipud(self.scan_img)
        if is_save:
            start_time = time.time()
            im = np.copy(self.scan_img)
            im[HALF_SIZE - 5:HALF_SIZE + 5, HALF_SIZE - 5:HALF_SIZE + 5] = 1
            # 保存图像，确保图像格式正确（这里将二值图转换为RGB以便正常保存）
            save_img = cv2.cvtColor((im * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            # 储存到根部目录下的log下的lidar文件夹中
            if not os.path.exists("./log/lidar/"):
                os.makedirs("./log/lidar/")
            cv2.imwrite("./log/lidar/lidar_img.jpg", save_img)
            used_time = time.time() - start_time
            print("LiDAR图像已保存到 ./log/lidar/lidar_img.jpg, 用时%.4f秒" % used_time)
        if self.cv_show:
            # not recommended
            size = int(self.size * self.scope)
            im = Image.fromarray(im)
            im = im.resize((size, size), Image.BILINEAR)
            cv2.imshow("LiDAR", im)
            cv2.waitKey(1)

    def detect_leg(self, kmeans: KMeans, is_save: bool = False) -> (np.ndarray, np.ndarray):
        """
        Analyze the top-view map. Using Kmeans to
        :param kmeans: A Kmeans module
        :param is_save
        :return:
        """
        # leg-img is the detecting walking area
        # idea is simple, first you need the distance between the lidar center point and the boundary you define
        # then as the half_size of the detecting area is known
        # then convert to the matrix axis:
        self.leg_img[:, :] = self.scan_img[HALF_SIZE - WALKER_TOP_BOUNDARY:HALF_SIZE + WALKER_BOTTOM_BOUNDARY,
        HALF_SIZE - WALKER_LEFT_BOUNDARY:HALF_SIZE + WALKER_RIGHT_BOUNDARY]

        # remove the box area detection
        # basic idea is simply removing the box related rows
        # as you don't need to count the points outside the walker boundary right?
        self.leg_img[
            0:WALKER_TOP_BOUNDARY + self.walker_box, :] = 0  # this line is to wipe out the scanning inside the main box

        # then you need to filter out the rear wheel area
        # actually, most of the time the leg will block the rear wheel
        # but when there's no user, the lidar will detect it
        rear_wheel_row_idx = WALKER_TOP_BOUNDARY + WALKER_REAR_WHEEL_ROW_IDX
        rear_left_wheel_col_idx = WALKER_REAR_WHEEL_COL_IDX
        rear_right_wheel_col_idx = -WALKER_REAR_WHEEL_COL_IDX
        rear_wheel_width = WALKER_REAR_WHEEL_WIDTH

        # clear left rear wheel column
        self.leg_img[
            rear_wheel_row_idx - WALKER_REAR_WHEEL_RADIUS:rear_wheel_row_idx + WALKER_REAR_WHEEL_RADIUS,
            rear_left_wheel_col_idx - rear_wheel_width:rear_left_wheel_col_idx + rear_wheel_width
        ] = 0

        # clear right rear wheel column
        self.leg_img[
            rear_wheel_row_idx - WALKER_REAR_WHEEL_RADIUS:rear_wheel_row_idx + WALKER_REAR_WHEEL_RADIUS,
            rear_right_wheel_col_idx - rear_wheel_width:rear_right_wheel_col_idx + rear_wheel_width
        ] = 0

        # then do the clustering(k-means)
        if self.leg_img.sum() >= 2:
            index = np.where(self.leg_img == 1)
            sample = np.c_[index[0], index[1]]
            kmeans.fit(sample)
            center_1 = np.around(kmeans.cluster_centers_[0]).astype(int)
            center_2 = np.around(kmeans.cluster_centers_[1]).astype(int)
            if self.cv_show or is_save:
                # to show the leg position in the image
                self.leg_img[center_1[0] - 3: center_1[0] + 3, center_1[1] - 3:center_1[1] + 3] = 1
                self.leg_img[center_2[0] - 3:center_2[0] + 3, center_2[1] - 3:center_2[1] + 3] = 1
                # # to show the LiDAR point in the image
                # self.leg_img[self.walker_tb - 1:self.walker_tb + 1,
                # self.walker_lb - 1:self.walker_lb + 1] = 0
                im_show = self.leg_img
                im_show[
                    WALKER_TOP_BOUNDARY - 5:WALKER_TOP_BOUNDARY + 5, WALKER_LEFT_BOUNDARY - 5:WALKER_LEFT_BOUNDARY + 5] = 1
                if self.cv_show:
                    # transform to Image to change the size of the print image
                    im_show = Image.fromarray(im_show)
                    img_scope = 5
                    img_size_row = (WALKER_TOP_BOUNDARY + WALKER_BOTTOM_BOUNDARY) * img_scope
                    img_size_column = (WALKER_LEFT_BOUNDARY + WALKER_RIGHT_BOUNDARY) * img_scope
                    im_show = im_show.resize((img_size_column, img_size_row), Image.BILINEAR)
                    im_show = np.array(im_show)
                    cv2.imshow("leg", im_show)
                    cv2.waitKey(1)
                if is_save:
                    start_time = time.time()
                    save_img = cv2.cvtColor((im_show * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
                    # 储存到根部目录下的log下的lidar文件夹中
                    if not os.path.exists("./log/lidar/"):
                        os.makedirs("./log/lidar/")
                    cv2.imwrite("./log/lidar/leg_img.jpg", save_img)
                    used_time = time.time() - start_time
                    print("脚图像已保存到 ./log/lidar/leg_img.jpg, 用时%.4f秒" % used_time)
            if center_1[1] < center_2[1]:
                self.left_leg = self.center_point - center_1
                self.right_leg = self.center_point - center_2
            else:
                self.left_leg = self.center_point - center_2
                self.right_leg = self.center_point - center_1
        else:
            infinite_far = np.array([-INFINITE_FAR, -INFINITE_FAR])
            self.left_leg = infinite_far
            self.right_leg = infinite_far
        return self.left_leg, self.right_leg

    def detect_obstacle(self,is_shown:bool=False):
        """
        set the circular regions to detect the obstacle
        """
        obs_dis = OBSTACLE_DISTANCE
        obstacle_area = self.scan_img[HALF_SIZE - WALKER_TOP_BOUNDARY - obs_dis:
                            HALF_SIZE + WALKER_BOTTOM_BOUNDARY + obs_dis,
                        HALF_SIZE - WALKER_LEFT_BOUNDARY - obs_dis:
                        HALF_SIZE + WALKER_RIGHT_BOUNDARY + obs_dis]
        # obs_dis -= 30
        self.ob_front_left = obstacle_area[0:obs_dis, 0:obs_dis].sum()
        self.ob_front = obstacle_area[0:obs_dis, obs_dis:-obs_dis].sum()
        self.ob_front_right = obstacle_area[0:obs_dis, -obs_dis:-1].sum()
        # # left and right are blocked, can not well detect obstacle
        self.ob_left = obstacle_area[obs_dis:-1, 0:obs_dis].sum()
        self.ob_right = obstacle_area[obs_dis:-1, -obs_dis:-1].sum()
        # # detect the back area to avoid crash the back
        # self.ob_back = obstacle_area[-int(obs_dis+20):-1, obs_dis:-obs_dis].sum()
        self.ob_back = obstacle_area[-int(obs_dis):-1, obs_dis:-obs_dis].sum()
        if is_shown:
            print("Front_Left:%i, Front:%i, Front_Right:%i, Left:%i, Right:%i, Back:%i"%
                  (self.ob_front_left,self.ob_front,self.ob_front_right,self.ob_left,self.ob_right, self.ob_back))

    def scan(self):
        try_times = 0
        scan_time_for_save = 1000
        is_save_lidar = False   # whether save the whole img
        is_save_leg = False      # whether save the leg scanning area
        while self.running:
            try:
                while self.running and self.ret and ydlidar.os_isOk():
                    scan = ydlidar.LaserScan()
                    r = self.lidar.doProcessSimple(scan)
                    if r:
                        scan_time_for_save += 1
                        temp_list = []
                        for i, point in enumerate(scan.points):
                            temp_list.append([point.intensity, point.angle, point.range])
                        self.scan_raw_data = np.array(temp_list)
                        self.detect_obstacle(False)
                        # to save the image every certain scans
                        if scan_time_for_save > self.save_freq:
                            scan_time_for_save = 0
                            self.turn_to_img(temp_list, is_save=is_save_lidar)
                            self.detect_leg(self.kmeans, is_save=is_save_leg)
                        else:
                            self.turn_to_img(temp_list, is_save=False)
                            self.detect_leg(self.kmeans, is_save=False)

                        if self.text_show:
                            print("left leg idx:", self.left_leg, "right leg idx:", self.right_leg)

                    else:
                        print("获取数据失败")
                    time.sleep(0.05)
            except Exception as e:
                print(f"扫描过程中出现错误: {e}")
                time.sleep(0.5)
                try_times += 1
                if try_times > 100:
                    self.lidar.turnOff()
                    self.lidar.disconnecting()
                    break

    def get_lidar_data(self) -> np.ndarray:
        """
        Get the top-view image.
        :return: A SCAN_SIZE x SCAN_SIZE 2D numpy array
        """
        return self.scan_img

    def get_leg_data(self) -> (np.ndarray, np.ndarray):
        """
        Get the left leg and right leg position
        :return: 2 one dimensional numpy array: the X-Y coordinate of the left leg and the right leg.
        """
        return self.left_leg, self.right_leg

    def get_leg_image(self) -> np.ndarray:
        """
        Get the top-view image of the operational area(leg) image
        :return:
        """
        return self.leg_img

    def stop(self):
        """外部调用：请求线程退出并关闭雷达"""
        self.running = False
        if hasattr(self, 'lidar'):
            self.lidar.turnOff()
            self.lidar.disconnecting()

if __name__ == "__main__":
    lidar = LiDAR_YDLIDAR(text_show=True)
    while True:
        time.sleep(1)
    # just for checking the LiDAR
    # lidar_instance = LiDAR(is_zmq=False)
    # lidar_instance.python_scan(is_show=True)
    # print(lidar_instance.port_name)