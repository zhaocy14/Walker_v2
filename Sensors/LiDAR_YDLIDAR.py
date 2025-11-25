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
    def __init__(self, is_show: bool = False):
        # 初始化雷达
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
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
        self.half_size = HALF_SIZE
        self.scan_img = np.zeros((self.size,self.size))

        # old version of filtering useless data
        self.column_boundary = COLUMN_BOUNDARY
        self.filter_theta = FILTER_THETA
        self.bottom_boundary = BOTTOM_BOUNDARY
        # new version of filtering useless data
        self.walker_top_boundary = WALKER_TOP_BOUNDARY
        self.walker_bottom_boundary = WALKER_BOTTOM_BOUNDARY
        self.walker_left_boundary = WALKER_LEFT_BOUNDARY
        self.walker_right_boundary = WALKER_RIGHT_BOUNDARY
        self.leg_img = np.zeros((self.walker_top_boundary+self.walker_bottom_boundary,
                                 self.walker_left_boundary+self.walker_right_boundary))
        # center point is the geometry center of the walker
        self.center_point = np.array([WALKER_TOP_BOUNDARY+CENTER_TO_LIDAR,WALKER_LEFT_BOUNDARY])
        self.is_show = is_show


        self.theta_flag = 0
        # obstacle part
        # five regions to detect the obstacle
        # 0 means no obstacle, else means yes
        self.ob_front_left = 0
        self.ob_front = 0
        self.ob_front_right = 0
        self.ob_left = 0
        self.ob_right = 0
        # obstacle detection threshold
        self.obstacle_distance = OBSTACLE_DISTANCE  # front obstacle distance

        # event
        self.lidar_process_event = threading.Event()
        self.lidar_process_event.clear()
        self.lidar_process_event.set()

        # threading
        self.reading_thread = threading.Thread(target=self.scan, args=())
        self.reading_thread.start()

    def lidar_settings(self,):
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
        self.lidar.setlidaropt(ydlidar.LidarPropIntenstiy, False)  # 若需强度信息，设为True
        print("YDLIDAR参数配置完成")

    def turn_to_img(self, original_list: list, show: bool = False, save:bool = False) -> None:
        """
        turn the scan list to an image
        :param original_list: a list of the [angle, distance, quality]
        :param show: to display the top-view map

        """
        self.scan_img[:] = 0
        for i in range(len(original_list)):
            theta = original_list[i][1]
            # theta = -theta / 180 * math.pi
            distance = original_list[i][2] * 100 # unit: m->cm, cm is enough, mm will not bring more scan point
            # print(theta,distance)
            # distance = original_list[i][2]  # unit: mm
            # turn distance*theta -> x-y axis in the scan image
            index_x = int(distance * math.cos(theta) + self.half_size)
            index_y = int(distance * math.sin(theta) + self.half_size)
            index_x = min(max(index_x, 0), self.size - 1)
            index_y = min(max(index_y, 0), self.size - 1)
            # if index_x >= 2 and index_x <= self.size - 2:
            #     if index_y >=2 and index_y <= self.size -2:
            #         img[index_x-2:index_x+2,index_y-2:index_y+2] = 1
            self.scan_img[index_x, index_y] = 1
        im = np.copy(self.scan_img)
        im[self.half_size - 3:self.half_size + 3, self.half_size - 3:self.half_size + 3] = 1
        size = int(self.size * self.scope)
        im = Image.fromarray(im)
        im = im.resize((size, size), Image.BILINEAR)
        im = np.array(im)
        if show:
            cv2.imshow("LiDAR", im)
            cv2.waitKey(1)
        if save:
            # 保存图像，确保图像格式正确（这里将二值图转换为RGB以便正常保存）
            save_img = cv2.cvtColor((im * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            cv2.imwrite("./img.jpg", save_img)

    def detect_leg(self, kmeans: KMeans, show: bool = False) -> (np.ndarray, np.ndarray):
        """
        Analyze the top-view map. Using Kmeans to
        :param kmeans: A Kmeans module
        :param show:
        :return:
        """
        # leg-img is the detecting walking area
        self.leg_img[:, :] = self.scan_img[self.half_size - self.walker_top_boundary:self.half_size + self.walker_bottom_boundary,
                        self.half_size - self.walker_left_boundary:self.half_size + self.walker_right_boundary]
        self.leg_img[0:self.walker_top_boundary+7, :] = 0 # this line is to wipe out the scanning inside the main box

        # what is this ???
        # detect_leg_img = np.zeros((self.leg_img.shape))
        # detect_leg_img[self.walker_top_boundary:-1, 15:-15] = self.leg_img[18:-1, 15:-15]
        if self.leg_img.sum() >= 2:
            index = np.where(self.leg_img == 1)
            sample = np.c_[index[0], index[1]]
            kmeans.fit(sample)  # TODO: this part will consume 90% of CPU
            center_1 = np.around(kmeans.cluster_centers_[0]).astype(int)
            center_2 = np.around(kmeans.cluster_centers_[1]).astype(int)
            if show:
                # to show the leg position in the image
                self.leg_img[center_1[0] - 2: center_1[0] + 2, center_1[1] - 2:center_1[1] + 2] = 1
                self.leg_img[center_2[0] - 2:center_2[0] + 2, center_2[1] - 2:center_2[1] + 2] = 1
                # to show the LiDAR point in the image
                self.leg_img[self.walker_top_boundary - 1:self.walker_top_boundary + 1,
                self.walker_left_boundary - 1:self.walker_left_boundary + 1] = 1
                # im_show = im + img
                im_show = self.leg_img
                # transform to Image to change the size of the print image
                im_show = Image.fromarray(im_show)
                img_scope = 5
                img_size_row = (self.walker_top_boundary + self.walker_bottom_boundary) * img_scope
                img_size_column = (self.walker_left_boundary + self.walker_right_boundary) * img_scope
                im_show = im_show.resize((img_size_column, img_size_row), Image.BILINEAR)
                im_show = np.array(im_show)
                cv2.imshow("leg", im_show)
                cv2.waitKey(1)
            if center_1[1] < center_2[1]:
                self.left_leg = self.center_point - center_1
                self.right_leg = self.center_point - center_2
            else:
                self.left_leg = self.center_point - center_2
                self.right_leg = self.center_point - center_1
        else:
            infinite_far = np.array([-180, -180])
            self.left_leg = infinite_far
            self.right_leg = infinite_far
        return self.left_leg, self.right_leg

    def detect_obstacle(self,is_shown:bool=False):
        """
        seperate the detecting area into several part
        However, the low lidar is blocked by the surroundings
        Useless now
        """
        obstacle_area = self.scan_img[self.half_size - self.walker_top_boundary - self.obstacle_distance:
                            self.half_size + self.bottom_boundary + 1,
                        self.half_size - self.walker_left_boundary - self.obstacle_distance:
                        self.half_size + self.walker_right_boundary + self.obstacle_distance + 1]
        self.ob_front_left = obstacle_area[0:self.obstacle_distance, 0:self.obstacle_distance].sum()
        self.ob_front = obstacle_area[0:self.obstacle_distance, self.obstacle_distance:-self.obstacle_distance].sum()
        self.ob_front_right = obstacle_area[0:self.obstacle_distance, -self.obstacle_distance:-1].sum()
        # # left and right are blocked, can not well detect obstacle
        # self.ob_left = obstacle_area[self.obstacle_distance:-1, 0:self.obstacle_distance].sum()
        # self.ob_right = obstacle_area[self.obstacle_distance:-1, -self.obstacle_distance:-1].sum()
        if is_shown:
            print("Front_Left:%i, Front:%i, Front_Right:%i, Left:%i, Right:%i"%
                  (self.ob_front_left,self.ob_front,self.ob_front_right,self.ob_left,self.ob_right))

    def scan(self):
        try_times = 0
        while True:
            try:
                self.lidar_process_event.wait()
                while self.ret and ydlidar.os_isOk():
                    scan = ydlidar.LaserScan()
                    r = self.lidar.doProcessSimple(scan)
                    if r:
                        # print(f"\n===== 新帧数据（时间戳: {scan.stamp}） =====")
                        # print(f"扫描频率: {1.0 / scan.config.scan_time:.2f} Hz")
                        # print(f"点数: {scan.points.size()}")
                        temp_list = []
                        for i, point in enumerate(scan.points):
                            temp_list.append([point.intensity, point.angle, point.range])
                        self.scan_raw_data = np.array(temp_list)
                        self.turn_to_img(temp_list)
                        # detect obstacle, but not appropriate for the low lidar
                        # self.detect_obstacle(True)
                        # detect leg
                        self.detect_leg(self.kmeans)
                        if self.is_show:
                            print(self.left_leg,self.right_leg)

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

if __name__ == "__main__":
    lidar = LiDAR_YDLIDAR(is_show=True)
    # just for checking the LiDAR
    # lidar_instance = LiDAR(is_zmq=False)
    # lidar_instance.python_scan(is_show=True)
    # print(lidar_instance.port_name)


