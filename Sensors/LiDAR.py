import serial
import threading
import time
import rplidar
import
from Sensors.SensorConfig import *
from Sensors.SensorFunctions import *


class LiDAR(object):
    def __init__(self, is_zmq: bool = True):
        """
        Search the LiDAR for scanning. It includes two version: python and C.
        Usually use the python version. While the C version is to use the ZMQ to communicate between C and python code.
        :param is_zmq: a bool value to decide whether you use python version(False) or C version(True)
        """
        super().__init__()
        if not is_zmq:
            self.port_name, _ = detect_serials(port_key=LIDAR_LOCATION_LOW, sensor_name="LiDAR Python")
            self.python_lidar = rplidar.RPLidar(self.port_name)
        else:
            self.port_name, _ = detect_serials(port_key=LIDAR_LOCATION_HIGH, sensor_name="LiDAR")
        # store the data
        self.scan_data_list = []

    def python_scan(self, is_show: bool = False, retry = 10):
        """ just for checking LiDAR serial port"""
        try:
            info = self.python_lidar.get_info()
            health = self.python_lidar.get_health()
            print(info)
            print(health)
            for i, scan in enumerate(self.python_lidar.iter_scans()):
                self.scan_data_list = scan
                if is_show:
                    print(self.scan_data_list)
        except BaseException as be:
            if retry > 0:
                print(f"{be}, retrying")
                self.python_lidar.stop()
                self.python_lidar.stop_motor()
                self.python_lidar.reset()
                self.python_lidar.start_motor()
                return self.python_scan(is_show=is_show, retry=retry-1)
            else:
                print("Failed to start the lidar.")


class LiDARProcessor(object):
    def __init__(self, is_show: bool = False, is_zmq: bool = False):
        """
            Processing the LiDAR data. The original LiDAR data is a list of data points. Each data point contains [Angle,
        Distance, Quality]. Angle is 0~360 degree. Distance unit is 1mm. In this module, the data list is converted to
        a rectangle represented by a 2D numpy array. The 2D numpy array is the top-view map centered the LiDAR.
            The most important data needs to be concerned is the map around the LiDAR in the operation area inside the
        Walker.
            You can get the LiDAR data by using get_lidar_data()
        :param is_show: To demo the top-view map
        :param is_zmq: False:activate the python version. True:activate the C version.
        """

        # LiDAR object
        self.rplidar = LiDAR.LiDAR(is_zmq=False).python_lidar

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
        # zmq part
        self.rzo = ReceiveZMQ.get_instance()
        self.zmq_temp_list = []
        self.zmq_scan_list = []
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
        self.reading_thread = threading.Thread(target=self.python_scan, args=(self.is_show,))
        self.reading_thread.start()

    def turn_to_img(self, original_list: list, show: bool = False) -> None:
        """
        turn the scan list to an image
        :param original_list: a list of the [angle, distance, quality]
        :param show: to display the top-view map

        """
        self.scan_img[:] = 0
        for i in range(len(original_list)):
            theta = original_list[i][1]
            theta = -theta / 180 * math.pi
            distance = original_list[i][2] / 10  # unit: mm->cm, cm is enough, mm will not bring more scan point
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
        if show:
            im = np.copy(self.scan_img)
            im[self.half_size - 3:self.half_size + 3, self.half_size - 3:self.half_size + 3] = 1
            size = int(self.size * self.scope)
            im = Image.fromarray(im)
            im = im.resize((size, size), Image.BILINEAR)
            im = np.array(im)
            cv2.imshow("LiDAR", im)
            cv2.waitKey(1)

    def detect_leg_version(self, kmeans: KMeans, show: bool = False) -> [np.ndarray, np.ndarray]:
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

    def python_scan(self, show: bool = False):
        """
        use python rplidar library to scan
        :param show: show the scanning image
        :param file_path:
        :return:
        """
        while True:
            try:
                self.lidar_process_event.wait()
                for i, scan in enumerate(self.rplidar.iter_scans(max_buf_meas=10000)):
                    self.scan_raw_data = np.array(scan)
                    # establish a 2D scan map
                    self.turn_to_img(scan, show=show)
                    # detect obstacle, but not appropriate for the low lidar
                    # self.detect_obstacle(True)
                    # detect leg
                    self.detect_leg_version(self.kmeans, show=show)
                    # time.sleep
                    # time.sleep(0.12) # to slow down the detection rate and reduce memory consumption
                    # self.rplidar.clean_input()

            except BaseException as be:
                # pass
                self.rplidar.clean_input()
                self.rplidar.stop()
                # if stop motor, this will
                self.rplidar.stop_motor()
                # self.rplidar.disconnect()

    # No need from the ZMQ, just use python rplidar
    def zmq_get_one_round(self, zmq_data: dict):
        """use zmq to get lidar data from C++"""
        theta = float(zmq_data["theta"])
        dist = float(zmq_data["dist"])
        quality = float(zmq_data["q"])
        if theta < self.theta_flag:
            self.zmq_scan_list = self.zmq_temp_list
            self.zmq_temp_list = []
        self.theta_flag = theta
        self.zmq_temp_list.append([quality, theta, dist])

    def zmq_scan(self, show: bool = False, is_record: bool = False, file_path: str = DATA_PATH):
        """main procedure of scanning legs and the environment"""
        if is_record:
            data_path = file_path + os.path.sep + "leg.txt"
            file_leg = open(data_path, 'w')
        while True:
            for scan in self.rzo.startLidar():
                self.zmq_get_one_round(scan)
                if len(self.zmq_temp_list) == 1:
                    self.scan_raw_data = np.array(self.zmq_scan_list)
                    self.turn_to_img(self.zmq_scan_list)
                    self.detect_leg_version(self.kmeans, show=show)
                    self.detect_obstacle(is_shown=show)
                    if is_record:
                        time_index = time.time()
                        leg_data = np.r_[self.left_leg, self.right_leg]
                        write_data = leg_data.tolist()
                        write_data.insert(0, time_index)
                        file_leg.write(str(write_data) + '\n')
                        file_leg.flush()

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
    # just for checking the LiDAR
    lidar_instance = LiDAR(is_zmq=False)
    lidar_instance.python_scan(is_show=True)
    # print(lidar_instance.port_name)
