import os
import ydlidar
import time

if __name__ == "__main__":
    ydlidar.os_init();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
        print(port);
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08);
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan();
        i = 0
        while True:
            try:
                while ret and ydlidar.os_isOk() :
                    r = laser.doProcessSimple(scan);
                    if r:
                        print(f"\n===== 新帧数据（时间戳: {scan.stamp}） =====")
                        print(f"扫描频率: {1.0 / scan.config.scan_time:.2f} Hz")
                        print(f"点数: {scan.points.size()}")
                        # 打印前5个点（避免输出过多）
                        for i, point in enumerate(scan.points[:5]):
                            print(f"点{i + 1}: 角度={point.angle:.2f}, 距离={point.range:.2f}m")
                        if scan.points.size() > 5:
                            print("... 更多点省略 ...")
                    # if r:
                    #     print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
                    else :
                        print("Failed to get Lidar Data")
                    time.sleep(0.05);
                laser.turnOff();
            except Exception as e:
                print(e);
                i+=1
                time.sleep(0.5)
                if i > 100:
                    break
    laser.disconnecting();