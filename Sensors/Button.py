import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import subprocess
import time
import threading

class Button(object):
    def __init__(self, wPi: int = 13):
        """
        Button sensor using system command 'gpio read {wPi}'
        :param wPi: GPIO wPi number (wiringpi numbering). default is 13 (GPIO 40, GPIO1_B0)
        """
        super(Button, self).__init__()

        self.wpi = wPi
        self.pressed = False
        # 检查gpio命令是否存在
        self._check_gpio_command()

        # 启动检测线程
        self.thread = threading.Thread(target=self.is_pressed, args=(), daemon=True)
        self.thread.start()

    def _check_gpio_command(self):
        """检查系统中是否存在gpio命令（wiringOP的命令行工具）"""
        try:
            subprocess.run(
                ["gpio", "-v"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            raise RuntimeError(
                "错误：系统中未找到gpio命令，请先安装wiringOP底层库！\n"
                "安装方法：git clone https://github.com/orangepi-xunlong/wiringOP.git && cd wiringOP && sudo ./build"
            )

    def _read_gpio(self) -> int:
        """
        执行gpio read命令并解析结果
        :return: 引脚电平值（0/1）
        """
        try:
            # 执行gpio read {wPi}命令，捕获输出
            result = subprocess.run(
                ["gpio", "read", str(self.wpi)],
                stdout=subprocess.PIPE,  # 捕获标准输出
                stderr=subprocess.PIPE,  # 捕获标准错误
                text=True,               # 输出为字符串（而非字节）
                check=False              # 不主动抛出命令执行异常（比如引脚不存在）
            )
            # 解析输出：去除空格和换行符，转换为整数
            value = int(result.stdout.strip())
            return value
        except (ValueError, subprocess.SubprocessError) as e:
            print(f"读取GPIO失败：{e}，标准错误：{result.stderr.strip()}")
            return 1  # 默认返回高电平，避免程序崩溃

    def is_pressed(self) -> bool:
        """
        循环检测按钮状态（低电平有效，active low）
        :return: True if pressed, False otherwise
        """
        while True:
            state = self._read_gpio()
            self.pressed = not state  # 按钮按压时为低电平（0），故pressed=True
            time.sleep(0.02)  # 20ms检测一次，平衡响应速度和资源占用

if __name__ == "__main__":
    # 检查是否为root权限（gpio命令需要sudo，否则可能读取失败）

    try:
        button = Button(wPi=13)
        print("按钮检测已启动，按Ctrl+C退出...")
        while True:
            print("Button pressed:", button.pressed)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序退出中...")
        sys.exit(0)