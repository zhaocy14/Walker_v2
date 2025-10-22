import serial
import crcmod.predefined
import time

def read_battery_voltage(port="/dev/ttyS4", baudrate=115200, slave_addr=0x01):
    """
    读取数控直流降压电源的输入电压（即电池电压）
    :param port: 串口端口（如Windows的COM3、Linux的/dev/ttyUSB0）
    :param baudrate: 波特率（出厂默认115200）
    :param slave_addr: 从机地址（出厂默认1）
    :return: 电池电压（单位V），失败返回None
    """
    # 1. 初始化串口
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.bytesize = serial.EIGHTBITS  # 数据位8
    ser.parity = serial.PARITY_NONE  # 无校验
    ser.stopbits = serial.STOPBITS_ONE  # 停止位1
    ser.timeout = 1  # 接收超时1秒

    try:
        # 2. 打开串口
        ser.open()
        if not ser.is_open:
            print("串口打开失败")
            return None

        # 3. 构建MODBUS-RTU请求帧（读UIN寄存器：地址0x0005，读取1个寄存器）
        # 帧结构：地址码(1字节) + 功能码(1字节) + 寄存器起始地址(2字节) + 寄存器数量(2字节) + CRC(2字节)
        request_frame = [
            slave_addr,          # 从机地址
            0x03,                # 功能码：读寄存器
            0x00, 0x05,          # 寄存器起始地址：0x0005（UIN）（大端序）
            0x00, 0x01           # 读取寄存器数量：1个
        ]

        # 4. 计算CRC校验码（文档规定多项式0xA001，低位在前）
        crc16 = crcmod.predefined.Crc('modbus')
        crc16.update(bytes(request_frame))
        crc_result = crc16.digest()  # 生成2字节CRC，低位在前
        request_frame.extend(list(crc_result))  # 将CRC添加到请求帧末尾

        # 5. 发送请求
        ser.write(bytes(request_frame))
        time.sleep(0.1)  # 等待响应（根据波特率调整，避免接收不完整）

        # 6. 接收响应（正常响应长度：地址1 + 功能1 + 字节数1 + 数据2 + CRC2 = 7字节）
        response = ser.read(7)
        if len(response) != 7:
            print(f"响应数据长度异常，实际接收：{len(response)}字节")
            return None

        # 7. 解析响应数据
        # 响应帧结构：地址码(0) + 功能码(1) + 返回字节数(2) + 数据高字节(3) + 数据低字节(4) + CRC(5-6)
        if response[0] != slave_addr or response[1] != 0x03:
            print("响应地址或功能码错误")
            return None

        # 提取2字节数据，转换为十进制（大端序：高字节在前，低字节在后）
        voltage_raw = (response[3] << 8) | response[4]
        # 文档规定UIN为2位小数，故除以100
        battery_voltage = voltage_raw / 100.0

        return round(battery_voltage, 2)

    except Exception as e:
        print(f"读取失败：{str(e)}")
        return None

    finally:
        # 关闭串口
        if ser.is_open:
            ser.close()


# ------------------- 测试调用 -------------------
if __name__ == "__main__":
    # 请根据实际串口端口修改（Windows用COMx，Linux/macOS用/dev/ttyUSBx）
    battery_vol = read_battery_voltage(port="/dev/ttyS4")
    if battery_vol is not None:
        print(f"当前电池电压：{battery_vol} V")
    else:
        print("电池电压读取失败")
