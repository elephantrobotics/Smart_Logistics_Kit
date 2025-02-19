import serial
import struct
import time

# 配置串口
SERIAL_PORT = '/dev/ttyUSB0'  # 根据你的设备修改
BAUDRATE = 9600  # 波特率设置为9600
TIMEOUT = 1  # 超时时间设置为1秒

# CAN 数据解析
def parse_can_data(data):
    """
    解析8字节CAN数据帧。
    假设数据内容为:2字节X轴速度,2字节Y轴速度,2字节Z轴速度,1字节红外标志,1字节充电电流。
    """
    if len(data) != 8:
        print("数据帧长度不正确")
        return None

    # 解析各个字段
    x_speed, y_speed, z_speed, ir_flag, charge_current = struct.unpack('<hhhbB', data)
    
    return {
        'X轴速度': x_speed,
        'Y轴速度': y_speed,
        'Z轴速度': z_speed,
        '红外标志': ir_flag,
        '充电电流': charge_current
    }

def read_serial_data(serial_port):
    """读取串口数据并解析"""
    while True:
        if serial_port.in_waiting > 0:
            # 假设数据帧格式为 [帧ID (4字节) | 数据 (8字节)]
            frame_header = serial_port.read(4)  # 读取4字节帧头（帧ID）
            if len(frame_header) == 4:
                frame_id = struct.unpack('<I', frame_header)[0]  # 解包得到帧ID
                if frame_id == 0x182:
                    # 如果帧ID为0x182，读取8字节数据
                    data = serial_port.read(8)
                    if len(data) == 8:
                        can_data = parse_can_data(data)
                        if can_data:
                            print(f"接收到帧ID: {hex(frame_id)}，数据解析结果：", can_data)
                    else:
                        print("数据长度不匹配，跳过该帧。")
                else:
                    print(f"跳过帧ID: {hex(frame_id)}")
            else:
                print("帧头读取失败，跳过此帧。")

# 打开串口
def open_serial():
    try:
        serial_port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"串口 {SERIAL_PORT} 已打开，波特率：{BAUDRATE}")
        return serial_port
    except Exception as e:
        print(f"打开串口失败: {e}")
        return None

def close_serial(serial_port):
    """关闭串口"""
    if serial_port and serial_port.is_open:
        serial_port.close()
        print("串口已关闭。")

def main():
    # 打开串口
    serial_port = open_serial()
    if not serial_port:
        return

    try:
        # 读取并解析数据
        read_serial_data(serial_port)
    except KeyboardInterrupt:
        print("手动中止程序。")
    finally:
        # 关闭串口
        close_serial(serial_port)

if __name__ == '__main__':
    main()