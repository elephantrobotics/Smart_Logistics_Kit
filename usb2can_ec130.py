import serial
import rospy
import time

# 键盘控制相关
import sys, select, termios, tty

from geometry_msgs.msg import Twist

#获取键值初始化，读取终端相关属性
settings = termios.tcgetattr(sys.stdin)
def getKey():
    '''获取键值函数'''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_and_fixRetract(str):
    '''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(str)

class AutoRecharger():
    def __init__(self):
        #创建节点
        rospy.init_node("auto_recharger") 
        # usb2can 打开串口
        self.serial_conn = serial.Serial("/dev/ttyACM0", 115200) # 跟270串口容易搞混，需要对EC130进行串口固定
        #红外信号的数量
        self.red_count=0

        #按键控制说明
        self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start Navigation to find charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
        """

        self.pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.buffer = bytearray()  # 存储当前读取的字节

    def crc16_check(self,data):
        # return crc16.crc16xmodem(bytes(data))

        temp = 0xFFFF  # 初始值
        k = 0  # 索引
        while k < len(data):  # 要计算的字节数
            temp ^= data[k]  # 输入要检验字节，异或运算结果
            i = 0
            while i < 8:  # 单字节位移数
                if (temp & 0x01) == 0:  # 检查运算结果最后一位是否为零
                    temp >>= 1  # 向右移一位
                    i += 1  # 计数器加1
                else:
                    temp >>= 1  # 向右移一位
                    temp ^= 0xA001  # 异或运算结果
                    i += 1  # 计数器加1
            k += 1  # 下一个校验字节
        return temp    
    
    def close_serial(self):
        """
        关闭串口
        """
        if self.serial_conn:
            self.serial_conn.close()

    def read_serial_data(self):
        """
        读取串口数据并解析
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            raise Exception("Serial port is not open")
        
        while True:
            if self.serial_conn.in_waiting > 0:   # 判断缓冲区是否有足够的数据
                byte = self.serial_conn.read(1)   # 读取一个字节
            
                if len(self.buffer) < 2:
                    self.buffer.extend(byte)  
                    if len(self.buffer) == 2:
                        if len(self.buffer) == 2:
                            # 如果帧头为 0xFE 0xFE，则开始接收数据
                            if self.buffer[0] != 0xFE or self.buffer[1] != 0xFE:
                                # 如果不是有效的帧头，则清空缓冲区并跳到下次循环
                                self.buffer.clear()
                                continue  # 继续等待下一个字节                   
                else:
                    self.buffer.extend(byte)

                    # print("缓冲区内容:", ' '.join(f'{b:02x}' for b in self.buffer)) # debug

                    # 如果缓冲区字节长度大于等于 14 字节（数据帧长度）
                    if len(self.buffer) >= 14:
                        
                        print("Received Frame (Hex):", ' '.join(f'{byte:02x}' for byte in self.buffer))

                        crc_from_frame = self.buffer[12] << 8 | self.buffer[13]  # 高字节在前
                        data = self.buffer[4:12]

                        # print(f"crc_from_frame:{crc_from_frame:#06x},crc16_check:{self.crc16_check(self.buffer[:12]):#06x}") # debug
                        
                        # CRC16校验
                        if crc_from_frame == self.crc16_check(self.buffer[:12]):
                            # 校验通过，进行数据赋值
                            # 解析 X、Y 和 Z 速度
                            x_speed_raw = ((data[0] << 8) | data[1])  # X速度的原始数据
                            z_speed_raw = ((data[4] << 8) | data[5])  # Z速度的原始数据

                            # 将原始数据转换为浮动数值，并考虑正负
                            if x_speed_raw & 0x8000:  # 如果最高位为1，表示负数
                                x_speed_raw = -((65536 - x_speed_raw) & 0xFFFF)  # 补码转换为负数

                            if z_speed_raw & 0x8000:  # 如果最高位为1，表示负数
                                z_speed_raw = -((65536 - z_speed_raw) & 0xFFFF)  # 补码转换为负数

                            # 转换单位为 m/s 和 rad/s
                            x_speed = x_speed_raw / 1000.0  # X速度单位为 m/s
                            y_speed = 0  # Y速度为0
                            z_speed = z_speed_raw / 1000.0  # Z速度单位为 rad/s

                            infrared = data[6]  # 红外数据
                            raw_current = data[7]  # 电流数据

                            if raw_current > 32767:  # 无符号数大于 32767 表示负值（因为最大值是 65535）
                                # 转换为负数
                                actual_current = -(65536 - raw_current) / 30.0
                            else:
                                # 正数直接转换
                                actual_current = raw_current / 30.0

                            # 处理红外数据，提取第七位和第八位
                            infrared_seventh_bit = (infrared >> 6) & 0x01  # 提取第七位
                            infrared_eighth_bit = (infrared >> 7) & 0x01   # 提取第八位

                            # 打印或处理数据
                            print(f"X Speed: {x_speed:.3f}, Y Speed: {y_speed}, Z Speed: {z_speed:.3f}, "
                                f"Actual Current: {actual_current:.3f} A, Infrared: {infrared}")
                            
                            # 打印或处理红外位信息
                            print(f"Seventh Bit: {infrared_seventh_bit}, Eighth Bit: {infrared_eighth_bit}")

                            # 清空缓冲区，准备下一帧数据
                            self.buffer.clear()
                            return True
                        
                        else:
                            print("CRC校验失败!")
 
            # time.sleep(0.03)  # 每 30 毫秒检查一次串口数据

    def autoRecharger(self, key):
        while not rospy.is_shutdown():
            if self.read_serial_data(): # 获取串口数据
                pass # 根据串口数据来做出反应
        self.close_serial()

        # if key=='q' or key=='Q':
        # # 存在3路以上的红外信号,小车姿态接近于对准充电桩,无需导航
        #     if self.red_count>=3:
        #         self.start_autocharger() #开始回充
        #         print_and_fixRetract('已捕获到高强度红外信号,使用红外信号对接.(High-intensity infrared signals have been captured and are docked using infrared signals.)')
        #     else:
        #         if self.robot['RED']==1:
        #             self.chargeflag=1
                    
if __name__ == '__main__':
    try:
        AutoRecharger=AutoRecharger() #创建自动回充类
        print_and_fixRetract(AutoRecharger.tips)

        while not rospy.is_shutdown():
            key = getKey() #获取键值
            AutoRecharger.autoRecharger(key) #开始自动回充功能
            if (key == '\x03'):
                print_and_fixRetract('自动回充功能已关闭.(Quit AutoRecharger.)')#Auto charging quit
                break #Ctrl+C退出自动回充功能

    except rospy.ROSInterruptException:
        print_and_fixRetract('exception')