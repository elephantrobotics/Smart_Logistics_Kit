import threading
import time
import numpy as np
import rospy
import cv2
from cuda_simple_camera import CameraProcessor  

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class RealTimeData(threading.Thread):
    def __init__(self, processor):
        super().__init__()
        self.processor = processor  # 传入 CameraProcessor 对象

    def run(self):
        # 启动摄像头的各个任务
        self.processor.show_camera()

    def get_real_time_data(self):
        # 返回 CameraProcessor 中的实时数据
        return self.processor.pose_data[0], self.processor.pose_data[4], self.processor.pose_data[6][0]/960.0
class ROSController:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('robot_control', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.is_aligned = False  # 记录小车是否对齐了 Aruco

    def control_y_translation(self, perc):
        """
        控制小车在 y 轴方向上的平移，使其对齐 Aruco 的中心 perc
        """
        cmd = Twist()
        error_y = perc - 0.5  # 假设 Aruco 的中心是 perc = 0.5
        cmd.linear.y = error_y * 0.5  # 控制平移速度，根据误差调整

        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)

        # 检查是否对齐，如果误差小于阈值（比如 0.05），则认为对齐
        if abs(error_y) < 0.05:
            self.is_aligned = True  # 设置为已对齐
            print("已对齐 Aruco 中心")

    def control_x_translation(self, z):
        """
        控制小车在 x 轴方向上的移动，根据 z 的距离来决定移动
        """
        cmd = Twist()
        if z > 0.1:  # 假设 z 代表与 Aruco 的距离，小于 0.1 停止
            cmd.linear.x = 0.1  # 根据 z 值控制前进速度
            print("前进中...")
        else:
            cmd.linear.x = 0.0  # 当距离小于阈值时停止
            print("停止前进，已接近目标距离")

        # 发布控制命令
        self.cmd_vel_pub.publish(cmd)

class ControllerThread(threading.Thread):
    def __init__(self, ros_controller, real_time_data):
        super().__init__()
        self.ros_controller = ros_controller
        self.real_time_data = real_time_data

    def run(self):
        while not rospy.is_shutdown():
            # 获取实时数据
            z, ry, perc = self.real_time_data.get_real_time_data()

            # 第一个控制任务：控制 y 轴平移
            if not self.ros_controller.is_aligned:
                self.ros_controller.control_y_translation(perc)
            else:
                # 对齐完成后，执行第二个控制任务：控制 x 轴平移
                self.ros_controller.control_x_translation(z)

            # 小车控制频率
            time.sleep(0.1)  # 根据需要调整控制频率

if __name__ == "__main__":
    # 设置相机参数 (可以根据需要调整)
    camera_matrix = np.array([[785.855437, 0.000000, 451.670922], 
                              [0.000000, 584.820336, 259.056856],
                              [0.000000, 0.000000, 1.000000]], dtype=np.float32)
    dist_matrix = np.array([0.095135, -0.109279, -0.002513, -0.002433, 0.000000], dtype=np.float32)

    # 创建 CameraProcessor 实例
    camera_processor = CameraProcessor(camera_matrix, dist_matrix)

    # 启动 RealTimeData 线程，传入 CameraProcessor 实例
    real_time_data = RealTimeData(camera_processor)
    real_time_data.start()

    # 实时获取数据
    while True:
        # 获取实时数据 z, ry 和 pose_data_dict
        z, ry, perc = real_time_data.get_real_time_data()

        print(f"实时获取的 z: {z}, ry: {ry}, 百分比:{perc}")

        # 这里可以加个延时，避免过快输出
        # 如果你需要将这些数据用作其他任务处理，也可以在这里进行进一步的操作

        # 可以设置退出条件，例如按某个键退出
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键退出
            break

    # 启动控制线程
    controller_thread = ControllerThread(ros_controller, real_time_data)
    controller_thread.start()
    # 等待控制线程结束
    controller_thread.join()

    # 等待实时数据线程结束
    real_time_data.join()