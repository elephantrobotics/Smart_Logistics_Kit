import threading
import numpy as np
import cv2
from cuda_simple_camera import CameraProcessor  

class RealTimeData(threading.Thread):
    def __init__(self, processor):
        super().__init__()
        self.processor = processor  # 传入 CameraProcessor 对象

    def run(self):
        # 启动摄像头的各个任务
        self.processor.show_camera()

    def get_real_time_data(self):
        # 返回 CameraProcessor 中的实时数据
        return self.processor.pose_data[0], self.processor.pose_data[4], self.processor.pose_data_dict

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
        z, ry, pose_data_dict = real_time_data.get_real_time_data()

        print(f"实时获取的 z: {z}, ry: {ry}, 数据字典: {pose_data_dict}")

        # 这里可以加个延时，避免过快输出
        # 如果你需要将这些数据用作其他任务处理，也可以在这里进行进一步的操作

        # 可以设置退出条件，例如按某个键退出
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC键退出
            break

    # 等待实时数据线程结束
    real_time_data.join()