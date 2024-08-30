#!/usr/bin/env python3
#ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from interfaces.msg import Armor

import cv2

# #yolov8
import numpy as np 
import time 
from tqdm import tqdm
from PIL import Image as PILImage #避免和sensor_msgs冲突
import onnxruntime
from ultralytics import YOLO
from ultralytics.data.augment import LetterBox
from ultralytics.utils import ops
import matplotlib.pyplot as plt
import torch


class ImageProNode(Node):
    # Node constructor
    def __init__(self) -> None:
        super().__init__("img_pro_node")
        # self.get_logger().info("yolo node start!")
 
        self._class_to_color = {}
        self.cv_bridge = CvBridge()
 
        # subscribers
        self._image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        #装甲板数据发布者
        self._armor_pub = self.create_publisher(
            Armor,
            '/energe_detect/armor',
            qos_profile_sensor_data
        )


        self.init_yolo()
 
    #初始化yolov8模型
    def init_yolo(self):
        self.device = torch.device('cpu')
        self.ort_session = onnxruntime.InferenceSession('src/img_pro/weights/total_data.onnx', providers=['CPUExecutionProvider'])

        self.model_input = self.ort_session.get_inputs()
        self.input_name = [self.model_input[0].name]
        self.input_shape = self.model_input[0].shape
        self.input_height, self.input_width = self.input_shape[2:]

        self.model_output = self.ort_session.get_outputs()
        self.output_name = [self.model_output[0].name]
        self.output_shape = self.model_output[0].shape

        self.get_logger().info("inityolo success!")

    def image_callback(self, msg) -> None:
        try:
            # self.get_logger().info("rec success!")
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            #将图像变换为等宽高
            original_width, original_height = cv_image.shape[:2]
            size = max(original_width, original_height)
            new_image = PILImage.new('RGB', (size, size), color='black')
            left = (size - original_width) // 2
            top = (size - original_height) // 2
            new_image.paste(PILImage.fromarray(cv_image), (left, top)) #使图像居中
            img = np.array(new_image)
            
            x_ratio = img.shape[1] / self.input_width
            y_ratio = img.shape[0] / self.input_height

            self.res = self.process_frame(img, x_ratio, y_ratio)

            if len(self.res) > 0:
                #发布识别信息
                self.armor_msg = Armor()
                #R坐标
                self.armor_msg.sign_r.x.data = (self.res[0])
                self.armor_msg.sign_r.y.data = (self.res[1])
                #装甲板四个坐标
                self.armor_msg.p_lt.x.data = (self.res[2])
                self.armor_msg.p_lt.y.data = (self.res[3])
                self.armor_msg.p_lb.x.data = (self.res[4])
                self.armor_msg.p_lb.y.data = (self.res[5])
                self.armor_msg.p_rb.x.data = (self.res[6])
                self.armor_msg.p_rb.y.data = (self.res[7])
                self.armor_msg.p_rt.x.data = (self.res[8])
                self.armor_msg.p_rt.y.data = (self.res[9])
                #时间
                self.armor_msg.header.stamp = msg.header.stamp

                self._armor_pub.publish(self.armor_msg)
                # self.get_logger().info("img_pro success!")
            else:
                pass
            

        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def if_appear_once(self, arr):
        #理论上说 RR Rr BR Br只会出现一次 不等于一次都认为识别错误
        #        0  2  3 5
        if np.sum(arr == 0) == 1 and np.sum(arr == 2) == 1:
            return True
        elif np.sum(arr == 3) == 1 and np.sum(arr == 5) == 1:
            return True
        else:
            return False


    #逐帧处理函数 返回：class、五点坐标
    def process_frame(self, img_bgr, x_ratio, y_ratio):
        # 预处理-缩放图像尺寸
        img_bgr_640 = cv2.resize(img_bgr, [self.input_height, self.input_width])
        img_rgb_640 = img_bgr_640[:,:,::-1]
        # 预处理-归一化
        input_tensor = img_rgb_640 / 255
        # 预处理-构造输入 Tensor
        input_tensor = np.expand_dims(input_tensor, axis=0) # 加 batch 维度
        input_tensor = input_tensor.transpose((0, 3, 1, 2)) # N, C, H, W
        input_tensor = np.ascontiguousarray(input_tensor)   # 将内存不连续存储的数组，转换为内存连续存储的数组，使得内存访问速度更快
        input_tensor = torch.from_numpy(input_tensor).to(self.device).float() # 转 Pytorch Tensor
        # input_tensor = input_tensor.half() # 是否开启半精度，即 uint8 转 fp16，默认转 fp32 

        # ONNX Runtime 推理预测
        self.ort_output = self.ort_session.run(self.output_name, {self.input_name[0]: input_tensor.numpy()})[0]
        
        # 解析输出
        preds = torch.Tensor(self.ort_output)

        # 后处理-置信度过滤、NMS过滤
        preds = ops.non_max_suppression(preds, conf_thres=0.25, iou_thres=0.7, nc=6)
        #0-5 box-xy xy conf class    6-13 lt_xy lb_xy rb_xy rt_xy
        pred = preds[0]
        # print(pred)
        # print("\n")

        predict = pred[:, :].cpu().numpy()
        num_bbox = len(predict)
        #判断识别效果是否符合要求
        Ifcontinue = self.if_appear_once(predict[:, 5])
        # print(Ifcontinue)
        result = np.zeros(10)

        if Ifcontinue == True:
            # 取出结果 待打击、R标
            for i in range(num_bbox):
                # R标点的处理
                if predict[i, 5] == 2 or predict[i, 5] == 5:
                    # 清除负值并转换类型，只针对当前行
                    row = predict[i, :].copy()  # 避免原地修改原始数组
                    row[row < 0] = 0
                    r_x = x_ratio * (row[6] + row[8] + row[10] + row[12]) * 0.25
                    r_y = y_ratio * (row[7] + row[9] + row[11] + row[13]) * 0.25
                    result[0] = r_x
                    result[1] = r_y

                # 扇叶四点坐标处理
                elif predict[i, 5] == 0 or predict[i, 5] == 3:
                    # 清除负值并转换类型，只针对当前行
                    row = predict[i, :].copy()  # 避免原地修改原始数组
                    row[row < 0] = 0
                    # 关键点坐标，并添加一个元组到列表
                    result[2] = row[6] * x_ratio
                    result[3] = row[7] * y_ratio
                    result[4] = row[8] * x_ratio
                    result[5] = row[9] * y_ratio
                    result[6] = row[10] * x_ratio
                    result[7] = row[11] * y_ratio
                    result[8] = row[12] * x_ratio
                    result[9] = row[13] * y_ratio

        return result


 
def main():
    rclpy.init()
    node = ImageProNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()