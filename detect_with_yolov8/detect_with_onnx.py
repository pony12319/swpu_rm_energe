import cv2
import numpy as np
import time
from tqdm import tqdm
from PIL import Image

import onnxruntime

from ultralytics import YOLO
from ultralytics.data.augment import LetterBox
from ultralytics.utils import ops

import matplotlib.pyplot as plt

import torch

'''
使用四点模型onnx文件识别代码并可视化，使用其他的，五点，也只需要修改一些东西即可
修改：ort_session-模型路径
preds-几个阈值，调整得到最好的效果
video_path-视频路径
'''

# 有 GPU 就用 GPU，没有就用 CPU
# device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')

kpts_shape = [4, 2]  # 关键点 shape

ort_session = onnxruntime.InferenceSession('weight/final.onnx', providers=['CPUExecutionProvider'])

model_input = ort_session.get_inputs()
input_name = [model_input[0].name]
input_shape = model_input[0].shape
input_height, input_width = input_shape[2:]

model_output = ort_session.get_outputs()
output_name = [model_output[0].name]
output_shape = model_output[0].shape

x_ratio = 1
y_ratio = 1


def process_frame(img_bgr, x_ratio, y_ratio):
    # 记录该帧开始处理的时间
    start_time = time.time()

    # 预处理-缩放图像尺寸
    img_bgr_640 = cv2.resize(img_bgr, [input_height, input_width])
    img_rgb_640 = img_bgr_640[:, :, ::-1]
    # 预处理-归一化
    input_tensor = img_rgb_640 / 255
    # 预处理-构造输入 Tensor
    input_tensor = np.expand_dims(input_tensor, axis=0)  # 加 batch 维度
    input_tensor = input_tensor.transpose((0, 3, 1, 2))  # N, C, H, W
    input_tensor = np.ascontiguousarray(input_tensor)  # 将内存不连续存储的数组，转换为内存连续存储的数组，使得内存访问速度更快
    input_tensor = torch.from_numpy(input_tensor).to(device).float()  # 转 Pytorch Tensor
    # input_tensor = input_tensor.half() # 是否开启半精度，即 uint8 转 fp16，默认转 fp32

    # ONNX Runtime 推理预测
    ort_output = ort_session.run(output_name, {input_name[0]: input_tensor.numpy()})[0]

    # 解析输出
    preds = torch.Tensor(ort_output)

    # 后处理-置信度过滤、NMS过滤
    preds = ops.non_max_suppression(preds, conf_thres=0.15, iou_thres=0.10, nc=6)
    # box-xyxy conf class xy xy xy xy
    pred = preds[0]

    #box-xyxy conf class
    pred_det = pred[:, :6].cpu().numpy()
    # print(pred_det)
    # print(type(pred[:, 5].cpu().numpy()))
    num_bbox = len(pred_det)
    bboxes_cls = pred_det[:, 5]  # 类别
    # print(type(bboxes_cls))
    bboxes_conf = pred_det[:, 4]  # 置信度

    # 目标检测框 XYXY 坐标
    # 还原为缩放之前原图上的坐标
    # print(x_ratio, y_ratio)
    pred_det[:, 0] = pred_det[:, 0] * x_ratio
    pred_det[:, 1] = pred_det[:, 1] * y_ratio
    pred_det[:, 2] = pred_det[:, 2] * x_ratio
    pred_det[:, 3] = pred_det[:, 3] * y_ratio
    pred_det[pred_det < 0] = 0  # 把小于零的抹成零
    bboxes_xyxy = pred_det[:, :4].astype('uint32')

    # 解析关键点检测预测结果
    pred_kpts = pred[:, 6:]
    bboxes_keypoints = pred_kpts.cpu().numpy()
    # 还原为缩放之前原图上的坐标
    bboxes_keypoints[:, 0] = bboxes_keypoints[:, 0] * x_ratio
    bboxes_keypoints[:, 2] = bboxes_keypoints[:, 2] * x_ratio
    bboxes_keypoints[:, 4] = bboxes_keypoints[:, 4] * x_ratio
    bboxes_keypoints[:, 6] = bboxes_keypoints[:, 6] * x_ratio

    bboxes_keypoints[:, 1] = bboxes_keypoints[:, 1] * y_ratio
    bboxes_keypoints[:, 3] = bboxes_keypoints[:, 3] * y_ratio
    bboxes_keypoints[:, 5] = bboxes_keypoints[:, 5] * y_ratio
    bboxes_keypoints[:, 7] = bboxes_keypoints[:, 7] * y_ratio
    bboxes_keypoints[bboxes_keypoints < 0] = 0
    bboxes_keypoints = bboxes_keypoints.astype('uint32')

    # OpenCV可视化
    for idx in range(num_bbox):  # 遍历每个框

        # 获取该框坐标
        bbox_xyxy = bboxes_xyxy[idx]
        keypoints = bboxes_keypoints[idx]

        # 画框
        if (bboxes_cls[idx] == 0 or bboxes_cls[idx] == 3):
            img_bgr = cv2.rectangle(img_bgr, (bbox_xyxy[0], bbox_xyxy[1]), (bbox_xyxy[2], bbox_xyxy[3]), (0, 0, 255), 2)
        else:
            img_bgr = cv2.rectangle(img_bgr, (bbox_xyxy[0], bbox_xyxy[1]), (bbox_xyxy[2], bbox_xyxy[3]), (0, 255, 0), 2)
        img_bgr = cv2.putText(img_bgr, str(int(bboxes_cls[idx]))+" "+str(bboxes_conf[idx]), (bbox_xyxy[0], bbox_xyxy[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (150, 0, 150), 2)

        # 画点
        for i in range(4):
            if(bboxes_cls[idx] == 0 or bboxes_cls[idx] == 3):
                img_bgr = cv2.circle(img_bgr, (keypoints[2 * i], keypoints[2 * i + 1]), 10, (0, 0, 255), -1)
            else:
                img_bgr = cv2.circle(img_bgr, (keypoints[2 * i], keypoints[2 * i + 1]), 10, (255, 0, 0), -1)
            # img_bgr = cv2.putText(img_bgr, str(i), (keypoints[2*i], keypoints[2*i + 1]), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (255, 0, 255), 2)

        # img_bgr = cv2.rectangle(img_bgr, (keypoints[4]-rect, keypoints[5]-rect), (keypoints[4]+rect, keypoints[5]+rect), (0, 255, 0), 2)
        img_bgr = cv2.circle(img_bgr, ((keypoints[0] + keypoints[2] + keypoints[6] + keypoints[4]) // 4,
                                       (keypoints[1] + keypoints[3] + keypoints[7] + keypoints[5]) // 4), 8,
                             (255, 0, 0), -1)

    # 记录该帧处理完毕的时间
    end_time = time.time()
    # 计算每秒处理图像帧数FPS
    FPS = 1 / (end_time - start_time)

    # 在画面上写字：图片，字符串，左上角坐标，字体，字体大小，颜色，字体粗细
    FPS_string = 'FPS  {:.2f}'.format(FPS)  # 写在画面上的字符串
    img_bgr = cv2.putText(img_bgr, FPS_string, (25, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (255, 0, 255), 2)

    return img_bgr


def read_video_frames(video_path):
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    while True:
        # 尝试读取下一帧
        ret, frame = cap.read()
        # 检查是否读取成功
        if ret:
            # 确保frame不是None
            if frame is not None:
                original_width, original_height = frame.shape[:2]
                width_height = max(original_width, original_height)
                new_image = Image.new('RGB', (width_height, width_height), color='black')

                left = (width_height - original_width) // 2
                top = (width_height - original_height) // 2
                new_image.paste(Image.fromarray(frame), (left, top))
                img = np.array(new_image) #把原图片填充为正方形

                x_ratio = img.shape[1] / input_width
                y_ratio = img.shape[0] / input_height

                # print(img.shape[1], input_width, x_ratio)
                # print(img.shape[0], input_height, y_ratio)

                # 在这里处理帧数据，例如显示或保存帧
                print(f"img type: {img.dtype}")
                pre = process_frame(img, x_ratio, y_ratio)
                cv2.imshow("pre", img)
                cv2.waitKey(1)

                # 按 'q' 键退出循环
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Warning: Frame is None. Skipping processing.")
        else:
            # 如果没有更多的帧或读取失败，则退出循环
            print("End of video or error occurred. Exiting loop.")
            break

    # 释放视频对象
    cap.release()
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()


# 使用函数读取视频
video_path = 'C:/Users/YY/Desktop/energe_video/gaming_record/red_520.avi'  # 替换为你的视频文件路径
read_video_frames(video_path)