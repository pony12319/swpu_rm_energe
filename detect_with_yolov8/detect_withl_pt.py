# 测试图片
from ultralytics import YOLO
import cv2
import numpy as np
import sys

'''
使用.pt文件检测图片
'''

# 读取命令行参数
weight_path = 'weight/scu2.pt'
media_path = "img/image_5.jpg"

# 加载模型
model = YOLO(weight_path)

# 获取类别
objs_labels = model.names  # get class labels
# print(objs_labels)

# 类别的颜色
class_color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),(255, 0, 0), (0, 255, 0)]
# 关键点的顺序
class_list = ["RR", "RW", "Rr", "BR", "BW", "Br"]

# 关键点的颜色
keypoint_color = [(255, 0, 0), (0, 255, 0),(255, 0, 0), (0, 255, 0)]

# 读取图片
frame = cv2.imread(media_path)
# rotate
# 检测
result = list(model(frame, conf=0.3, stream=True))[0]  # inference，如果stream=False，返回的是一个列表，如果stream=True，返回的是一个生成器
boxes = result.boxes  # Boxes object for bbox outputs
boxes = boxes.cpu().numpy()  # convert to numpy array
# print(boxes)

# 遍历每个框
for box in boxes.data:
    l, t, r, b = box[:4].astype(np.int32)  # left, top, right, bottom
    conf, id = box[4:]  # confidence, class
    id = int(id)
    # 绘制框
    cv2.rectangle(frame, (l, t), (r, b), (0, 0, 255), 2)
    # 绘制类别+置信度（格式：98.1%）
    cv2.putText(frame, f"{objs_labels[id]} {conf * 100:.1f}", (l, t - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 255), 1)

# 遍历keypoints
keypoints = result.keypoints  # Keypoints object for pose outputs
keypoints = keypoints.cpu().numpy()  # convert to numpy array

# draw keypoints, set first keypoint is red, second is blue
for keypoint in keypoints.data:
    for i in range(len(keypoint)):
        x, y = keypoint[i]
        x, y = int(x), int(y)
        cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
        #cv2.putText(frame, f"{keypoint_list[i]}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, keypoint_color[i], 2)

# save image
print(frame.shape)
cv2.imshow("result.jpg", frame)
cv2.waitKey(0)
