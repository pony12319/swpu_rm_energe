import cv2
import numpy as np

'''
非正方形的图片经过检测时候会被压缩破坏图像特征
把图片剪裁填充成正方形 多余的部分黑色填充
'''

# 读取图片
image = cv2.imread('media/img.png')  # 替换为您的图片路径

# 获取原始图片的尺寸
original_height, original_width = image.shape[:2]

# 给定的输入尺寸
input_height, input_width = 260, 320  # 您可以根据需要修改这个值

# 计算等比例缩放因子
scale_factor = min(input_width / original_width, input_height / original_height)

# 计算缩放后的尺寸
resized_width = int(original_width * scale_factor)
resized_height = int(original_height * scale_factor)

# 等比例缩放图片
resized_image = cv2.resize(image, (resized_width, resized_height))

# 创建一个新的黑色图片，尺寸为您想要的尺寸
new_image = np.zeros((input_height, input_width, 3), dtype=np.uint8)

# 计算居中粘贴的起始坐标
start_x = (input_width - resized_width) // 2
start_y = (input_height - resized_height) // 2

# 将缩放后的图片粘贴到黑色背景上并居中
new_image[start_y:start_y + resized_height, start_x:start_x + resized_width] = resized_image

# 显示图片
cv2.imshow('Centered Resized Image with Padding', new_image)

# 等待用户按键然后关闭窗口
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存图片
cv2.imwrite('media/resize.jpg', new_image)  # 替换为您想要保存的文件名