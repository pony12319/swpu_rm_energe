import cv2
import os

'''
读取视频,按照一定的帧间隔，抽取帧图片并且保存于frame文件夹中
修改video_path，这是你视频的路径
修改skip_frames，每隔多少帧抽取一张图片
修改folder_path，保存图片
'''

# 视频文件路径
video_path = 'C:/Users/YY/Desktop/energe_video/gaming_record/red_521.avi'

# 每隔多少帧截取一张图片
skip_frames = 100

# 创建文件夹
folder_path = 'frame'
if not os.path.exists(folder_path):
    os.makedirs(folder_path)

# 打开视频文件
cap = cv2.VideoCapture(video_path)

# 确保视频可以被读取
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

frame_count = 0
while True:
    # 读取视频的下一帧
    ret, frame = cap.read()

    # 如果正确读取帧，ret为True
    if not ret:
        print("Error: No more frames to read.")
        break

    # 每隔 skip_frames 帧保存图片
    if frame_count % skip_frames == 0:
        # 为图片构造文件名
        frame_name = f"{folder_path}/Frame_{frame_count // skip_frames + 1}.jpg"
        # 保存图片
        cv2.imwrite(frame_name, frame)
        print(f"Saved: {frame_name}")

    # 更新帧计数
    frame_count += 1

# 释放视频对象
cap.release()
print("Video processing completed.")