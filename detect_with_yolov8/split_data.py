import os
import shutil
import random


'''
将制作好的.jpg/png 和.txt格式的yolo数据集自动分配成yolo训练所需的格式
修改data_directory，里面包含图片和txt文件
修改total_energy_yolo_path，分配好的数据集
修改split_index，训练集验证集比例（一般8:2)
'''

# 设置随机种子以获得可重复的结果（可选）
random.seed(42)

# 数据文件目录
data_directory = 'C:/Users/YY/Desktop/total_energy'

# 创建总文件夹和子文件夹
total_energy_yolo_path = 'C:/Users/YY/Desktop/total_energy_yolo'
images_path = os.path.join(total_energy_yolo_path, "images")
labels_path = os.path.join(total_energy_yolo_path, "labels")

# 创建train和test子文件夹
train_images_path = os.path.join(images_path, "train")
test_images_path = os.path.join(images_path, "test")
train_labels_path = os.path.join(labels_path, "train")
test_labels_path = os.path.join(labels_path, "test")

# 确保所有需要的文件夹都存在
os.makedirs(images_path, exist_ok=True)
os.makedirs(labels_path, exist_ok=True)
os.makedirs(train_images_path, exist_ok=True)
os.makedirs(test_images_path, exist_ok=True)
os.makedirs(train_labels_path, exist_ok=True)
os.makedirs(test_labels_path, exist_ok=True)

# 获取数据目录下所有的.jpg、.png文件和对应的.txt文件
image_files = {f for f in os.listdir(data_directory) if f.endswith(('.jpg', '.png'))}

# 随机打乱图片文件列表
image_list = list(image_files)
random.shuffle(image_list)

# 按照8:2的比例分割数据集
split_index = int(len(image_list) * 0.8)

# 分割图片文件
train_images = image_list[:split_index]
test_images = image_list[split_index:]

# 定义一个函数来复制图片和对应的.txt文件到相应的文件夹
def copy_corresponding_files(image_files, image_target_path, label_target_path, data_dir):
    for image_file in image_files:
        image_basename = os.path.splitext(image_file)[0]
        # 复制图片文件
        shutil.copy(os.path.join(data_dir, image_file), image_target_path)
        # 复制对应的.txt文件
        txt_file = f"{image_basename}.txt"
        if txt_file in os.listdir(data_dir):
            shutil.copy(os.path.join(data_dir, txt_file), label_target_path)

# 复制train和test的图片及标签文件
copy_corresponding_files(train_images, train_images_path, train_labels_path, data_directory)
copy_corresponding_files(test_images, test_images_path, test_labels_path, data_directory)

print("文件已成功分配并复制到相应的文件夹。")