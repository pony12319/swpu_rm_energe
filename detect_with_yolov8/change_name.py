import os
import shutil


'''
按序号批量命名.jpg .json文件的脚本
只需要修改两个文件路径 original_folder_path  result_folder_path
以及按照需求修改文件命名格式 new_jpg_filename new_json_filename
'''

# 设置原始文件夹路径
original_folder_path = 'C:/Users/YY/Desktop/稀有的十环'  # 替换为您原始文件的文件夹路径
# 设置结果文件夹路径
result_folder_path = 'C:/Users/YY/Desktop/稀有的十环/res'  # 替换为您希望保存新文件的文件夹路径

# 确保结果文件夹存在
os.makedirs(result_folder_path, exist_ok=True)

# 分别获取原始文件夹中的所有.jpg和.json文件
jpg_files = [f for f in os.listdir(original_folder_path) if f.endswith('.jpg')]
json_files = [f for f in os.listdir(original_folder_path) if f.endswith('.json')]

# 创建一个用于匹配.jpg和.json文件的字典
file_pairs = {}
for file in jpg_files:
    base_name = os.path.splitext(file)[0]  # 获取基本名称，不含扩展名
    json_match = base_name + '.json'
    if json_match in json_files:
        file_pairs[file] = json_match

# 对匹配的文件对进行排序
sorted_file_pairs = sorted(file_pairs.items(), key=lambda x: x[0])

# 重置文件名计数器
file_number = 1

# 复制并重命名文件
for jpg_file, json_file in sorted_file_pairs:
    #在这里修改你的命名格式
    new_jpg_filename = f"ten_{file_number}.jpg"
    new_json_filename = f"ten_{file_number}.json"

    old_jpg_file = os.path.join(original_folder_path, jpg_file)
    old_json_file = os.path.join(original_folder_path, json_file)
    new_jpg_file = os.path.join(result_folder_path, new_jpg_filename)
    new_json_file = os.path.join(result_folder_path, new_json_filename)

    shutil.copy2(old_jpg_file, new_jpg_file)
    shutil.copy2(old_json_file, new_json_file)
    print(f"Copied and renamed '{jpg_file}' to '{new_jpg_filename}' and '{json_file}' to '{new_json_filename}'")

    file_number += 1

print("All corresponding .jpg and .json files have been copied and renamed successfully.")