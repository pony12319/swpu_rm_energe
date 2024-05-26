import os
import json


'''
将labelme生成的json文件转换成yolo格式的归一化txt文件
先打框还是先打点，点的顺序如何都没关系 只要标签对应信息正确，顺序无所谓

标签定义规则：
每个扇叶四个点，两个框
    0(或者lt)-左上角
    1(或者lb)-左下角
    2(或者rb)-右下角
    3(或者rt)-右上角

    RR- 红色待打击 id-0
    RW- 红色已打击 id-1
    Rr- 红色R标 id-2
    BR- 蓝色待打击 id-3
    BW- 蓝色已打击 id-4
    Br- 蓝色R标 id-5

'''

# 指定包含 JSON 文件的目录路径
json_directory = 'C:/Users/YY/Desktop/total_energy'  # 替换成您的 JSON 文件所在的文件夹路径
# 指定输出 TXT 文件的目录路
output_directory = 'result'  # 替换成您希望保存 TXT 文件的文件夹路径

# 确保输出目录存在
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# 遍历指定目录下的所有 JSON 文件
for json_file_name in os.listdir(json_directory):
    if json_file_name.endswith('.json'):
        json_file_path = os.path.join(json_directory, json_file_name)
        txt_file_name = json_file_name[:-5] + '.txt'  # 从 JSON 文件名中移除 '.json' 并加上 '.txt'
        txt_file_path = os.path.join(output_directory, txt_file_name)

        # 读取 JSON 文件
        with open(json_file_path, 'r') as f:
            data = json.load(f)
            image_height = data["imageHeight"]
            image_width = data["imageWidth"]

        #检查并且跳过错误的json文件
        error_flag = 0
        rect_num = 0
        lt_num = 0
        lb_num = 0
        rb_num = 0
        rt_num = 0

        for check_shape in data['shapes']:
            #检查label信息是否有误
            if check_shape['label'] in ["RR", "RW", "Rr", "BR", "BW", "Br"]:
                rect_num = rect_num + 1
            if check_shape['label'] in ["lt", '0']:
                lt_num = lt_num + 1
            if check_shape['label'] in ["lb", '1']:
                lb_num = lb_num + 1
            if check_shape['label'] in ["rb", '2']:
                rb_num = rb_num + 1
            if check_shape['label'] in ["rt", '3']:
                rt_num = rt_num + 1

            #是否有有效、正确的group_id
            if not isinstance(check_shape['group_id'], int):
                print(f"Skipping {json_file_name} due to shape without valid group_id.")
                error_flag = 1

        if not (rect_num == lt_num == lb_num == rb_num == rt_num):
            error_flag = 1

        if error_flag == 1:
            print(f"{json_file_name} due to label info error")
            continue

        # 初始化输出行列表
        output_lines = []
        # 存储处理过的shape的索引
        processed_indices = []

        # 遍历 JSON 中的 shapes
        for shape_index, shape in enumerate(data['shapes']):
            if shape_index in processed_indices:
                continue
            # 处理矩形形状
            if shape['shape_type'] == 'rectangle':
                points = shape['points']
                x1, y1 = points[0][0], points[0][1]
                x2, y2 = points[1][0], points[1][1]
                norm_x = (x2 + x1) / (2 * image_width)
                norm_y = (y2 + y1) / (2 * image_height)
                norm_width = (x2 - x1) / image_width
                norm_height = (y2 - y1) / image_height
                output_line = f"{shape['group_id']} {norm_x} {norm_y} {norm_width} {norm_height}"
                output_lines.append(output_line)
                processed_indices.append(shape_index)

                # 处理lt点
                for shape_index1, shape1 in enumerate(data['shapes']):
                    if shape_index1 in processed_indices:
                        continue
                    points1 = shape1['points']
                    if shape1['shape_type'] == 'point' and (shape1['label'] == '0' or shape1['label'] == 'lt') and x1-5 <= points1[0][0] <= x2+5 and y1-5 <= points1[0][1] <= y2+5:
                        output_line = f" {points1[0][0] / image_width} {points1[0][1] / image_height}"
                        output_lines.append(output_line)
                        processed_indices.append(shape_index1)

                        # 处理lb点
                        for shape_index2, shape2 in enumerate(data['shapes']):
                            if shape_index2 in processed_indices:
                                continue
                            points2 = shape2['points']
                            if shape2['shape_type'] == 'point' and (shape2['label'] == '1' or shape2['label'] == 'lb') and x1-5 <= points2[0][0] <= x2+5 and y1-5 <= points2[0][1] <= y2+5:
                                output_line = f" {points2[0][0] / image_width} {points2[0][1] / image_height}"
                                output_lines.append(output_line)
                                processed_indices.append(shape_index2)

                                # 处理rb点
                                for shape_index3, shape3 in enumerate(data['shapes']):
                                    if shape_index3 in processed_indices:
                                        continue
                                    points3 = shape3['points']
                                    if shape3['shape_type'] == 'point' and (shape3['label'] == '2' or shape3['label'] == 'rb') and x1-5 <= points3[0][0] <= x2+5 and y1-5 <= points3[0][1] <= y2+5:
                                        output_line = f" {points3[0][0] / image_width} {points3[0][1] / image_height}"
                                        output_lines.append(output_line)
                                        processed_indices.append(shape_index3)

                                        # 处理rt点
                                        for shape_index4, shape4 in enumerate(data['shapes']):
                                            if shape_index4 in processed_indices:
                                                continue
                                            points4 = shape4['points']
                                            if shape4['shape_type'] == 'point' and (shape4['label'] == '3' or shape4['label'] == 'rt') and x1-5 <= points4[0][0] <= x2+5 and y1-5 <= points4[0][1] <= y2+5:
                                                output_line = f" {points4[0][0] / image_width} {points4[0][1] / image_height}\n"
                                                output_lines.append(output_line)
                                                processed_indices.append(shape_index4)


        # 写入 TXT 文件
        with open(txt_file_path, 'w') as f:
            for line in output_lines:
                f.write(line)

print("all json ared converted")