这是yolov8官方的代码，只是我把很多没用的都删了

本次使用yolov8识别能量机关灵感来自[同济子豪兄的个人空间-同济子豪兄个人主页-哔哩哔哩视频 (bilibili.com)](https://space.bilibili.com/1900783/channel/collectiondetail?sid=1316981)

就是关键点检测简单修改

里面很多地方也是copy他提供的代码



训练模型推荐网站[AutoDL算力云 | 弹性、好用、省钱。租GPU就上AutoDL](https://www.autodl.com/home)



打标签推荐使用labelme，虽然费时间，但是精度比较高



如何训练自己的模型：

1. 确定关键点数量、定义方式，可以有多个不同的东西，但是他们关键点数量必须一致

2. 用labelme打标签

3. 使用json_to_txt.py转换成yolo格式

4. 在ultralytics/cfg/datasets/中新建或者修改一个yaml文件 在这里指定你的数据集位置、训练数据集、验证数据集，names指的是框的类别

5. 调用train.py代码，这里修改data参数，指定为4.中的yaml文件，cfg指定为ultralytics/cfg/models/中的模型yaml文件

6. train代码能调用就说明没问题，之后也可以拿到云GPU上训练

ps：ultralytics/cfg//models/my_yolov8.yaml是张子琪学长对yolov8n-pose.yaml修改的针对小目标的模型文件，R标识别效果会好很多，但是帧率就只有一半了



每个.py文件我都说明了干啥的，请仔细阅读

训练好的模型文件我都删了，请使用数据集自行训练
