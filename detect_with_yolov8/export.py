from ultralytics import YOLO
import onnxruntime
onnxruntime.get_device()

'''
转.pt文件为.onnx文件
只需修改模型地址即可
'''

model = YOLO('weight/final.pt')

model.export(format='onnx')
