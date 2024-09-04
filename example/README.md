# 目录说明

| 目录                | 对应教程章节                       |
| ------------------ | --------------------------------- |
| lenet              | 手写数字识别--飞桨（PaddlePaddle）  | 
| resnet18           | ResNet18网络--PyTorch              | 
| picodet            | 飞桨（PaddlePaddle） FastDeploy    | 
| ppocrv3            | PP-ORCv3                        |
| ppocrv4            | PP-ORCv4                        | 
| ppyoloe            | PP-YOLOE                        |
| yolov5             | YOLOv5(目标检测)                 |
| yolov5_seg         | YOLOv5(实例分割)                 |
| yolov8             | YOLOv8                 |
| yolov10            | YOLOv10目标检测                 |
| garbage_detection  | 垃圾检测和识别               |


# 文件说明

scaling_frequency.sh 是系统CPU，DDR，NPU频率修改脚本，例如：

```sh
# USAGE: ./fixed_frequency.sh -c {chip_name} [-h]"
# "  -c:  chip_name, such as rv1126 / rk3588"
# "  -h:  Help"
sudo bash scaling_frequency.sh -c rk3568
```