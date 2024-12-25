import cv2
import os

# RTSP流地址
rtsp_url = "rtsp://192.168.1.12/user=admin&password=&channel=1&stream=1"

# 使用VideoCapture对象连接RTSP流
cap = cv2.VideoCapture(rtsp_url)
write_path = "/home/xujg/yolo_rknn/temp"
os.makedirs(write_path, exist_ok=True)
# 检查是否成功打开流
if not cap.isOpened():
    print("无法连接到RTSP流")
else:
    print("RTSP流连接成功！")

# 用于保存帧的计数器
frame_count = 0
save_interval = 30  # 每隔 30 帧保存一次

# 循环读取视频流中的帧
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("无法读取视频帧，退出...")
        break
    
    # 显示当前帧
    cv2.imshow("RTSP Stream", frame)
    
    # 保存帧
    if frame_count % save_interval == 0:  # 每隔一定帧数保存
        filename = f"frame_{frame_count}.jpg"
        cv2.imwrite(os.path.join(write_path, filename), frame)
        print(f"保存帧: {filename}")
    
    frame_count += 1  # 更新计数器
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
