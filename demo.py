import cv2

# RTSP流地址
rtsp_url = "rtsp://192.168.1.12:554/user=admin&password=&channel=1&stream=0"

# 使用VideoCapture对象连接RTSP流
cap = cv2.VideoCapture(rtsp_url)

# 检查是否成功打开流
if not cap.isOpened():
    print("无法连接到RTSP流")
else:
    print("RTSP流连接成功！")

# 循环读取视频流中的帧
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("无法读取视频帧，退出...")
        break
    
    # 显示当前帧
    cv2.imshow("RTSP Stream", frame)
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
