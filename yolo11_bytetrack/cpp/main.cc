#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <map>
#include <string>

#include "yolo11.h"
#include "image_utils.h"
#include "file_utils.h"
#include "BYTETracker.h"
#include "easy_timer.h"
#include "aair.h"
#include "detector.h"
#include "drone_objlocation.h"
#include "udp_data.h"

static const unsigned char colors[19][3] = {
    {54, 67, 244}, {99, 30, 233}, {176, 39, 156}, {183, 58, 103}, {181, 81, 63},
    {243, 150, 33}, {244, 169, 3}, {212, 188, 0}, {136, 150, 0}, {80, 175, 76},
    {74, 195, 139}, {57, 220, 205}, {59, 235, 255}, {7, 193, 255}, {0, 152, 255},
    {34, 87, 255}, {72, 85, 121}, {158, 158, 158}, {139, 125, 96}
};

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s yolov11.rknn 0\n", argv[0]);
        printf("Usage: %s yolov11.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_path = argv[2];

    // 创建日志文件和控制台输出的sink
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/application_log.txt", true);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    
    // 合并两个sink
    std::vector<spdlog::sink_ptr> sinks = {file_sink, console_sink};
    
    // 创建logger并指定使用两个sinks
    auto logger = std::make_shared<spdlog::logger>("multi_sink_logger", sinks.begin(), sinks.end());
    
    // 设置日志级别
    logger->set_level(spdlog::level::info);

    // 输出初始化信息
    logger->info("Starting the yolo11+bytetrack");
    logger->info("Model path: {}", model_path);
    logger->info("Device path: {}", device_path);

    int ret;
    TIMER timer;
    cv::Mat frame, image;
    BYTETracker tracker(30, 90);
    std::vector<Object> objects;

    YoloDetector detector;

    ret = detector.init(model_path);
    if (ret != 0) {
        logger->error("init_yolo11_model fail! ret={} model_path={}", ret, model_path);
        return -1;
    }

    cv::VideoCapture cap;
    // 摄像头
    if (isdigit(device_path[0])) {
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);
        if (!cap.isOpened()) {
            logger->error("Error: Could not open camera. Camera ID: {}", camera_id);
            return -1;
        }
    } else {
        // 视频文件或者其他
        cap.open(argv[2]);
        if (!cap.isOpened()) {  

            logger->error("Error: Could not open video file. Path: {}", device_path);
            return -1;
        }
    }

    // 初始化AAIR接收器
    AAIRReceiver aair_receiver("192.168.1.19", 12345, "192.168.1.19", 23456);
    aair_receiver.start();
    logger->info("AAIRReceiver started with IP 192.168.1.19");

    // 初始化解算类
    DroneObjlocation geo_location = DroneObjlocation();
    geo_location.set_parameter((uint16_t)640, (uint16_t)480, 640, 640, 300, 240); // img_width, img_height, fx, fy, cx, cy

    while (true) 
    {
        if (!cap.read(frame)) {  
            logger->error("Error: Could not read frame from the camera or video");
            break;
        }

        // 获取最新的AAIR数据
        AAIR cur_aair = aair_receiver.getCurAAIR();

        timer.tik();
        ret = detector.infer(frame, objects);
        if (ret != 0) {
            logger->error("Inference failed! ret={}", ret);
            break;
        }

        // 调用 ByteTrack 更新跟踪信息
        vector<STrack> tracked_objects = tracker.update(objects);

        timer.tok();
        timer.print_time("yolo11_bytetrack");

        // 记录处理时间
        logger->info("Processing time for Yolo + ByteTrack: {:.2f} ms", timer.get_time());

        for (const auto& tracked : tracked_objects) {
            float x = tracked.tlwh[0];  
            float y = tracked.tlwh[1];
            float w = tracked.tlwh[2];
            float h = tracked.tlwh[3];

            // 位置解算
            std::map<std::string, std::vector<float>> res;
            std::vector<float> uv = {x + w, y + h};
            std::vector<float> euler_camera = {0.0, 1.309, 0.0};
            float height = cur_aair.height;
            std::vector<float> euler_drone = {cur_aair.roll, cur_aair.pitch, cur_aair.yaw};
            std::vector<float> position_drone = {cur_aair.lat, cur_aair.lng};
            std::map<std::string, std::vector<float>> result;
            result = geo_location.get_target_location(uv, height, euler_camera, euler_drone, position_drone); 

            // 记录GPS数据到日志
            logger->info("GPS Coordinates: Latitude: {:.6f}, Longitude: {:.6f}, Altitude: {:.2f}",
                         result["gps"][0], result["gps"][1], result["gps"][2]);
            // aair_receiver.sendGpsData(latitude, longitude, altitude);  // 发送 GPS 数据
            // 可视化
            const unsigned char* color = colors[tracked.track_id % 19];
            cv::Scalar cc(color[0], color[1], color[2]);

            char text[256];
            sprintf(text, "ID:%d %.1f%% %s", tracked.track_id, tracked.score * 100, coco_cls_to_name(tracked.label));

            cv::Rect target_rect(x, y, w, h);
            cv::rectangle(frame, target_rect, cc, 2);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int tx = x;
            int ty = y - label_size.height - baseLine;
            if (ty < 0) ty = 0;
            if (tx + label_size.width > frame.cols) tx = frame.cols - label_size.width;

            // 绘制文本背景矩形
            cv::rectangle(frame, cv::Rect(cv::Point(tx, ty), cv::Size(label_size.width, label_size.height + baseLine)), cc, -1);
            // 绘制文本
            cv::putText(frame, text, cv::Point(tx, ty + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }

        cv::imshow("YOLO11 + ByteTrack", frame);

        char c = cv::waitKey(1);
        if (c == 27) { // ESC
            break;
        }
    }

    // 停止接收器并清理资源
    aair_receiver.stop();
    detector.deinit();
    logger->info("AAIRReceiver stopped");

    return 0;
}
