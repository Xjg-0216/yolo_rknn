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

// #define GIMBAL_ENABLED  // 云台相关代码使能开关，注释禁用


/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("%s <model path>\n", argv[0]);

        return -1;
    }

    const char *model_path = argv[1];
    const char *device_path = "/dev/video22";



    // 创建日志文件和控制台输出的sink
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/track_log.txt", true);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // 合并两个sink
    std::vector<spdlog::sink_ptr> sinks = {file_sink, console_sink};
    // 创建logger并指定使用两个sinks
    auto logger = std::make_shared<spdlog::logger>("main_stream_logger", sinks.begin(), sinks.end());
    // 设置日志级别
    logger->set_level(spdlog::level::info);
    // 输出初始化信息
    logger->info("Starting the yolo11+bytetrack");
    logger->info("Model path: {}", model_path);
    logger->info("Device path: {}", device_path);



    int ret;
    TIMER timer;
    cv::Mat frame, image;

    // yolo 检测算法初始化
    YoloDetector detector;

    ret = detector.init(model_path);
    if (ret != 0) {
        logger->error("init_yolo11_model fail! ret={} model_path={}", ret, model_path);
        return -1;
    }


    // BYTETrack跟踪算法初始化
    BYTETracker tracker(30, 90);
    std::vector<Object> objects;
    

    // 云台初始化
    #ifdef GIMBAL_ENABLED
        GimbalController gimbalController;
        GimbalCalc gimbalCalc(0.5, 0.1, 0.05, 64.0, 48.0, 640, 480);

        if (gimbalController.init_serial() == -1) {
            logger->error("串口初始化失败！");
            return -1;
        }

        gbc_info_t gbc_info={0};
    #endif


    cv::VideoCapture cap(device_path, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('N', 'V', '1', '2'));
    // 摄像头

    if (!cap.isOpened()) {
        logger->error("Error: Could not open camera.");
        return -1;
    }


    // 初始化AAIR接收器， 第一个ip和port是接收的， 第二个ip和Port是发送的
    AAIRReceiver aair_receiver("192.168.1.19", 12345, "192.168.1.19", 23456);
    aair_receiver.start();
    logger->info("AAIRReceiver started with IP 192.168.1.19");

    // 初始化解算类
    DroneObjlocation geo_location = DroneObjlocation();
    // 相机内参
    uint16_t img_width = 640;
    uint16_t img_height = 480;
    float fx = 640;
    float fy = 480;
    float cx = 300;
    float cy = 240;
    geo_location.set_parameter(img_width, img_height, fx, fy, cx, cy); // img_width, img_height, fx, fy, cx, cy
    logger->info("img_width: {}, img_height: {}, fx: {}, fy: {}, cx: {}, cy: {}", img_width, img_height, fx, fy, cx, cy);


    float pitch = 0; 
    float yaw = 0;
    while (true) 
    {
        #ifdef GIMBAL_ENABLED
            if (gimbalController.send_gimbal_control_command(pitch, yaw) == -1) {
                logger->error("云台姿态设置失败！");
                return -1;
            }
            if (gimbalController.read_gimbal_status(&gbc_info) == 0) {
            // 打印云台状态信息
            logger->info("固件版本={}, hw_err={}, 倒置标志={}, 云台状态={}",
                    gbc_info.fw_ver, gbc_info.hw_err, gbc_info.inv_flag, gbc_info.gbc_stat);
            float pitch_mtr = gbc_info.mtr_angl[0] * 0.01;
            float yaw_mtr = gbc_info.mtr_angl[1] * 0.01;
            logger->info("pitch={:.1f}, yaw={:.1f}", pitch_mtr, yaw_mtr);
            } else {
                logger->error("读取云台状态失败！");
                return -1;
            }
        #endif

        if (!cap.read(frame)) {  
            logger->error("Error: Could not read frame from the camera or video");
            break;
        }

        // 获取最新的AAIR数据
        bool is_updated;
        AAIR cur_aair = aair_receiver.getCurAAIR(is_updated);
        if (!is_updated)
        {
           logger->info("Waiting for udp data...");
           continue; 
        }

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
            #ifdef GIMBAL_ENABLED
                std::vector<float> euler_camera = {yaw_mtr, pitch_mtr, 0.0};
            #else 
                std::vector<float> euler_camera = {0.0, 1.309, 0.0};
            #endif
            float height = cur_aair.height;
            std::vector<float> euler_drone = {cur_aair.roll, cur_aair.pitch, cur_aair.yaw};
            std::vector<float> position_drone = {cur_aair.lat, cur_aair.lng};
            std::map<std::string, std::vector<float>> result;
            result = geo_location.get_target_location(uv, height, euler_camera, euler_drone, position_drone); 

            logger->info("current drone: roll: {}; pitch: {}; yaw: {}; lat: {}; lng: {}; height: {}", cur_aair.roll, cur_aair.pitch, cur_aair.yaw, cur_aair.lat, cur_aair.lng, height);
            logger->info("current camera: roll: {}; pitch: {}; yaw: {};", euler_camera[0], euler_camera[1], euler_camera[2]);
            logger->info("GPS Coordinates: lat: {:.6f}, lng: {:.6f}, H: {:.2f}",
                         result["gps"][0], result["gps"][1], result["gps"][2]);
            // aair_receiver.sendGpsData(latitude, longitude, altitude);  // 发送 GPS 数据


            // 调整攻击姿态



            // 可视化

            detector.drawDetection(frame, tracked);
        }
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(frame.cols / 2, frame.rows / 2));
        
        cv::imshow("YOLO11 + ByteTrack", resized_frame);

        char c = cv::waitKey(1);
        if (c == 27) { // ESC
            break;
        }

        #ifdef GIMBAL_ENABLED
            // 计算新的云台的角度
            float deltaYaw, deltaPitch;
            gimbalControl.calculate_angle_offset(targetX, targetY, deltaYaw, deltaPitch);

            float dt = 0.1; // 时间间隔 100ms
            gimbalControl.calculate_pid_control(deltaPitch, deltaYaw, dt, pitch, yaw);

            // 打印PID计算结果
            printf("Pitch Command: %f, Yaw Command: %f\n", pitch, yaw);
        #endif


    }

    // 停止接收器并清理资源
    aair_receiver.stop();
    detector.deinit();
    logger->info("AAIRReceiver stopped");

    return 0;
}
