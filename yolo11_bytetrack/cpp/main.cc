// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "yolo11.h"
#include "image_utils.h"
#include "file_utils.h"
#include "BYTETracker.h"
#include "easy_timer.h"
#include "aair.h"
#include "detector.h"
#include "drone_objlocation.h"
#include "udp_data.h"
#include <opencv2/opencv.hpp>

static const unsigned char colors[19][3] = {
    {54, 67, 244},
    {99, 30, 233},
    {176, 39, 156},
    {183, 58, 103},
    {181, 81, 63},
    {243, 150, 33},
    {244, 169, 3},
    {212, 188, 0},
    {136, 150, 0},
    {80, 175, 76},
    {74, 195, 139},
    {57, 220, 205},
    {59, 235, 255},
    {7, 193, 255},
    {0, 152, 255},
    {34, 87, 255},
    {72, 85, 121},
    {158, 158, 158},
    {139, 125, 96}
};




/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov11.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov11.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_path = argv[2];

    int ret;
    TIMER timer;
    cv::Mat frame, image;
    BYTETracker tracker(30, 90);
    std::vector<Object> objects;

    YoloDetector detector;

    ret = detector.init(model_path);
    if (ret != 0) {
        printf("init_yolo11_model fail! ret=%d model_path=%s\n", ret, model_path);
        return -1;
    }

    cv::VideoCapture cap;
    // 摄像头
    if (isdigit(device_path[0])) {
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);

        if (!cap.isOpened()) {
            printf("Error: Could not open camera.\n");
            return -1;
        }
    } else {
        // 视频文件或者其他
        cap.open(argv[2]);
        if (!cap.isOpened()) {  
            printf("Error: Could not open video file.\n");
            return -1;
        }
    }

    // 初始化AAIR接收器
    AAIRReceiver aair_receiver("192.168.1.19", 12345);
    aair_receiver.start();

    //初始化解算类
    DroneObjlocation geo_location = DroneObjlocation();
    geo_location.set_parameter((uint16_t)640, (uint16_t)480, 640, 640, 300, 240); // img_width, img_height, fx, fy, cx, cy

    while(true) 
    {
        if (!cap.read(frame)) {  
            printf("cap read frame fail!\n");
            break;
        }


        // 获取最新的AAIR数据
        AAIR cur_aair = aair_receiver.getLatestAAIR();

        timer.tik();
        // rknn目标检测
        ret = detector.infer(frame, objects);
        if (ret != 0) {
            printf("inference fail! ret=%d\n", ret);
            break;
        }

        // 调用 ByteTrack 更新跟踪信息
        vector<STrack> tracked_objects = tracker.update(objects);


        timer.tok();
        timer.print_time("yolo11_bytetrack");


        for (const auto& tracked : tracked_objects) {

            float x = tracked.tlwh[0];  
            float y = tracked.tlwh[1];
            float w = tracked.tlwh[2];
            float h = tracked.tlwh[3];

            //位置解算
            std::map<std::string, std::vector<float>> res;
            std::vector<float> uv = {x + w, y + h};
            std::vector<float> euler_camera = {0.0, 1.309, 0.0};
            float height = cur_aair.height;
            std::vector<float> euler_drone = {cur_aair.roll, cur_aair.pitch, cur_aair.yaw};
            std::vector<float> position_drone = {cur_aair.lat, cur_aair.lng};
            std::map<std::string, std::vector<float>> result;
            result = geo_location.get_target_location(uv, height, euler_camera, euler_drone, position_drone); 
            printf("gps: %f, %f, %f\n", result["gps"][0], result["gps"][1], result["gps"][2]);

            // 可视化
            const unsigned char* color = colors[tracked.track_id % 19];
            cv::Scalar cc(color[0], color[1], color[2]);

            char text[256];
            // 使用 tracked.score 代替 prob, tracked.tlwh 来获取目标框
            // sprintf(text, "ID:%d %.1f%%", tracked.track_id, tracked.score * 100);
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

    aair_receiver.stop();
    detector.deinit();

    return 0;
}
