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

#include "yolo11.h"
#include "image_utils.h"
#include "file_utils.h"
#include "BYTETracker.h"
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
        printf("Usage: %s  yolov10s.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov10s.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_path = argv[2];

    int ret;
    cv::Mat frame, image;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));

    BYTETracker tracker(30, 90);
    std::vector<Object> objects;
    cv::VideoCapture cap;
    // 摄像头
    if (isdigit(device_path[0])) {
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);

        if (!cap.isOpened()) {
            printf("Error: Could not open camera.\n");
            return -1;
        }
        // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);    // 设置宽度
        // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);  // 设置长度
    } else {
        // 视频文件或者其他
        cap.open(argv[2]);
        if (!cap.isOpened()) {  
            printf("Error: Could not open video file.\n");
            return -1;
        }
    }

    init_post_process();
    ret = init_yolo11_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolo11_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    while(true) {
        if (!cap.read(frame)) {  
            printf("cap read frame fail!\n");
            break;
        }

        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        src_image.width  = image.cols;
        src_image.height = image.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = (unsigned char*)image.data;

        // rknn推理和处理
        object_detect_result_list od_results;
        ret = inference_yolo11_model(&rknn_app_ctx, &src_image, &od_results);
        if (ret != 0)
        {
            printf("init_yolo11_model fail! ret=%d\n", ret);
            goto out;
        }

        // 将检测结果转化为ByteTrack格式
        objects.clear(); // 清空上一帧的对象
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result *det_result = &(od_results.results[i]);

            Object obj;
            obj.label = det_result->cls_id;
            obj.prob = det_result->prop;
            obj.rect = cv::Rect(det_result->box.left, det_result->box.top,
                               det_result->box.right - det_result->box.left,
                               det_result->box.bottom - det_result->box.top);
            objects.push_back(obj);
        }

        // 调用 ByteTrack 更新跟踪信息
        vector<STrack> tracked_objects = tracker.update(objects);

        // 可视化
        for (const auto& tracked : tracked_objects) {
            const unsigned char* color = colors[tracked.track_id % 19];
            cv::Scalar cc(color[0], color[1], color[2]);

            char text[256];
            // 使用 tracked.score 代替 prob, tracked.tlwh 来获取目标框
            // sprintf(text, "ID:%d %.1f%%", tracked.track_id, tracked.score * 100);
            sprintf(text, "ID:%d %.1f%% %s", tracked.track_id, tracked.score * 100, coco_cls_to_name(tracked.label));

            // 使用 tlwh 来绘制框
            float x = tracked.tlwh[0];  
            float y = tracked.tlwh[1];
            float w = tracked.tlwh[2];
            float h = tracked.tlwh[3];
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

        cv::imshow("YOLO11 + ByteTrack Demo", frame);

        char c = cv::waitKey(1);
        if (c == 27) { // ESC
            break;
        }
    }

out:
    deinit_post_process();

    ret = release_yolo11_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolo11_model fail! ret=%d\n", ret);
    }

    if (src_image.virt_addr != NULL)
    {
        free(src_image.virt_addr);
    }

    return 0;
}
