#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "yolov5.h"
#include "easy_timer.h"
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

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov5s.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov5s.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_name = argv[2];

    int ret;
    TIMER timer;
    cv::Mat image, frame;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;
    object_detect_result_list od_results;

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));

    BYTETracker tracker;
    std::vector<Object> objects;

    cv::VideoCapture cap;
    if (isdigit(device_name[0])) {
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);
        if (!cap.isOpened()) {
            printf("Error: Could not open camera.\n");
            return -1;
        }
    } else {
        cap.open(argv[2]);
        if (!cap.isOpened()) {  
            printf("Error: Could not open video file.\n");
            return -1;
        }
    }

    // 初始化
    init_post_process();
#ifndef ENABLE_ZERO_COPY
    ret = init_yolov5_model(model_path, &rknn_app_ctx);
#else
    ret = init_yolov5_zero_copy_model(model_path, &rknn_app_ctx);
#endif
    if (ret != 0)
    {
        printf("init yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    // 推理，画框，显示
    while(true) {
        gettimeofday(&start_time, NULL);

        if (!cap.read(frame)) {  
            printf("cap read frame fail!\n");
            break;  
        }  

        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        src_image.width  = image.cols;
        src_image.height = image.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = (unsigned char*)image.data;

        timer.tik();
#ifndef ENABLE_ZERO_COPY
        ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
#else
        ret = inference_yolov5_zero_copy_model(&rknn_app_ctx, &src_image, &od_results);
#endif
        if (ret != 0)
        {
            printf("inference yolov5_model fail! ret=%d\n", ret);
            goto out;
        }
        timer.tok();
        timer.print_time("inference_yolov5_model");

        // 将检测结果转化为ByteTrack格式
        objects.clear(); // 清空上一帧的对象
        for (int i = 0; i < od_results.count; i++) {
            object_detect_result *det_result = &(od_results.results[i]);
            // if (det_result->prop < 0.5) continue; // 过滤低置信度目标

            Object obj;
            obj.label = det_result->cls_id;
            obj.prob = det_result->prop;
            obj.rect = cv::Rect(det_result->box.left, det_result->box.top,
                               det_result->box.right - det_result->box.left,
                               det_result->box.bottom - det_result->box.top);
            objects.push_back(obj);
        }

        // 调用 ByteTrack 更新跟踪信息
        auto tracked_objects = tracker.update(objects);

        // 可视化
        for (const auto& tracked : tracked_objects) {
            const unsigned char* color = colors[tracked.track_id % 19];
            cv::Scalar cc(color[0], color[1], color[2]);

            char text[256];
            // 使用 tracked.score 代替 prob, tracked.tlwh 来获取目标框
            sprintf(text, "ID:%d %.1f%%", tracked.track_id, tracked.score * 100);

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

        cv::imshow("YOLOv5 + ByteTrack Demo", frame);

        char c = cv::waitKey(1);
        if (c == 27) { // ESC
            break;
        }
    }

out:
    deinit_post_process();

#ifndef ENABLE_ZERO_COPY
    ret = release_yolov5_model(&rknn_app_ctx);
#else
    ret = release_yolov5_zero_copy_model(&rknn_app_ctx);
#endif
    if (ret != 0)
    {
        printf("release yolov5_model fail! ret=%d\n", ret);
    }

    return 0;
}
