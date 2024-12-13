#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/opencv.hpp>
#include "yolo11.h"
#include "image_utils.h"
#include "file_utils.h"
#include "../../aair.h"

// struct Object
// {
//     cv::Rect_<float> rect;
//     int label;
//     float prob;
// };

// YOLO Detector Class
class YoloDetector {
public:
    YoloDetector();
    ~YoloDetector();

    // 初始化模型
    int init(const char* model_path);

    // 释放资源
    void deinit();

    // 推理并返回结果
    int infer(cv::Mat& frame, std::vector<Object>& objects);

    // 绘制检测框
    void drawDetection(cv::Mat& frame, object_detect_result_list& od_results);

private:
    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;
};

#endif // DETECTOR_H
