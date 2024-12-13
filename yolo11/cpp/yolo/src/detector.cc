#include "detector.h"

static const unsigned char colors[19][3] = {
    {54, 67, 244}, {99, 30, 233}, {176, 39, 156}, {183, 58, 103}, {181, 81, 63},
    {243, 150, 33}, {244, 169, 3}, {212, 188, 0}, {136, 150, 0}, {80, 175, 76},
    {74, 195, 139}, {57, 220, 205}, {59, 235, 255}, {7, 193, 255}, {0, 152, 255},
    {34, 87, 255}, {72, 85, 121}, {158, 158, 158}, {139, 125, 96}
};

YoloDetector::YoloDetector() : rknn_app_ctx(), src_image() {}

YoloDetector::~YoloDetector() {
    deinit();
}

int YoloDetector::init(const char* model_path) {
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));
    init_post_process();
    return init_yolo11_model(model_path, &rknn_app_ctx);
}

void YoloDetector::deinit() {
    deinit_post_process();
    release_yolo11_model(&rknn_app_ctx);
    if (src_image.virt_addr != NULL) {
        free(src_image.virt_addr);
    }
}

int YoloDetector::infer(cv::Mat& frame, object_detect_result_list& od_results) {
    cv::Mat image;
    cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);

    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.virt_addr = (unsigned char*)image.data;

    return inference_yolo11_model(&rknn_app_ctx, &src_image, &od_results);
}

void YoloDetector::drawDetection(cv::Mat& frame, object_detect_result_list& od_results) {
    int color_index = 0;
    char text[256];
    for (int i = 0; i < od_results.count; i++) {
        const unsigned char* color = colors[color_index % 19];
        cv::Scalar cc(color[0], color[1], color[2]);
        color_index++;

        object_detect_result* det_result = &(od_results.results[i]);
        sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);

        cv::rectangle(frame, cv::Rect(cv::Point(det_result->box.left, det_result->box.top),
                                     cv::Point(det_result->box.right, det_result->box.bottom)), cc, 2);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = det_result->box.left;
        int y = det_result->box.top - label_size.height - baseLine;
        if (y < 0) y = 0;
        if (x + label_size.width > frame.cols) x = frame.cols - label_size.width;

        cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), cc, -1);
        cv::putText(frame, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
    }
}
