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

int YoloDetector::infer(cv::Mat& frame, std::vector<Object>& objects) {
    // cv::Mat image;
    // cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);

    src_image.width = frame.cols;
    src_image.height = frame.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.virt_addr = (unsigned char*)frame.data;
    object_detect_result_list od_results;
    int ret =  inference_yolo11_model(&rknn_app_ctx, &src_image, &od_results);
    // 将检测结果转换为ByteTrack格式
    objects.clear();  // 清空之前的对象
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

    return 0;
}

void YoloDetector::drawDetection(cv::Mat& frame, const STrack& tracked) {
    const unsigned char* color = colors[tracked.track_id % 19];
    cv::Scalar cc(color[0], color[1], color[2]);
    char text[256];
    sprintf(text, "ID:%d %.1f%% %s", tracked.track_id, tracked.score * 100, coco_cls_to_name(tracked.label));


    cv::rectangle(frame, cv::Rect(tracked.tlwh[0], tracked.tlwh[1], tracked.tlwh[2], tracked.tlwh[3]), cc, 2);

    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    int x = tracked.tlwh[0];
    int y = tracked.tlwh[1] - label_size.height - baseLine;
    if (y < 0) y = 0;
    if (x + label_size.width > frame.cols) x = frame.cols - label_size.width;

    cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), cc, -1);
    cv::putText(frame, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
}
