#include <opencv2/opencv.hpp>
#include "detector.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov10s.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov10s.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char* model_path = argv[1];
    const char* device_path = argv[2];

    YoloDetector detector;

    int ret = detector.init(model_path);
    if (ret != 0) {
        printf("init_yolo11_model fail! ret=%d model_path=%s\n", ret, model_path);
        return -1;
    }

    cv::VideoCapture cap;
    if (isdigit(device_path[0])) {
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

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            printf("cap read frame fail!\n");
            break;
        }
        // 获取当前时间
        int64_t start_time = cv::getTickCount();

        object_detect_result_list od_results;
        ret = detector.infer(frame, od_results);
        if (ret != 0) {
            printf("inference fail! ret=%d\n", ret);
            break;
        }

        
        // 计算推理时间
        int64_t end_time = cv::getTickCount();
        double inference_time = (end_time - start_time) / cv::getTickFrequency();
        printf("Inference time: %.4f seconds\n", inference_time);

        detector.drawDetection(frame, od_results);

        cv::imshow("yolo11", frame);

        char c = cv::waitKey(1);
        if (c == 27) {  // ESC
            break;
        }
    }

    detector.deinit();

    return 0;
}
